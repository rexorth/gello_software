"""Joint angles from a GELLO built with AS5600 magnetic encoders instead of Dynamixels.

The encoders sit behind a TCA9548A I2C multiplexer, read by an ESP32 running
firmware/as5600_leader, which streams every channel over USB serial. This driver reads that
stream on a background thread. It stands in for DynamixelDriver: get_joints() returns radians,
and the joints are passive (no torque, nothing to command).

Each reading is the encoder's absolute angle at power-up plus the whole turns tracked since, so
like a Dynamixel's present position it is only known up to a multiple of 2*pi across power
cycles. DynamixelRobot resolves that against start_joints.

The serial protocol, one line each (README in firmware/as5600_leader):
    D,<seq>,<t_us>,<ok>,<buttons>,<count0>,...,<count7>*<xor>   a frame, 200 per second
    S,<uptime_ms>,<passes/s>,<dropped>,<bus resets>,<present>,<status>:<agc>:<magnitude>:<errors>,...
    I,fw=1,protocol=1,...                                          the reply to "?"
    # ...                                                          for people
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

log = logging.getLogger(__name__)

COUNTS_PER_REV = 4096
RAD_PER_COUNT = 2 * np.pi / COUNTS_PER_REV
PROTOCOL_VERSION = 1
BACKLOG_S = 0.1  # on opening, drop what arrives this soon: the ESP32's buffered backlog

# AS5600 STATUS bits
MAGNET_TOO_STRONG, MAGNET_TOO_WEAK, MAGNET_DETECTED = 0x08, 0x10, 0x20


def fold(angle: float, center: float) -> float:
    """angle plus or minus whole turns, within half a turn of center."""
    return center + (angle - center + np.pi) % (2 * np.pi) - np.pi


class EncoderError(RuntimeError):
    """No fresh reading: the stream stopped, or an encoder isn't reading."""


@dataclass
class Frame:
    seq: int
    t_us: int  # the ESP32's clock when the encoders were read
    ok: int  # bit i: channel i read, with its magnet detected
    buttons: int  # bit i: button i held
    counts: List[int]  # per channel: 4096 per turn, whole turns tracked since power-up


@dataclass
class ChannelStatus:
    status: int  # AS5600 STATUS register
    agc: int  # automatic gain: high = weak field, low = strong
    magnitude: int
    errors: int  # failed reads since power-up

    @property
    def magnet(self) -> str:
        if not self.status & MAGNET_DETECTED:
            return "not detected"
        if self.status & MAGNET_TOO_WEAK:
            return "too weak"
        if self.status & MAGNET_TOO_STRONG:
            return "too strong"
        return "ok"


@dataclass
class Status:
    uptime_ms: int
    passes_per_s: int  # how often the ESP32 reads every channel
    frames_dropped: int  # frames the ESP32 couldn't send: nobody was reading
    bus_resets: int
    present: int  # bit i: an encoder answers on channel i
    channels: List[ChannelStatus]


def _checksum_ok(line: bytes) -> Optional[bytes]:
    body, star, cs = line.rpartition(b"*")
    if not star or len(cs) != 2:
        return None
    x = 0
    for b in body:
        x ^= b
    try:
        return body if x == int(cs, 16) else None
    except ValueError:
        return None


def parse_frame(line: bytes) -> Optional[Frame]:
    """A D line, or None if it isn't one or its checksum fails."""
    if not line.startswith(b"D,"):
        return None
    body = _checksum_ok(line)
    if body is None:
        return None
    f = body.split(b",")
    try:
        return Frame(
            int(f[1]), int(f[2]), int(f[3], 16), int(f[4], 16), [int(v) for v in f[5:]]
        )
    except (IndexError, ValueError):
        return None


def parse_status(line: bytes) -> Optional[Status]:
    if not line.startswith(b"S,"):
        return None
    f = line.split(b",")
    try:
        channels = []
        for c in f[6:]:
            st, agc, mag, err = c.split(b":")
            channels.append(ChannelStatus(int(st, 16), int(agc), int(mag), int(err)))
        return Status(
            int(f[1]), int(f[2]), int(f[3]), int(f[4]), int(f[5], 16), channels
        )
    except (IndexError, ValueError):
        return None


def parse_info(line: bytes) -> Optional[Dict[str, str]]:
    if not line.startswith(b"I,"):
        return None
    out = {}
    for item in line[2:].decode(errors="replace").split(","):
        key, _, value = item.partition("=")
        out[key] = value
    return out


@dataclass
class Reading:
    """The latest frame, as the host saw it."""

    frame: Frame
    t: float  # host time.monotonic() when it arrived
    good_since: Dict[int, float] = field(
        default_factory=dict
    )  # channel -> last host time it was ok

    def age(self, now: Optional[float] = None) -> float:
        return (time.monotonic() if now is None else now) - self.t


class EncoderDriver:
    """Reads the AS5600 leader firmware's stream. The DynamixelDriver calls that make sense for
    a passive arm work the same; set_joints and torque raise.

    channels: the multiplexer channel of each joint, in joint order.
    stale_s: get_joints() raises EncoderError once a joint's last good reading is older.
    single_turn: channel -> angle (rad): joints that never turn a full revolution, like the
        gripper trigger, read within half a turn of that angle (say the middle of their travel)
        whatever whole turns the count holds.
    """

    def __init__(
        self,
        channels: Sequence[int],
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        rate_hz: Optional[int] = None,
        stale_s: float = 0.1,
        single_turn: Optional[Dict[int, float]] = None,
        serial_factory: Optional[Callable[..., object]] = None,
        wait_s: float = 2.0,
    ):
        if serial_factory is None:
            import serial  # pyserial

            serial_factory = serial.Serial
        self.channels = list(channels)
        self.port = port
        self.stale_s = stale_s
        self.single_turn = dict(single_turn or {})
        self.info: Optional[Dict[str, str]] = None
        self.messages: List[str] = []  # the firmware's "#" lines, latest last
        self._lock = threading.Lock()
        self._reading: Optional[Reading] = None
        self._status: Optional[Status] = None
        self._good_since: Dict[int, float] = {}
        self._bad_lines = 0
        self._error: Optional[BaseException] = None
        self._stop = threading.Event()
        self.frames = 0  # frames received
        self.frames_missed = (
            0  # frames the firmware sent that never arrived (gaps in seq)
        )
        self._last_seq: Optional[int] = None
        self._serial = serial_factory(port, baudrate, timeout=0.05)
        self._thread = threading.Thread(
            target=self._run, name="gello-encoders", daemon=True
        )
        self._thread.start()
        self._start(rate_hz, wait_s)

    # ---- DynamixelDriver-compatible ----

    def get_joints(self) -> np.ndarray:
        """Joint angles in radians, in `channels` order. Raises EncoderError if a joint has no
        reading from the last stale_s."""
        r = self.reading()
        now = time.monotonic()
        if r is None or r.age(now) > self.stale_s:
            raise EncoderError(
                f"no frames from the encoders on {self.port} for {self.stale_s} s"
            )
        stale = [
            c
            for c in self.channels
            if now - r.good_since.get(c, -np.inf) > self.stale_s
        ]
        if stale:
            raise EncoderError(f"encoder channel(s) {stale}: {self.describe(stale)}")
        q = (
            np.array([r.frame.counts[c] for c in self.channels], dtype=float)
            * RAD_PER_COUNT
        )
        for i, c in enumerate(self.channels):
            if c in self.single_turn:
                q[i] = fold(q[i], self.single_turn[c])
        return q

    def set_joints(self, joint_angles: Sequence[float]) -> None:
        raise NotImplementedError(
            "magnetic encoders are passive: there is nothing to drive"
        )

    def set_torque_mode(self, enable: bool) -> None:
        if enable:
            raise NotImplementedError(
                "magnetic encoders are passive: there is no torque"
            )

    def torque_enabled(self) -> bool:
        return False

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._serial.close()
        except Exception:
            pass

    # ---- more ----

    def reading(self) -> Optional[Reading]:
        """The latest frame (counts of every channel, ok bits, buttons), or None before the
        first. Raises the reader thread's error if the port failed."""
        with self._lock:
            if self._error is not None:
                raise EncoderError(f"{self.port}: {self._error}") from self._error
            return self._reading

    def status(self) -> Optional[Status]:
        """The firmware's once-a-second status line: rate, magnet state, error counts."""
        with self._lock:
            return self._status

    def describe(self, channels: Sequence[int]) -> str:
        """Why these channels aren't reading, from the latest status line."""
        st = self.status()
        parts = []
        for c in channels:
            if st is None or c >= len(st.channels):
                parts.append(f"ch{c} not reading")
            elif not st.present >> c & 1:
                parts.append(f"ch{c} no encoder answering (wiring or power)")
            else:
                parts.append(f"ch{c} magnet {st.channels[c].magnet}")
        return ", ".join(parts)

    def command(self, text: str) -> None:
        """Send a firmware command: "?", "scan", "bus", "rate <hz>", "i2c <hz>"."""
        self._serial.write(text.encode() + b"\n")

    # ---- reader thread ----

    def _start(self, rate_hz: Optional[int], wait_s: float) -> None:
        """Ask for the firmware's info (and set the rate), then wait for its first frame. The
        reader drops the backlog the ESP32 kept while nobody read it, which also crowds out
        replies: ask until one comes."""
        deadline = time.monotonic() + wait_s
        asked = 0.0
        while time.monotonic() < deadline:
            if self.info is None and time.monotonic() - asked > 0.2:
                asked = time.monotonic()
                self.command("?")
                if rate_hz is not None:
                    self.command(f"rate {int(rate_hz)}")
            if self.info is not None and self.reading() is not None:
                return
            time.sleep(0.01)
        self.close()
        raise EncoderError(
            f"no frames from {self.port} in {wait_s} s: is firmware/as5600_leader flashed?"
        )

    def _run(self) -> None:
        buf = b""
        try:
            # The backlog the ESP32 buffered while the port was closed: frames from long ago.
            # It comes in one burst on opening.
            drain_until = time.monotonic() + BACKLOG_S
            while not self._stop.is_set() and time.monotonic() < drain_until:
                self._serial.read(self._serial.in_waiting or 1)
            while not self._stop.is_set():
                data = self._serial.read(self._serial.in_waiting or 1)
                if not data:
                    continue
                buf += data
                *lines, buf = buf.split(b"\n")
                for line in lines:
                    self._line(line.strip())
                if len(buf) > 4096:  # no newline in a long time: not this firmware
                    buf = b""
        except Exception as exc:  # unplugged: the port raises
            if not self._stop.is_set():
                log.warning("gello encoders: %s: %s", self.port, exc)
                with self._lock:
                    self._error = exc

    def _line(self, line: bytes) -> None:
        now = time.monotonic()
        frame = parse_frame(line)
        if frame is not None:
            with self._lock:
                self.frames += 1
                if self._last_seq is not None and frame.seq > self._last_seq + 1:
                    self.frames_missed += frame.seq - self._last_seq - 1
                self._last_seq = frame.seq  # it restarts from 0 if the ESP32 resets
                for c in range(len(frame.counts)):
                    if frame.ok >> c & 1:
                        self._good_since[c] = now
                self._reading = Reading(frame, now, dict(self._good_since))
            return
        status = parse_status(line)
        if status is not None:
            with self._lock:
                self._status = status
            return
        info = parse_info(line)
        if info is not None:
            self.info = info
            if info.get("protocol") != str(PROTOCOL_VERSION):
                log.warning(
                    "gello encoders: firmware protocol %s, this driver speaks %d: reflash "
                    "firmware/as5600_leader",
                    info.get("protocol"),
                    PROTOCOL_VERSION,
                )
            return
        if line.startswith(b"#"):
            text = line[1:].decode(errors="replace").strip()
            self.messages = (self.messages + [text])[-50:]
            log.debug("gello encoders: %s", text)
        elif line.startswith(b"D,"):
            self._bad_lines += 1


class FakeEncoderDriver:
    """For tests: joints set by hand."""

    def __init__(self, channels: Sequence[int]):
        self.channels = list(channels)
        self._joints = np.zeros(len(self.channels))

    def set_fake_joints(self, joints: Sequence[float]) -> None:
        self._joints = np.asarray(joints, dtype=float).copy()

    def get_joints(self) -> np.ndarray:
        return self._joints.copy()

    def set_joints(self, joint_angles: Sequence[float]) -> None:
        raise NotImplementedError(
            "magnetic encoders are passive: there is nothing to drive"
        )

    def set_torque_mode(self, enable: bool) -> None:
        if enable:
            raise NotImplementedError(
                "magnetic encoders are passive: there is no torque"
            )

    def torque_enabled(self) -> bool:
        return False

    def close(self) -> None:
        pass
