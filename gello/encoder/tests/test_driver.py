import threading
import time

import numpy as np
import pytest

from gello.encoder.driver import (
    RAD_PER_COUNT,
    EncoderDriver,
    EncoderError,
    FakeEncoderDriver,
    fold,
    parse_frame,
    parse_status,
)


def frame_line(seq, counts, ok=0xFF, buttons=0, t_us=1000):
    body = f"D,{seq},{t_us},{ok:02X},{buttons:02X}," + ",".join(str(c) for c in counts)
    x = 0
    for b in body.encode():
        x ^= b
    return f"{body}*{x:02X}".encode()


class FakeESP32:
    """Talks like firmware/as5600_leader: a frame every 5 ms, a status line now and then, and
    replies to commands, but drops the first reply as the real one does while its buffer is
    full of backlog."""

    def __init__(self, port, baudrate, timeout):
        self.port, self.timeout = port, timeout
        self.lock = threading.Lock()
        self.counts = [0] * 8
        self.ok = 0xFF
        self.present = 0xFF
        self.seq = 0
        self.skip = 0  # frames to leave out (as if lost)
        self.streaming = True
        self.out = b""
        self.written = b""
        self.replies_dropped = 1
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.out)

    def read(self, n):
        time.sleep(0.005)
        with self.lock:
            if self.streaming:
                self.seq += 1 + self.skip
                self.skip = 0
                self.out += frame_line(self.seq, self.counts, self.ok) + b"\n"
                if self.seq % 20 == 0:
                    chans = ",".join(
                        "20:64:2000:0" if self.ok >> c & 1 else "00:128:5:0"
                        for c in range(8)
                    )
                    self.out += (
                        f"S,{self.seq},998,0,0,{self.present:02X},{chans}\n".encode()
                    )
            data, self.out = self.out[:n], self.out[n:]
        return data

    def write(self, data):
        with self.lock:
            self.written += data
            if b"?" in data:
                if self.replies_dropped > 0:
                    self.replies_dropped -= 1
                else:
                    self.out += b"I,fw=1,protocol=1,channels=8,cpr=4096,rate_hz=200\n"

    def close(self):
        self.closed = True


@pytest.fixture
def esp():
    made = []

    def factory(*args, **kwargs):
        made.append(FakeESP32(*args, **kwargs))
        return made[-1]

    factory.made = made
    return factory


def test_parse_frame_and_checksum():
    f = parse_frame(
        frame_line(7, [1, 2, 3, 4, 5, 6, 7, -4096], ok=0x05, buttons=0x02, t_us=99)
    )
    assert (f.seq, f.t_us, f.ok, f.buttons) == (7, 99, 0x05, 0x02)
    assert f.counts == [1, 2, 3, 4, 5, 6, 7, -4096]
    corrupted = frame_line(7, [1, 2, 3, 4, 5, 6, 7, 8]).replace(b",3,", b",4,")
    assert parse_frame(corrupted) is None
    assert parse_frame(b"D,1,2,FF,00,1,2") is None  # no checksum
    assert parse_frame(b"# ch0: no encoder") is None


def test_parse_status():
    st = parse_status(
        b"S,1000,995,3,0,04,"
        + b",".join([b"00:0:0:0"] * 2 + [b"30:128:1300:1"] + [b"00:0:0:0"] * 5)
    )
    assert (st.uptime_ms, st.passes_per_s, st.frames_dropped, st.present) == (
        1000,
        995,
        3,
        0x04,
    )
    assert st.channels[2].magnet == "too weak" and st.channels[2].errors == 1
    assert st.channels[0].magnet == "not detected"


def test_fold_keeps_a_single_turn_joint_near_its_center():
    assert fold(np.deg2rad(350), 0.0) == pytest.approx(np.deg2rad(-10))
    assert fold(np.deg2rad(10 + 3 * 360), np.deg2rad(20)) == pytest.approx(
        np.deg2rad(10)
    )


def test_driver_reads_joints_in_channel_order(esp):
    d = EncoderDriver([2, 0], serial_factory=esp)
    try:
        dev = esp.made[0]
        assert d.info["protocol"] == "1"  # asked again after the first reply was lost
        with dev.lock:
            dev.counts[0], dev.counts[2] = 1024, -4096 - 2048
        time.sleep(0.05)
        np.testing.assert_allclose(d.get_joints(), [-1.5 * 2 * np.pi, np.pi / 2])
        assert d.torque_enabled() is False
        with pytest.raises(NotImplementedError):
            d.set_joints([0, 0])
    finally:
        d.close()
    assert dev.closed


def test_single_turn_channel_reads_near_its_center(esp):
    d = EncoderDriver([0, 6], serial_factory=esp, single_turn={6: np.deg2rad(30)})
    try:
        with esp.made[0].lock:
            esp.made[0].counts[6] = 4096 * 5 + 512  # 45 deg plus five turns
        time.sleep(0.05)
        assert np.rad2deg(d.get_joints()[1]) == pytest.approx(45.0)
    finally:
        d.close()


def test_a_channel_that_stops_reading_fails_after_stale_s(esp):
    d = EncoderDriver([0, 1], serial_factory=esp, stale_s=0.1)
    try:
        dev = esp.made[0]
        d.get_joints()
        with dev.lock:
            dev.ok = 0xFF & ~0x02  # channel 1: the magnet is gone
        time.sleep(0.05)
        d.get_joints()  # still within stale_s
        time.sleep(0.2)
        with pytest.raises(
            EncoderError, match=r"channel\(s\) \[1\].*magnet not detected"
        ):
            d.get_joints()
        with dev.lock:
            dev.ok = 0xFF
        time.sleep(0.05)
        d.get_joints()  # back
    finally:
        d.close()


def test_a_silent_stream_fails(esp):
    d = EncoderDriver([0], serial_factory=esp, stale_s=0.1)
    try:
        with esp.made[0].lock:
            esp.made[0].streaming = False
        time.sleep(0.2)
        with pytest.raises(EncoderError, match="no frames"):
            d.get_joints()
    finally:
        d.close()


def test_missed_frames_are_counted(esp):
    d = EncoderDriver([0], serial_factory=esp)
    try:
        with esp.made[0].lock:
            esp.made[0].skip = 3
        time.sleep(0.05)
        assert d.frames_missed == 3
    finally:
        d.close()


def test_no_firmware_is_an_error(esp):
    def silent(*args, **kwargs):
        dev = FakeESP32(*args, **kwargs)
        dev.streaming = False
        return dev

    with pytest.raises(EncoderError, match="is firmware/as5600_leader flashed"):
        EncoderDriver([0], serial_factory=silent, wait_s=0.3)


def test_fake_driver():
    d = FakeEncoderDriver([0, 1])
    d.set_fake_joints([0.5, -0.5])
    np.testing.assert_allclose(d.get_joints(), [0.5, -0.5])


def test_encoder_robot_config_offsets_signs_gripper_and_turns(monkeypatch):
    serial = pytest.importorskip("serial")
    from gello.agents.gello_agent import EncoderRobotConfig

    made = []

    def factory(*args, **kwargs):
        made.append(FakeESP32(*args, **kwargs))
        dev = made[-1]
        # The calibration pose, one turn off on joint 0 (the ESP32 restarted elsewhere), and
        # the trigger halfway between open (10 deg) and closed (70 deg).
        dev.counts[:3] = [4096 + 1024, 2048, int(40 / 360 * 4096)]
        return dev

    monkeypatch.setattr(serial, "Serial", factory)
    cfg = EncoderRobotConfig(
        channels=(0, 1),
        joint_offsets=(np.pi / 2, np.pi),
        joint_signs=(1, -1),
        gripper_config=(2, 10.0, 70.0),
    )
    robot = cfg.make_robot(port="/dev/fake", start_joints=np.array([0.0, 0.0, 0.0]))
    try:
        q = robot.get_joint_state()
        np.testing.assert_allclose(q[:2], [0.0, 0.0], atol=1e-9)  # the turn is resolved
        assert q[2] == pytest.approx(0.5, abs=0.01)
        with made[0].lock:
            made[0].counts[1] = (
                2048 + 1024
            )  # joint 1 turns +90 deg raw, so -90 deg with its sign
        time.sleep(0.05)
        robot._last_pos = None  # no smoothing for the test
        assert robot.get_joint_state()[1] == pytest.approx(
            -np.pi / 2, abs=RAD_PER_COUNT
        )
    finally:
        robot._driver.close()
