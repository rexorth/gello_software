# AS5600 leader firmware

A GELLO with magnetic encoders instead of Dynamixels: an AS5600 at each joint (and optionally
the trigger), each on its own channel of a TCA9548A I2C multiplexer, read by an ESP32 that
streams them over USB serial. On the host, `gello.encoder.EncoderDriver` reads the stream, and
`EncoderRobotConfig` (in `gello/agents/gello_agent.py`) plugs it into GELLO wherever a
`DynamixelRobotConfig` goes.

Every AS5600 answers at the same fixed address (0x36), hence the multiplexer. The firmware reads
every channel about 1000 times a second and counts whole turns from the change between two reads.
A joint would have to turn half a revolution in 1 ms to fool it. Frames go out 200 times a second.

## Wiring (Seeed XIAO ESP32C3)

| TCA9548A | XIAO |
|---|---|
| VIN | 3V3 |
| GND | GND |
| SDA | D4 (GPIO6) |
| SCL | D5 (GPIO7) |
| RST | 3V3 (or a pin, see below) |
| A0, A1, A2 | GND (address 0x70) |

| AS5600 | to |
|---|---|
| VCC | XIAO 3V3 |
| GND, DIR | XIAO GND |
| SDA, SCL | TCA9548A SDn, SCn: channel n = joint n, base first; the trigger after the last joint |

DIR to GND makes the count rise turning clockwise, looking down at the chip. Which way that is
for a joint depends on how its encoder faces, so each joint's sign is found by watching it
(below). The diametric magnet sits centred over the chip, 0.5 to 3 mm away. The `S` line's
status says whether the field is right.

Optional extras, read without reflashing (`buttons` in each frame):

| pin | use |
|---|---|
| BOOT button | bit 0 |
| D1 (GPIO3) to GND through a push button | bit 1, e.g. a deadman or clutch on the handle |
| D2 (GPIO4) to GND through a push button | bit 2 |
| TCA9548A RST to D3 (GPIO5) | lets the firmware reset the multiplexer if the bus locks up; build with `-DPIN_MUX_RST=5` |

Other boards: set the pins with `-DPIN_SDA=.. -DPIN_SCL=..` (see `platformio.ini`, which has a
generic ESP32 on GPIO18/19).

## Build and flash

No Arduino IDE needed. PlatformIO fetches the toolchain on first use (about 1 GB in
`~/.platformio`):

```bash
uvx --from platformio pio run -d firmware/as5600_leader -t upload    # or: pio run ... if installed
```

## Check it

```bash
uvx --from platformio pio device monitor -d firmware/as5600_leader
```

Type `scan` and press Enter. It prints a line per channel, like `# ch2: AS5600, magnet ok, raw 1234`, or what is wrong:
`no encoder` (wiring or power), or the magnet `not detected`, `too weak / too far` or `too strong / too close`.
`bus` lists every address that answers on each channel, with the idle level of SDA and SCL. That
finds an encoder on the wrong channel, or a line something holds low.

Then, with the arm in GELLO's calibration pose, from the repository root:

```bash
python scripts/gello_get_offset.py --encoder --port /dev/serial/by-id/usb-Espressif_* \
    --start-joints 0 -1.57 1.57 -1.57 -1.57 0 --joint-signs 1 1 -1 1 1 1     # UR
python scripts/gello_get_offset.py --encoder --watch --port ... --joint-signs ... --joint-offsets ...
```

The first command prints the offsets and the trigger's open and closed angles as an
`EncoderRobotConfig` for `PORT_CONFIG_MAP`. The second shows the joints live through those
offsets and signs. Move each joint the way the robot's turns positive, and its value should
rise: if not, flip that joint's sign and run the first command again.

## Serial protocol (version 1)

Lines, `\n`-terminated, any baud rate (it is native USB).

From the ESP32:

| line | meaning |
|---|---|
| `D,<seq>,<t_us>,<ok>,<buttons>,<c0>,...,<c7>*<xx>` | a frame. `seq` counts frames since power-up; `t_us` is when the encoders were read (ESP32 clock, µs); `ok` (hex) has bit n set if channel n read with its magnet detected; `buttons` (hex) as above; `cn` is channel n's position, 4096 counts per turn: its raw angle at power-up plus the turns since. `xx` is the XOR of every byte before the `*`, in hex. |
| `S,<uptime_ms>,<passes/s>,<dropped>,<bus resets>,<present>,<st>:<agc>:<mag>:<err>,...` | once a second. `passes/s`: how often every channel was read; `dropped`: frames not sent because nobody was reading; `present` (hex): channels with an encoder. Per channel: the AS5600 STATUS register (hex: 0x20 magnet detected, 0x10 too weak, 0x08 too strong), its AGC (gain: high = weak field), field magnitude, failed reads. |
| `I,fw=1,protocol=1,channels=8,cpr=4096,...` | the reply to `?` |
| `# ...` | for people: boot banner, scan results, problems |

To the ESP32: `?` (info), `scan`, `bus`, `rate <hz>` (frames per second, 1 to 1000; default 200),
`i2c <hz>` (bus clock; default 400000).

A line is sent whole or not at all. While nothing reads the port, the ESP32 drops frames rather
than wait. Its USB buffer still holds the last few kilobytes, which arrive in a burst when the
port is opened: the driver discards the first 0.1 s. Opening the port doesn't reset the ESP32,
so the turn counts carry over between programs.
