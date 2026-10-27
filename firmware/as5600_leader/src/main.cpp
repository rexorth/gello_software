// GELLO leader arm: AS5600 magnetic encoders, one per TCA9548A channel, streamed over USB serial.
//
// Every AS5600 answers at the same fixed address (0x36), so each sits on its own channel of a
// TCA9548A multiplexer (0x70). The loop reads every channel as fast as the I2C bus allows
// (~1 kHz) and tracks whole turns from the change since the last read: a joint would have to turn
// half a revolution between two reads (1 ms) to fool it. Frames go out at a fixed rate (default
// 200 Hz). The serial protocol is in README.md; the host side is gello/encoder/driver.py.

#include <Arduino.h>
#include <Wire.h>

#define FW_VERSION 1
#define PROTOCOL_VERSION 1

#ifndef PIN_SDA
#define PIN_SDA SDA
#endif
#ifndef PIN_SCL
#define PIN_SCL SCL
#endif
// The multiplexer's RST line, if it goes to a pin rather than to 3V3: pulsed low to recover a
// stuck bus. -1 = tied to 3V3.
#ifndef PIN_MUX_RST
#define PIN_MUX_RST -1
#endif

namespace {

constexpr uint8_t MUX_ADDR = 0x70;  // TCA9548A with A0..A2 to GND
constexpr uint8_t AS5600_ADDR = 0x36;
constexpr uint8_t N_CH = 8;
constexpr uint8_t REG_STATUS = 0x0B;  // then RAW ANGLE high, low
constexpr uint8_t REG_AGC = 0x1A;     // then MAGNITUDE high, low
constexpr uint8_t ST_MH = 0x08, ST_ML = 0x10, ST_MD = 0x20;  // too strong, too weak, detected
constexpr int32_t CPR = 4096;  // counts per revolution

// Buttons, active low with the internal pull-ups; bit i of the frame's button field is
// BUTTON_PINS[i]. On the XIAO: bit 0 the BOOT button, bits 1-2 D1 and D2 to GND.
#if defined(ARDUINO_XIAO_ESP32C3)
const uint8_t BUTTON_PINS[] = {9, D1, D2};
constexpr uint8_t N_BUTTONS = sizeof(BUTTON_PINS);
#else
const uint8_t BUTTON_PINS[1] = {0};
constexpr uint8_t N_BUTTONS = 0;
#endif
constexpr uint32_t DEBOUNCE_US = 5000;

constexpr uint32_t SAMPLE_PERIOD_US = 1000;   // at most 1 kHz: the AS5600 filters over ~2 ms
constexpr uint32_t STATUS_PERIOD_US = 1000000;
constexpr uint32_t PROBE_PERIOD_US = 250000;   // look again for an encoder on an empty channel
constexpr uint32_t MAGNET_PERIOD_US = 100000;  // read one channel's AGC and magnitude

struct Channel {
  bool present = false;  // an AS5600 answered on this channel
  bool seen = false;     // `count` holds a reading
  bool ok = false;       // the last read succeeded with the magnet detected
  uint16_t raw = 0;      // 0..4095
  int32_t count = 0;     // raw plus whole turns since boot
  uint8_t status = 0;
  uint8_t agc = 0;
  uint16_t magnitude = 0;
  uint32_t errors = 0;   // failed reads since boot
};

Channel ch[N_CH];
uint32_t i2c_hz = 400000;
uint32_t frame_period_us = 5000;  // 200 Hz
uint32_t seq = 0;
uint32_t frames_dropped = 0;  // no room in the USB buffer: nobody reading
uint32_t bus_resets = 0;
uint32_t passes = 0;          // sample passes since the last status line
int64_t t_pass = 0;           // when the last pass started
uint8_t buttons = 0;
uint8_t button_raw = 0;
int64_t button_changed = 0;
uint8_t mux_failures = 0;     // consecutive passes the multiplexer didn't answer
int64_t last_reset = 0;
char cmd[48];
uint8_t cmd_len = 0;

bool selectChannel(uint8_t c) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(uint8_t(1u << c));
  return Wire.endTransmission() == 0;
}

void deselectAll() {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(uint8_t(0));
  Wire.endTransmission();
}

// Read n bytes from register reg of the AS5600 on the selected channel.
bool readRegs(uint8_t reg, uint8_t *out, uint8_t n) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) out[i] = Wire.read();
  return true;
}

bool probe(uint8_t c) {
  if (!selectChannel(c)) return false;
  Wire.beginTransmission(AS5600_ADDR);
  return Wire.endTransmission() == 0;
}

void startBus() {
  Wire.begin(PIN_SDA, PIN_SCL, i2c_hz);
  Wire.setTimeOut(3);  // ms; a stuck bus must not stall the loop for the default 50 ms
}

// Free a bus a device holds low: clock SCL until SDA is released, then reset the multiplexer if
// its RST is wired.
void recoverBus() {
  Wire.end();
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 16 && digitalRead(PIN_SDA) == LOW; i++) {
    digitalWrite(PIN_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(5);
  }
#if PIN_MUX_RST >= 0
  digitalWrite(PIN_MUX_RST, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_MUX_RST, HIGH);
  delayMicroseconds(10);
#endif
  startBus();
  bus_resets++;
}

const char *magnetText(uint8_t st) {
  if (!(st & ST_MD)) return "not detected";
  if (st & ST_ML) return "too weak / too far";
  if (st & ST_MH) return "too strong / too close";
  return "ok";
}

// The whole line goes into the USB buffer or none of it: a half-written line would corrupt the
// next. Returns false if it was dropped (no host reading).
bool emit(const char *line, size_t n) {
  if (Serial.availableForWrite() < int(n)) return false;
  Serial.write(reinterpret_cast<const uint8_t *>(line), n);
  return true;
}

void emitf(const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n > 0) emit(buf, min(size_t(n), sizeof(buf) - 1));
}

void scan() {
  emitf("# scan: TCA9548A at 0x%02X, SDA %d, SCL %d, %lu Hz\n", MUX_ADDR, PIN_SDA, PIN_SCL,
        (unsigned long)i2c_hz);
  for (uint8_t c = 0; c < N_CH; c++) {
    if (!selectChannel(c)) {
      emitf("# ch%u: multiplexer not answering: check its SDA/SCL, VIN and RST\n", c);
      ch[c].present = false;
      continue;
    }
    ch[c].present = probe(c);
    uint8_t b[3];
    if (!ch[c].present) {
      emitf("# ch%u: no encoder\n", c);
    } else if (readRegs(REG_STATUS, b, 3)) {
      emitf("# ch%u: AS5600, magnet %s, raw %u\n", c, magnetText(b[0]),
            ((b[1] << 8) | b[2]) & 0x0FFF);
    } else {
      emitf("# ch%u: AS5600 answered but its registers didn't read\n", c);
    }
  }
  deselectAll();
}

// Every address that answers, on the main bus (all channels off) and on each channel: finds an
// encoder on the wrong channel, on the main bus, or at another address.
void busScan() {
  deselectAll();
  for (int8_t c = -1; c < N_CH; c++) {
    if (c >= 0 && !selectChannel(c)) {
      emitf("# bus ch%d: multiplexer not answering\n", c);
      continue;
    }
    char found[128];
    int n = 0;
    found[0] = '\0';
    int64_t t0 = esp_timer_get_time();
    for (uint8_t a = 0x08; a < 0x78; a++) {
      if (a == MUX_ADDR) continue;
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) n += snprintf(found + n, sizeof(found) - n, " 0x%02X", a);
    }
    // ~1.5 ms on a healthy bus; far longer means transfers time out (a line held low, or a
    // device stretching the clock).
    unsigned long scan_us = (unsigned long)(esp_timer_get_time() - t0);
    // The idle levels with this channel switched through: low means something holds the line,
    // e.g. an unpowered encoder through its protection diodes, or no pull-up on the channel.
    Wire.end();
    pinMode(PIN_SDA, INPUT);
    pinMode(PIN_SCL, INPUT);
    delayMicroseconds(50);
    const char *sda = digitalRead(PIN_SDA) ? "high" : "LOW", *scl = digitalRead(PIN_SCL) ? "high" : "LOW";
    startBus();
    if (c < 0) {
      emitf("# bus main (channels off):%s (SDA %s, SCL %s, scan %lu us)\n",
            n ? found : " nothing besides the multiplexer", sda, scl, scan_us);
    } else {
      emitf("# bus ch%d:%s (SDA %s, SCL %s, scan %lu us)\n", c, n ? found : " nothing", sda, scl, scan_us);
    }
    delay(2);  // let the USB buffer drain
  }
  deselectAll();
}

// False if the multiplexer didn't answer.
bool readChannel(uint8_t c) {
  Channel &k = ch[c];
  uint8_t b[3];
  k.ok = false;
  if (!selectChannel(c)) {
    k.errors++;
    return false;
  }
  if (!readRegs(REG_STATUS, b, 3)) {
    k.errors++;
    return true;
  }
  k.status = b[0];
  if (!(k.status & ST_MD)) return true;  // no magnet: the angle is noise
  k.ok = true;
  uint16_t raw = ((b[1] << 8) | b[2]) & 0x0FFF;
  if (!k.seen) {
    k.count = raw;
    k.seen = true;
  } else {
    int32_t d = int32_t(raw) - int32_t(k.raw);
    if (d > CPR / 2) d -= CPR;
    if (d < -CPR / 2) d += CPR;
    k.count += d;
  }
  k.raw = raw;
  return true;
}

void readMagnet(uint8_t c) {
  uint8_t b[3];
  if (selectChannel(c) && readRegs(REG_AGC, b, 3)) {
    ch[c].agc = b[0];
    ch[c].magnitude = ((b[1] << 8) | b[2]) & 0x0FFF;
  }
}

void readButtons(int64_t now) {
  uint8_t raw = 0;
  for (uint8_t i = 0; i < N_BUTTONS; i++)
    if (digitalRead(BUTTON_PINS[i]) == LOW) raw |= 1u << i;
  if (raw != button_raw) {
    button_raw = raw;
    button_changed = now;
  } else if (raw != buttons && now - button_changed >= DEBOUNCE_US) {
    buttons = raw;
  }
}

uint8_t mask(bool Channel::*field) {
  uint8_t m = 0;
  for (uint8_t c = 0; c < N_CH; c++)
    if (ch[c].*field) m |= 1u << c;
  return m;
}

// D,<seq>,<t_us>,<ok>,<buttons>,<count0>,...,<count7>*<xor>
void sendFrame() {
  char buf[160];
  int n = snprintf(buf, sizeof(buf), "D,%lu,%lld,%02X,%02X", (unsigned long)seq, (long long)t_pass,
                   mask(&Channel::ok), buttons);
  for (uint8_t c = 0; c < N_CH; c++) n += snprintf(buf + n, sizeof(buf) - n, ",%ld", (long)ch[c].count);
  uint8_t x = 0;
  for (int i = 0; i < n; i++) x ^= uint8_t(buf[i]);
  n += snprintf(buf + n, sizeof(buf) - n, "*%02X\n", x);
  seq++;
  if (!emit(buf, n)) frames_dropped++;
}

// S,<uptime_ms>,<passes/s>,<frames dropped>,<bus resets>,<present>, then per channel
// <status>:<agc>:<magnitude>:<errors>
void sendStatus(uint32_t passes_per_s) {
  char buf[256];
  int n = snprintf(buf, sizeof(buf), "S,%lu,%lu,%lu,%lu,%02X", (unsigned long)millis(),
                   (unsigned long)passes_per_s, (unsigned long)frames_dropped,
                   (unsigned long)bus_resets, mask(&Channel::present));
  for (uint8_t c = 0; c < N_CH; c++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%02X:%u:%u:%lu", ch[c].status, ch[c].agc,
                  ch[c].magnitude, (unsigned long)ch[c].errors);
  n += snprintf(buf + n, sizeof(buf) - n, "\n");
  emit(buf, n);
}

void sendInfo() {
  emitf("I,fw=%d,protocol=%d,channels=%u,cpr=%ld,sda=%d,scl=%d,mux_rst=%d,i2c_hz=%lu,rate_hz=%lu,buttons=%u\n",
        FW_VERSION, PROTOCOL_VERSION, N_CH, (long)CPR, PIN_SDA, PIN_SCL, PIN_MUX_RST,
        (unsigned long)i2c_hz, (unsigned long)(1000000UL / frame_period_us), N_BUTTONS);
}

// Commands, one per line: ?  scan  bus  rate <hz>  i2c <hz>
void command(char *line) {
  unsigned long v = 0;
  if (strcmp(line, "?") == 0) {
    sendInfo();
  } else if (strcmp(line, "scan") == 0) {
    scan();
  } else if (strcmp(line, "bus") == 0) {
    busScan();
  } else if (sscanf(line, "rate %lu", &v) == 1 && v >= 1 && v <= 1000) {
    frame_period_us = 1000000UL / v;
    emitf("# rate %lu Hz\n", v);
  } else if (sscanf(line, "i2c %lu", &v) == 1 && v >= 10000 && v <= 1000000) {
    i2c_hz = v;
    Wire.setClock(i2c_hz);
    emitf("# i2c %lu Hz\n", v);
  } else if (line[0]) {
    emitf("# unknown command '%s': ?, scan, bus, rate <hz>, i2c <hz>\n", line);
  }
}

void readCommands() {
  while (Serial.available() > 0) {
    char c = char(Serial.read());
    if (c == '\n' || c == '\r') {
      cmd[cmd_len] = '\0';
      command(cmd);
      cmd_len = 0;
    } else if (cmd_len < sizeof(cmd) - 1) {
      cmd[cmd_len++] = c;
    }
  }
}

}  // namespace

void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);  // the rate is ignored over native USB
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  Serial.setTxTimeoutMs(0);  // never wait for a host that isn't reading
#endif
  for (uint8_t i = 0; i < N_BUTTONS; i++) pinMode(BUTTON_PINS[i], INPUT_PULLUP);
#if PIN_MUX_RST >= 0
  pinMode(PIN_MUX_RST, OUTPUT);
  digitalWrite(PIN_MUX_RST, HIGH);
#endif
  startBus();
  delay(50);
  emitf("# as5600_leader fw %d\n", FW_VERSION);
  scan();
  sendInfo();
}

void loop() {
  static int64_t next_pass = 0, next_frame = 0, next_status = 0, next_probe = 0, next_magnet = 0;
  static uint8_t probe_ch = 0, magnet_ch = 0;

  readCommands();
  int64_t now = esp_timer_get_time();
  if (now < next_pass) return;
  next_pass = now + SAMPLE_PERIOD_US;

  t_pass = now;
  bool mux_ok = true;
  for (uint8_t c = 0; c < N_CH; c++)
    if (ch[c].present) mux_ok &= readChannel(c);
  passes++;
  readButtons(now);
  // A multiplexer that stops answering means a stuck bus: free it, at most once a second.
  mux_failures = mux_ok ? 0 : min(mux_failures + 1, 255);
  if (mux_failures >= 20 && now - last_reset > 1000000) {
    emitf("# the multiplexer stopped answering: resetting the I2C bus\n");
    recoverBus();
    last_reset = now;
  }

  if (now >= next_frame) {
    sendFrame();
    next_frame = (now - next_frame > int64_t(frame_period_us)) ? now + frame_period_us
                                                               : next_frame + frame_period_us;
  }
  if (now >= next_probe) {  // an encoder plugged in later, or one that wasn't answering at boot
    next_probe = now + PROBE_PERIOD_US;
    probe_ch = (probe_ch + 1) % N_CH;
    if (!ch[probe_ch].present && probe(probe_ch)) {
      ch[probe_ch].present = true;
      emitf("# ch%u: encoder found\n", probe_ch);
    }
  }
  if (now >= next_magnet) {
    next_magnet = now + MAGNET_PERIOD_US;
    magnet_ch = (magnet_ch + 1) % N_CH;
    if (ch[magnet_ch].present) readMagnet(magnet_ch);
  }
  if (now >= next_status) {
    uint32_t elapsed = next_status == 0 ? STATUS_PERIOD_US : uint32_t(now - next_status + STATUS_PERIOD_US);
    sendStatus(uint32_t(uint64_t(passes) * 1000000ULL / elapsed));
    passes = 0;
    next_status = now + STATUS_PERIOD_US;
  }
}
