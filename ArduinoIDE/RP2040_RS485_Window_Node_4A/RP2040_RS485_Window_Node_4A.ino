#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_CAP1188.h>
#include "drivers/FlashIAP.h"

// RP2040 RS-485 window node: 4 actuators, 4 INA219, CAP1188, reeds, backup buttons.
// RS-485 protocol is line based, 3.3 V UART through an external MAX3485/SP3485 module.

constexpr uint8_t ACTUATORS = 4;
constexpr uint8_t GROUPS = 2; // top, bottom
constexpr uint8_t ACTUATORS_PER_GROUP = 2;

constexpr uint8_t DEFAULT_NODE_ADDRESS = 1;
constexpr uint8_t BROADCAST_ADDRESS = 0;

constexpr uint8_t RS485_TX_PIN = 0;
constexpr uint8_t RS485_RX_PIN = 1;
constexpr uint8_t RS485_DE_RE_PIN = 2;
constexpr uint32_t RS485_BAUD = 38400;
constexpr uint32_t RS485_TURNAROUND_US = 120;

constexpr uint8_t RELAY_EXTEND_PIN = 10;
constexpr uint8_t RELAY_RETRACT_PIN = 11;
constexpr bool RELAY_ACTIVE_LEVEL = HIGH;

constexpr uint8_t GROUP_ENABLE_PINS[GROUPS] = {12, 13}; // top, bottom
constexpr bool GROUP_ENABLE_ACTIVE_LEVEL = HIGH;

constexpr uint8_t REED_MID_PIN = 16;
constexpr uint8_t REED_VENT_PIN = 17;
constexpr uint8_t REED_CLOSED_PIN = 18;
constexpr uint8_t REED_EXTRA1_PIN = 19;
constexpr uint8_t REED_EXTRA2_PIN = 22;
constexpr bool REED_ACTIVE_LEVEL = HIGH;

constexpr uint8_t BUTTON_OPEN_PIN = 26;
constexpr uint8_t BUTTON_CLOSED_PIN = 27;
constexpr uint8_t BUTTON_VENT_PIN = 28;
constexpr bool BUTTON_ACTIVE_LEVEL = HIGH;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;

constexpr uint8_t I2C_SDA_PIN = 8;
constexpr uint8_t I2C_SCL_PIN = 9;
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint8_t INA_ADDRS[ACTUATORS] = {0x40, 0x41, 0x44, 0x45};

constexpr uint8_t CAP1188_ADDR = 0x29;
constexpr uint8_t CAP1188_IRQ_PIN = 5; // set to 255 if IRQ is not connected
constexpr uint8_t CAP1188_STOP_MASK = 0xFF;

constexpr uint32_t CONFIG_MAGIC = 0x52343835u; // R485
constexpr uint16_t CONFIG_VERSION = 1;

enum TargetMode : uint8_t { TARGET_NONE, TARGET_OPEN, TARGET_CLOSED, TARGET_VENT };
enum RunState : uint8_t { STATE_IDLE, STATE_MOVING, STATE_FAULT };
enum Direction : uint8_t { DIR_NONE, DIR_EXTEND, DIR_RETRACT };
enum GroupKind : uint8_t { GROUP_TOP, GROUP_BOTTOM };

struct PersistedConfig {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint8_t address;
  uint8_t reserved[3];
  uint32_t overCurrentConfirmMs;
  uint32_t endstopConfirmMs;
  uint32_t startIgnoreEndstopMs;
  uint32_t noCurrentFaultMs;
  uint32_t maxMoveMs;
  uint32_t sensorPollMs;
  float maxCurrentMa[ACTUATORS];
  float zeroCurrentMa[ACTUATORS];
  uint32_t checksum;
};

struct DebouncedButton {
  uint8_t pin = 255;
  bool stableActive = false;
  bool lastRawActive = false;
  bool rose = false;
  uint32_t changedAt = 0;

  void begin(uint8_t inputPin) {
    pin = inputPin;
    pinMode(pin, INPUT_PULLDOWN);
    lastRawActive = digitalRead(pin) == BUTTON_ACTIVE_LEVEL;
    stableActive = lastRawActive;
    changedAt = millis();
  }

  void update(uint32_t now) {
    rose = false;
    const bool raw = digitalRead(pin) == BUTTON_ACTIVE_LEVEL;
    if (raw != lastRawActive) {
      lastRawActive = raw;
      changedAt = now;
    }
    if ((now - changedAt) >= BUTTON_DEBOUNCE_MS && stableActive != raw) {
      const bool oldStable = stableActive;
      stableActive = raw;
      rose = !oldStable && stableActive;
    }
  }
};

TwoWire sensorBus(I2C_SDA_PIN, I2C_SCL_PIN);
Adafruit_INA219 ina[ACTUATORS] = {
  Adafruit_INA219(INA_ADDRS[0]),
  Adafruit_INA219(INA_ADDRS[1]),
  Adafruit_INA219(INA_ADDRS[2]),
  Adafruit_INA219(INA_ADDRS[3]),
};
Adafruit_CAP1188 cap1188 = Adafruit_CAP1188();

DebouncedButton buttonOpen;
DebouncedButton buttonClosed;
DebouncedButton buttonVent;

uint8_t nodeAddress = DEFAULT_NODE_ADDRESS;
String nodeUid = "unknown";
bool configLoaded = false;
bool configSaved = false;

float maxCurrentMa[ACTUATORS] = {2500, 2500, 2500, 2500};
float zeroCurrentMa[ACTUATORS] = {80, 80, 80, 80};
uint32_t overCurrentConfirmMs = 40;
uint32_t endstopConfirmMs = 250;
uint32_t startIgnoreEndstopMs = 450;
uint32_t noCurrentFaultMs = 1800;
uint32_t maxMoveMs = 45000;
uint32_t sensorPollMs = 10;
uint32_t statusPrintMs = 1000;

bool inaOk[ACTUATORS] = {};
bool capOk = false;
float currentMa[ACTUATORS] = {};
uint32_t overSince[ACTUATORS] = {};
uint32_t zeroSince[ACTUATORS] = {};
uint32_t earlyZeroSince[ACTUATORS] = {};

bool groupRunning[GROUPS] = {};
bool groupDone[GROUPS] = {};
bool reeds[5] = {};
uint8_t capTouched = 0;
volatile bool capIrqSeen = false;

RunState state = STATE_IDLE;
TargetMode target = TARGET_NONE;
TargetMode estimatedPosition = TARGET_NONE;
Direction direction = DIR_NONE;
String fault = "none";
int faultActuator = 0;
uint32_t moveStartedAt = 0;
uint32_t lastSensorPollAt = 0;
uint32_t lastStatusPrintAt = 0;
String rxLine;

static uint8_t actIndex(uint8_t group, uint8_t local) {
  return group * ACTUATORS_PER_GROUP + local;
}

static void onCapIrq() {
  capIrqSeen = true;
}

static void setRs485Tx(bool enabled) {
  digitalWrite(RS485_DE_RE_PIN, enabled ? HIGH : LOW);
  if (enabled) delayMicroseconds(RS485_TURNAROUND_US);
}

static void rs485Send(const String &line) {
  setRs485Tx(true);
  Serial1.print(line);
  Serial1.print('\n');
  Serial1.flush();
  delayMicroseconds(RS485_TURNAROUND_US);
  setRs485Tx(false);
}

static void writeActive(uint8_t pin, bool active, bool activeLevel) {
  digitalWrite(pin, active ? activeLevel : !activeLevel);
}

static void allRelaysOff() {
  writeActive(RELAY_EXTEND_PIN, false, RELAY_ACTIVE_LEVEL);
  writeActive(RELAY_RETRACT_PIN, false, RELAY_ACTIVE_LEVEL);
}

static void setGroup(uint8_t group, bool on) {
  groupRunning[group] = on;
  writeActive(GROUP_ENABLE_PINS[group], on, GROUP_ENABLE_ACTIVE_LEVEL);
}

static void allGroupsOff() {
  for (uint8_t group = 0; group < GROUPS; ++group) setGroup(group, false);
}

static void stopAll(const String &why, int actuator = 0, bool asFault = false) {
  allGroupsOff();
  allRelaysOff();
  direction = DIR_NONE;
  target = TARGET_NONE;
  state = asFault ? STATE_FAULT : STATE_IDLE;
  fault = why;
  faultActuator = actuator;
}

static const char *targetName(TargetMode value) {
  switch (value) {
    case TARGET_OPEN: return "open";
    case TARGET_CLOSED: return "closed";
    case TARGET_VENT: return "vent";
    default: return "none";
  }
}

static const char *stateName(RunState value) {
  switch (value) {
    case STATE_MOVING: return "moving";
    case STATE_FAULT: return "fault";
    default: return "idle";
  }
}

static uint32_t checksumConfig(const PersistedConfig &cfg) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&cfg);
  const size_t len = sizeof(PersistedConfig) - sizeof(cfg.checksum);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < len; ++i) {
    hash ^= bytes[i];
    hash *= 16777619u;
  }
  return hash;
}

static uint32_t flashConfigAddress(mbed::FlashIAP &flash) {
  const uint32_t start = flash.get_flash_start();
  const uint32_t size = flash.get_flash_size();
  const uint32_t last = start + size - 1;
  const uint32_t sectorSize = flash.get_sector_size(last);
  return start + size - sectorSize;
}

static void applyConfig(const PersistedConfig &cfg) {
  nodeAddress = constrain(cfg.address, 1, 247);
  overCurrentConfirmMs = constrain(cfg.overCurrentConfirmMs, 10ul, 5000ul);
  endstopConfirmMs = constrain(cfg.endstopConfirmMs, 20ul, 10000ul);
  startIgnoreEndstopMs = constrain(cfg.startIgnoreEndstopMs, 0ul, 10000ul);
  noCurrentFaultMs = constrain(cfg.noCurrentFaultMs, 100ul, 60000ul);
  maxMoveMs = constrain(cfg.maxMoveMs, 1000ul, 180000ul);
  sensorPollMs = constrain(cfg.sensorPollMs, 2ul, 1000ul);
  for (uint8_t i = 0; i < ACTUATORS; ++i) {
    maxCurrentMa[i] = constrain(cfg.maxCurrentMa[i], 100.0f, 10000.0f);
    zeroCurrentMa[i] = constrain(cfg.zeroCurrentMa[i], 1.0f, 1000.0f);
  }
}

static void makeConfig(PersistedConfig &cfg) {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;
  cfg.size = sizeof(PersistedConfig);
  cfg.address = nodeAddress;
  cfg.overCurrentConfirmMs = overCurrentConfirmMs;
  cfg.endstopConfirmMs = endstopConfirmMs;
  cfg.startIgnoreEndstopMs = startIgnoreEndstopMs;
  cfg.noCurrentFaultMs = noCurrentFaultMs;
  cfg.maxMoveMs = maxMoveMs;
  cfg.sensorPollMs = sensorPollMs;
  for (uint8_t i = 0; i < ACTUATORS; ++i) {
    cfg.maxCurrentMa[i] = maxCurrentMa[i];
    cfg.zeroCurrentMa[i] = zeroCurrentMa[i];
  }
  cfg.checksum = checksumConfig(cfg);
}

static bool loadConfigFromFlash() {
  mbed::FlashIAP flash;
  if (flash.init() != 0) return false;
  PersistedConfig cfg;
  const uint32_t addr = flashConfigAddress(flash);
  const int result = flash.read(&cfg, addr, sizeof(cfg));
  flash.deinit();
  if (result != 0) return false;
  if (cfg.magic != CONFIG_MAGIC || cfg.version != CONFIG_VERSION || cfg.size != sizeof(PersistedConfig)) return false;
  if (cfg.checksum != checksumConfig(cfg)) return false;
  applyConfig(cfg);
  return true;
}

static bool saveConfigToFlash() {
  mbed::FlashIAP flash;
  if (flash.init() != 0) return false;
  PersistedConfig cfg;
  makeConfig(cfg);
  const uint32_t addr = flashConfigAddress(flash);
  const uint32_t sectorSize = flash.get_sector_size(addr);
  const uint32_t pageSize = flash.get_page_size();
  const uint32_t writeSize = ((sizeof(PersistedConfig) + pageSize - 1) / pageSize) * pageSize;
  if (writeSize > 4096 || writeSize > sectorSize) {
    flash.deinit();
    return false;
  }
  uint8_t buffer[4096];
  memset(buffer, flash.get_erase_value(), sizeof(buffer));
  memcpy(buffer, &cfg, sizeof(cfg));
  const bool ok = (flash.erase(addr, sectorSize) == 0) && (flash.program(buffer, addr, writeSize) == 0);
  flash.deinit();
  return ok;
}

static void readReeds() {
  reeds[0] = digitalRead(REED_MID_PIN) == REED_ACTIVE_LEVEL;
  reeds[1] = digitalRead(REED_VENT_PIN) == REED_ACTIVE_LEVEL;
  reeds[2] = digitalRead(REED_CLOSED_PIN) == REED_ACTIVE_LEVEL;
  reeds[3] = digitalRead(REED_EXTRA1_PIN) == REED_ACTIVE_LEVEL;
  reeds[4] = digitalRead(REED_EXTRA2_PIN) == REED_ACTIVE_LEVEL;
}

static bool actuatorZeroConfirmed(uint8_t index, uint32_t now) {
  return zeroSince[index] != 0 && now - zeroSince[index] >= endstopConfirmMs;
}

static bool groupZeroConfirmed(uint8_t group, uint32_t now) {
  for (uint8_t i = 0; i < ACTUATORS_PER_GROUP; ++i) {
    if (!actuatorZeroConfirmed(actIndex(group, i), now)) return false;
  }
  return true;
}

static void clearTimers() {
  for (uint8_t i = 0; i < ACTUATORS; ++i) {
    overSince[i] = 0;
    zeroSince[i] = 0;
    earlyZeroSince[i] = 0;
  }
}

static void setDirection(Direction dir) {
  allRelaysOff();
  delay(80);
  if (dir == DIR_EXTEND) {
    writeActive(RELAY_RETRACT_PIN, false, RELAY_ACTIVE_LEVEL);
    delayMicroseconds(500);
    writeActive(RELAY_EXTEND_PIN, true, RELAY_ACTIVE_LEVEL);
  } else if (dir == DIR_RETRACT) {
    writeActive(RELAY_EXTEND_PIN, false, RELAY_ACTIVE_LEVEL);
    delayMicroseconds(500);
    writeActive(RELAY_RETRACT_PIN, true, RELAY_ACTIVE_LEVEL);
  }
  direction = dir;
}

static void startMove(TargetMode newTarget) {
  stopAll("none", 0, false);
  clearTimers();
  readReeds();
  target = newTarget;
  state = STATE_MOVING;
  fault = "none";
  faultActuator = 0;
  moveStartedAt = millis();
  for (uint8_t group = 0; group < GROUPS; ++group) groupDone[group] = false;
  setDirection(target == TARGET_CLOSED ? DIR_RETRACT : DIR_EXTEND);
  setGroup(GROUP_TOP, true);
  setGroup(GROUP_BOTTOM, true);
}

static void pollCap() {
  if (!capOk) {
    capTouched = 0;
    return;
  }
  if (!capIrqSeen && millis() - lastSensorPollAt < sensorPollMs) return;
  capIrqSeen = false;
  capTouched = cap1188.touched();
  if ((capTouched & CAP1188_STOP_MASK) != 0 && state == STATE_MOVING) {
    stopAll("cap1188", 0, true);
  }
}

static void pollCurrents(uint32_t now) {
  for (uint8_t i = 0; i < ACTUATORS; ++i) {
    if (!inaOk[i]) {
      currentMa[i] = 0;
      continue;
    }
    const float measured = ina[i].getCurrent_mA();
    if (!isfinite(measured) || !ina[i].success()) {
      inaOk[i] = false;
      currentMa[i] = 0;
      if (state == STATE_MOVING) stopAll("ina219", i + 1, true);
      return;
    }
    currentMa[i] = fabs(measured);
    if (state != STATE_MOVING) {
      overSince[i] = 0;
      zeroSince[i] = 0;
      earlyZeroSince[i] = 0;
      continue;
    }

    const uint8_t group = i / ACTUATORS_PER_GROUP;
    if (!groupRunning[group]) continue;

    if (currentMa[i] > maxCurrentMa[i]) {
      if (overSince[i] == 0) overSince[i] = now;
      if (now - overSince[i] >= overCurrentConfirmMs) {
        stopAll("overcurrent", i + 1, true);
        return;
      }
    } else {
      overSince[i] = 0;
    }

    const bool isZero = currentMa[i] < zeroCurrentMa[i];
    if (isZero) {
      if (zeroSince[i] == 0) zeroSince[i] = now;
    } else {
      zeroSince[i] = 0;
      earlyZeroSince[i] = 0;
    }

    if (now - moveStartedAt >= startIgnoreEndstopMs && isZero && !groupZeroConfirmed(group, now)) {
      if (earlyZeroSince[i] == 0) earlyZeroSince[i] = now;
      if (now - earlyZeroSince[i] >= noCurrentFaultMs) {
        stopAll("no_current", i + 1, true);
        return;
      }
    }
  }
}

static void finishGroup(uint8_t group) {
  groupDone[group] = true;
  setGroup(group, false);
}

static void updateMovement(uint32_t now) {
  if (state != STATE_MOVING) return;
  if (now - moveStartedAt > maxMoveMs) {
    stopAll("timeout", 0, true);
    return;
  }
  readReeds();
  if (reeds[0] && reeds[1]) {
    stopAll("reed_conflict", 0, true);
    return;
  }

  if (groupRunning[GROUP_TOP]) {
    if (target == TARGET_OPEN && reeds[0]) finishGroup(GROUP_TOP);
    else if (target == TARGET_VENT && reeds[1]) finishGroup(GROUP_TOP);
    else if (target == TARGET_CLOSED && groupZeroConfirmed(GROUP_TOP, now)) finishGroup(GROUP_TOP);
    else if ((target == TARGET_OPEN || target == TARGET_VENT) && groupZeroConfirmed(GROUP_TOP, now)) {
      finishGroup(GROUP_TOP);
      if (target == TARGET_OPEN) {
        stopAll("mid_reed_missed", 1, true);
        return;
      }
    }
  }

  if (groupRunning[GROUP_BOTTOM] && groupZeroConfirmed(GROUP_BOTTOM, now)) {
    finishGroup(GROUP_BOTTOM);
  }

  if (target == TARGET_OPEN && !groupRunning[GROUP_BOTTOM] && groupRunning[GROUP_TOP]) {
    finishGroup(GROUP_TOP);
    stopAll("mid_reed_missed", 1, true);
    return;
  }

  if (groupDone[GROUP_TOP] && groupDone[GROUP_BOTTOM]) {
    estimatedPosition = target;
    stopAll("none", 0, false);
  }
}

static String statusJson() {
  String json;
  json.reserve(760);
  json += F("{\"addr\":");
  json += String(nodeAddress);
  json += F(",\"uid\":\"");
  json += nodeUid;
  json += F("\",\"state\":\"");
  json += stateName(state);
  json += F("\",\"target\":\"");
  json += targetName(target);
  json += F("\",\"position\":\"");
  json += targetName(estimatedPosition);
  json += F("\",\"fault\":\"");
  json += fault;
  json += F("\",\"faultActuator\":");
  json += String(faultActuator);
  json += F(",\"cap\":");
  json += String(capTouched);
  json += F(",\"flashLoaded\":");
  json += configLoaded ? F("true") : F("false");
  json += F(",\"flashSaved\":");
  json += configSaved ? F("true") : F("false");
  json += F(",\"reed\":[");
  for (uint8_t i = 0; i < 5; ++i) {
    if (i) json += ',';
    json += reeds[i] ? '1' : '0';
  }
  json += F("],\"current\":[");
  for (uint8_t i = 0; i < ACTUATORS; ++i) {
    if (i) json += ',';
    json += String(currentMa[i], 0);
  }
  json += F("],\"inaOk\":[");
  for (uint8_t i = 0; i < ACTUATORS; ++i) {
    if (i) json += ',';
    json += inaOk[i] ? '1' : '0';
  }
  json += F("]}");
  return json;
}

static int valueAfter(const String &line, const String &key, int fallback) {
  const int pos = line.indexOf(key);
  if (pos < 0) return fallback;
  int start = pos + key.length();
  int end = line.indexOf(' ', start);
  if (end < 0) end = line.length();
  return line.substring(start, end).toInt();
}

static String valueStringAfter(const String &line, const String &key) {
  const int pos = line.indexOf(key);
  if (pos < 0) return "";
  int start = pos + key.length();
  int end = line.indexOf(' ', start);
  if (end < 0) end = line.length();
  return line.substring(start, end);
}

static void replyStatus() {
  rs485Send("@" + String(nodeAddress) + " " + statusJson());
}

static void handleCommand(String payload, bool broadcast) {
  payload.trim();
  payload.toUpperCase();

  if (payload == "DISCOVER") {
    delay(5 + (nodeAddress * 11));
    rs485Send("@0 DISCOVER UID=" + nodeUid + " ADDR=" + String(nodeAddress));
    return;
  }

  if (payload.startsWith("SETADDR ")) {
    const String uid = valueStringAfter(payload, "UID=");
    const int newAddress = valueAfter(payload, "ADDR=", -1);
    if (uid == nodeUid && newAddress >= 1 && newAddress <= 247) {
      nodeAddress = static_cast<uint8_t>(newAddress);
      configSaved = saveConfigToFlash();
      replyStatus();
    }
    return;
  }

  if (broadcast) return;

  if (payload == "CMD OPEN") startMove(TARGET_OPEN);
  else if (payload == "CMD CLOSED") startMove(TARGET_CLOSED);
  else if (payload == "CMD VENT") startMove(TARGET_VENT);
  else if (payload == "CMD STOP") stopAll("stopped", 0, false);
  else if (payload == "STATUS") {
    replyStatus();
    return;
  } else if (payload.startsWith("CFG ")) {
    int commonMax = valueAfter(payload, "MAXMA=", -1);
    int commonZero = valueAfter(payload, "ZEROMA=", -1);
    if (commonMax > 0) {
      for (uint8_t i = 0; i < ACTUATORS; ++i) maxCurrentMa[i] = commonMax;
    }
    if (commonZero > 0) {
      for (uint8_t i = 0; i < ACTUATORS; ++i) zeroCurrentMa[i] = commonZero;
    }
    for (uint8_t i = 0; i < ACTUATORS; ++i) {
      const String key = "A" + String(i + 1) + "MAX=";
      int actuatorMax = valueAfter(payload, key, -1);
      if (actuatorMax > 0) maxCurrentMa[i] = actuatorMax;
    }
    maxMoveMs = valueAfter(payload, "MAXMOVEMS=", maxMoveMs);
    noCurrentFaultMs = valueAfter(payload, "NOCURRENTMS=", noCurrentFaultMs);
    configSaved = saveConfigToFlash();
  }
  replyStatus();
}

static void parseRs485Line(String line) {
  line.trim();
  if (!line.startsWith("@")) return;
  const int space = line.indexOf(' ');
  if (space < 0) return;
  const int address = line.substring(1, space).toInt();
  const bool broadcast = address == BROADCAST_ADDRESS;
  if (!broadcast && address != nodeAddress) return;
  handleCommand(line.substring(space + 1), broadcast);
}

static void readRs485() {
  while (Serial1.available()) {
    const char c = static_cast<char>(Serial1.read());
    if (c == '\n' || c == '\r') {
      if (rxLine.length()) {
        parseRs485Line(rxLine);
        rxLine = "";
      }
    } else if (rxLine.length() < 160) {
      rxLine += c;
    }
  }
}

static void readBackupButtons(uint32_t now) {
  buttonOpen.update(now);
  buttonClosed.update(now);
  buttonVent.update(now);
  if (buttonOpen.rose) startMove(TARGET_OPEN);
  else if (buttonClosed.rose) startMove(TARGET_CLOSED);
  else if (buttonVent.rose) startMove(TARGET_VENT);
}

static String readUid() {
  uint8_t serial[32] = {};
  const uint8_t len = getUniqueSerialNumber(serial);
  String out;
  for (uint8_t i = 0; i < len; ++i) {
    if (serial[i] < 16) out += '0';
    out += String(serial[i], HEX);
  }
  out.toUpperCase();
  return out.length() ? out : "RP2040";
}

void setup() {
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  setRs485Tx(false);
  pinMode(RELAY_EXTEND_PIN, OUTPUT);
  pinMode(RELAY_RETRACT_PIN, OUTPUT);
  allRelaysOff();
  for (uint8_t group = 0; group < GROUPS; ++group) pinMode(GROUP_ENABLE_PINS[group], OUTPUT);
  allGroupsOff();

  pinMode(REED_MID_PIN, INPUT_PULLDOWN);
  pinMode(REED_VENT_PIN, INPUT_PULLDOWN);
  pinMode(REED_CLOSED_PIN, INPUT_PULLDOWN);
  pinMode(REED_EXTRA1_PIN, INPUT_PULLDOWN);
  pinMode(REED_EXTRA2_PIN, INPUT_PULLDOWN);

  buttonOpen.begin(BUTTON_OPEN_PIN);
  buttonClosed.begin(BUTTON_CLOSED_PIN);
  buttonVent.begin(BUTTON_VENT_PIN);

  Serial.begin(115200);
  Serial1.begin(RS485_BAUD);
  delay(300);

  nodeUid = readUid();
  configLoaded = loadConfigFromFlash();
  configSaved = configLoaded;

  sensorBus.begin();
  sensorBus.setClock(I2C_CLOCK_HZ);
  for (uint8_t i = 0; i < ACTUATORS; ++i) {
    inaOk[i] = ina[i].begin(&sensorBus);
    if (inaOk[i]) ina[i].setCalibration_32V_2A();
  }
  capOk = cap1188.begin(CAP1188_ADDR, &sensorBus);
  if (CAP1188_IRQ_PIN != 255) {
    pinMode(CAP1188_IRQ_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CAP1188_IRQ_PIN), onCapIrq, FALLING);
  }
  readReeds();
  Serial.println(F("RP2040 RS485 node ready"));
}

void loop() {
  const uint32_t now = millis();
  readRs485();
  readBackupButtons(now);
  pollCap();
  if (now - lastSensorPollAt >= sensorPollMs) {
    lastSensorPollAt = now;
    pollCurrents(now);
    updateMovement(now);
  }
  if (now - lastStatusPrintAt >= statusPrintMs) {
    lastStatusPrintAt = now;
    Serial.println(statusJson());
  }
}
