#include <Arduino.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

// ----------------------------- Hardware pins -----------------------------
static constexpr int PIN_SCK = 12;
static constexpr int PIN_MISO = 13;
static constexpr int PIN_MOSI = 11;
static constexpr int PIN_CS = 10;
static constexpr int PIN_GDO0 = 4;
static constexpr int PIN_GDO2 = 5;

static constexpr int PIN_LEARN_BUTTON = 9; // Button to GND.
static constexpr uint32_t LEARN_BUTTON_LONG_PRESS_MS = 5000;

static constexpr int RGB_LED_PIN = 48;     // Set to -1 to disable RGB LED.
static constexpr uint32_t RGB_FLASH_MS = 700;

static constexpr uint8_t OUTPUT_COUNT = 6;
static constexpr int OUTPUT_PINS[OUTPUT_COUNT] = {6, 7, 15, 16, 17, 18};
static constexpr bool OUTPUT_ACTIVE_HIGH = true;
static constexpr uint32_t OUTPUT_PULSE_MS = 500;

// ----------------------------- Wi-Fi / web -----------------------------
static const char *WIFI_SSID = "HOME";
static const char *WIFI_PASSWORD = "123A123BC";
static constexpr uint8_t WIFI_SSID_LEN = 33;
static constexpr uint8_t WIFI_PASSWORD_LEN = 65;

static const char *WEB_USER = "admin";
static const char *WEB_PASSWORD = "admin12345";

static const char *AP_SSID = "ESP32-CC1101";
static const char *AP_PASSWORD = "12345678";

// ----------------------------- RF settings -----------------------------
static constexpr float RF_FREQ_MHZ = 433.92;
static constexpr float RF_DATA_RATE_KBPS = 19.2;
static constexpr uint32_t RF_IDLE_FRAME_US = 30000;
static constexpr uint32_t RF_DEBOUNCE_MS = 700;
static constexpr size_t RF_MAX_PULSES = 180;
static constexpr byte CC1101_GDO0_ASYNC_SERIAL = 0x0D;
static constexpr byte CC1101_GDO2_CARRIER_SENSE = 0x0E;

static constexpr uint32_t SHORT_MIN_US = 180;
static constexpr uint32_t SHORT_MAX_US = 560;
static constexpr uint32_t LONG_MIN_US = 650;
static constexpr uint32_t LONG_MAX_US = 1300;
static constexpr uint32_t SYNC_MIN_US = 5000;
static constexpr uint32_t SYNC_MAX_US = 14000;
static constexpr uint8_t EXPECTED_BITS = 24;
static constexpr uint8_t MAX_CODES = 36;
static constexpr uint8_t NAME_LEN = 24;
static constexpr uint32_t LEARN_TIMEOUT_MS = 30000;

struct ButtonRecord {
  uint32_t code;
  uint8_t output;
  uint8_t actionType; // 0 local output, 1 local UART RP2040, 2 RS-485 RP2040
  uint8_t actionCommand; // 0 none, 1 open, 2 closed, 3 vent, 4 stop
  uint8_t rs485Node; // 0..7
  bool enabled;
  char name[NAME_LEN];
};

static ButtonRecord records[MAX_CODES];
static uint8_t recordCount = 0;
static uint32_t lastCode = 0;
static uint32_t lastCodeMs = 0;
static uint32_t lastSeenCode = 0;
static uint32_t lastSeenMs = 0;
static uint32_t lastMatchedCode = 0;
static uint32_t lastMatchedMs = 0;
static uint8_t lastMatchedOutput = 255;
static int8_t lastMatchedIndex = -1;
static char lastMatchedName[NAME_LEN] = "";
static bool learnMode = false;
static uint32_t learnStartedMs = 0;
static bool radioReady = false;
static uint32_t outputOffAt[OUTPUT_COUNT] = {0};
static uint32_t rgbOffAt = 0;
static uint8_t rgbBlinkSteps = 0;
static bool rgbBlinkOn = false;
static uint32_t rgbBlinkNextMs = 0;
static char wifiSsid[WIFI_SSID_LEN] = "";
static char wifiPassword[WIFI_PASSWORD_LEN] = "";

static Preferences prefs;
static WebServer server(80);
static HardwareSerial rpSerial(1);
static HardwareSerial rs485Serial(2);

static constexpr int RP2040_UART_RX_PIN = 2;
static constexpr int RP2040_UART_TX_PIN = 3;
static constexpr uint32_t RP2040_UART_BAUD = 115200;
static constexpr uint8_t WINDOW_ACTUATORS = 8;
static constexpr int RS485_RX_PIN = 38;
static constexpr int RS485_TX_PIN = 39;
static constexpr int RS485_DE_RE_PIN = 40;
static constexpr uint32_t RS485_BAUD = 38400;
static constexpr uint32_t RS485_TURNAROUND_US = 120;
static constexpr uint8_t RS485_NODE_COUNT = 8;
static constexpr uint8_t RS485_ACTUATORS = 4;
static constexpr uint8_t RP_NAME_LEN = 33;

struct Rs485NodeConfig {
  bool enabled;
  uint8_t address;
  char name[RP_NAME_LEN];
  char uid[33];
  uint16_t maxCurrentMa[RS485_ACTUATORS];
  uint16_t zeroCurrentMa;
  uint32_t maxMoveMs;
};

static uint8_t windowCount = 2;
static char localRpName[RP_NAME_LEN] = "Локальная RP2040";
static char mainWindowTarget[8] = "local";
static uint16_t windowMaxCurrentMa[WINDOW_ACTUATORS] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
static uint16_t windowZeroCurrentMa = 80;
static uint32_t windowMaxMoveMs = 45000;
static String rpLine;
static String lastRpStatus = "{}";
static uint32_t lastRpStatusMs = 0;
static Rs485NodeConfig rs485Nodes[RS485_NODE_COUNT];
static String rs485Line;
static String rs485Status[RS485_NODE_COUNT];
static uint32_t rs485LastSeenMs[RS485_NODE_COUNT] = {0};
static String rs485DiscoverLog;

static volatile uint16_t isrPulseCount = 0;
static volatile uint32_t isrPulseDurations[RF_MAX_PULSES];
static volatile uint32_t isrLastEdgeUs = 0;
static volatile uint32_t isrLastActivityUs = 0;
static volatile bool isrHadActivity = false;
static volatile bool isrFrameReady = false;

void IRAM_ATTR onGdo0Change() {
  const uint32_t nowUs = micros();
  if (isrHadActivity && isrPulseCount < RF_MAX_PULSES) {
    isrPulseDurations[isrPulseCount++] = nowUs - isrLastEdgeUs;
  }
  isrHadActivity = true;
  isrLastEdgeUs = nowUs;
  isrLastActivityUs = nowUs;
}

static void setRgb(uint8_t red, uint8_t green, uint8_t blue) {
  if (RGB_LED_PIN < 0) return;
  neopixelWrite(RGB_LED_PIN, red, green, blue);
}

static void flashRgb(uint8_t red, uint8_t green, uint8_t blue) {
  rgbBlinkSteps = 0;
  setRgb(red, green, blue);
  rgbOffAt = millis() + RGB_FLASH_MS;
}

static void blinkWhiteThreeTimes() {
  rgbOffAt = 0;
  rgbBlinkSteps = 6;
  rgbBlinkOn = false;
  rgbBlinkNextMs = 0;
}

static void updateRgb() {
  if (rgbBlinkSteps > 0 && static_cast<int32_t>(millis() - rgbBlinkNextMs) >= 0) {
    rgbBlinkOn = !rgbBlinkOn;
    setRgb(rgbBlinkOn ? 60 : 0, rgbBlinkOn ? 60 : 0, rgbBlinkOn ? 60 : 0);
    --rgbBlinkSteps;
    rgbBlinkNextMs = millis() + 180;
    if (rgbBlinkSteps == 0) setRgb(0, 0, 0);
    return;
  }
  if (rgbOffAt != 0 && static_cast<int32_t>(millis() - rgbOffAt) >= 0) {
    rgbOffAt = 0;
    setRgb(0, 0, 0);
  }
}

static void setOutput(uint8_t index, bool on) {
  if (index >= OUTPUT_COUNT) return;
  digitalWrite(OUTPUT_PINS[index], on == OUTPUT_ACTIVE_HIGH ? HIGH : LOW);
}

static void pulseOutput(uint8_t index) {
  if (index >= OUTPUT_COUNT) return;
  setOutput(index, true);
  outputOffAt[index] = millis() + OUTPUT_PULSE_MS;
  Serial.printf("[OUT] Output %u pulse\n", index + 1);
}

static void updateOutputs() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < OUTPUT_COUNT; ++i) {
    if (outputOffAt[i] != 0 && static_cast<int32_t>(now - outputOffAt[i]) >= 0) {
      outputOffAt[i] = 0;
      setOutput(i, false);
    }
  }
}

static bool isShortPulse(uint32_t pulseUs) { return pulseUs >= SHORT_MIN_US && pulseUs <= SHORT_MAX_US; }
static bool isLongPulse(uint32_t pulseUs) { return pulseUs >= LONG_MIN_US && pulseUs <= LONG_MAX_US; }
static bool isSyncPulse(uint32_t pulseUs) { return pulseUs >= SYNC_MIN_US && pulseUs <= SYNC_MAX_US; }

static uint32_t bitsToValue(const char *bits, uint8_t bitCount) {
  uint32_t value = 0;
  for (uint8_t i = 0; i < bitCount && i < 32; ++i) {
    value <<= 1;
    if (bits[i] == '1') value |= 1;
  }
  return value;
}

static uint8_t decodeAfterSync(const uint32_t *pulses, uint16_t pulseCount, uint16_t syncIndex, char *bits) {
  uint8_t bitCount = 0;
  uint16_t i = syncIndex + 1;
  while (i + 1 < pulseCount && bitCount < EXPECTED_BITS) {
    const uint32_t first = pulses[i];
    const uint32_t second = pulses[i + 1];
    if (isSyncPulse(first) || isSyncPulse(second)) break;
    if (isShortPulse(first) && isLongPulse(second)) {
      bits[bitCount++] = '0';
      i += 2;
    } else if (isLongPulse(first) && isShortPulse(second)) {
      bits[bitCount++] = '1';
      i += 2;
    } else {
      break;
    }
  }
  bits[bitCount] = '\0';
  return bitCount;
}

static bool decodeFrame(const uint32_t *pulses, uint16_t pulseCount, uint32_t &codeOut) {
  uint8_t fullCount = 0;
  uint32_t candidate = 0;
  for (uint16_t i = 0; i < pulseCount; ++i) {
    if (!isSyncPulse(pulses[i])) continue;
    char bits[EXPECTED_BITS + 1] = {0};
    if (decodeAfterSync(pulses, pulseCount, i, bits) != EXPECTED_BITS) continue;
    const uint32_t value = bitsToValue(bits, EXPECTED_BITS);
    if (fullCount == 0) candidate = value;
    else if (candidate != value) return false;
    ++fullCount;
  }
  if (fullCount == 0) return false;
  codeOut = candidate;
  return true;
}

static void configureRadio() {
  ELECHOUSE_cc1101.setCCMode(1);
  ELECHOUSE_cc1101.setModulation(2);
  ELECHOUSE_cc1101.setMHZ(RF_FREQ_MHZ);
  ELECHOUSE_cc1101.setDeviation(0);
  ELECHOUSE_cc1101.setChannel(0);
  ELECHOUSE_cc1101.setChsp(199.95);
  ELECHOUSE_cc1101.setRxBW(812.50);
  ELECHOUSE_cc1101.setDRate(RF_DATA_RATE_KBPS);
  ELECHOUSE_cc1101.setPA(10);
  ELECHOUSE_cc1101.setSyncMode(0);
  ELECHOUSE_cc1101.setAdrChk(0);
  ELECHOUSE_cc1101.setWhiteData(0);
  ELECHOUSE_cc1101.setPktFormat(3);
  ELECHOUSE_cc1101.setLengthConfig(0);
  ELECHOUSE_cc1101.setCrc(0);
  ELECHOUSE_cc1101.setCRC_AF(0);
  ELECHOUSE_cc1101.setDcFilterOff(0);
  ELECHOUSE_cc1101.setManchester(0);
  ELECHOUSE_cc1101.setFEC(0);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, CC1101_GDO0_ASYNC_SERIAL);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG2, CC1101_GDO2_CARRIER_SENSE);
  ELECHOUSE_cc1101.SetRx();
}

static bool beginRadio() {
  ELECHOUSE_cc1101.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  ELECHOUSE_cc1101.setGDO(PIN_GDO0, PIN_GDO2);
  ELECHOUSE_cc1101.Init();
  const bool detected = ELECHOUSE_cc1101.getCC1101();
  Serial.println(detected ? "[BOOT] CC1101: OK" : "[BOOT] CC1101: FAIL");
  if (!detected) return false;
  configureRadio();
  pinMode(PIN_GDO0, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_GDO0), onGdo0Change, CHANGE);
  return true;
}

static void resetRfCapture() {
  noInterrupts();
  isrPulseCount = 0;
  isrLastEdgeUs = micros();
  isrLastActivityUs = isrLastEdgeUs;
  isrHadActivity = false;
  isrFrameReady = false;
  interrupts();
}

static void pollRfFrameReady() {
  noInterrupts();
  const bool hadActivity = isrHadActivity;
  const uint32_t lastActivityUs = isrLastActivityUs;
  const bool ready = isrFrameReady;
  interrupts();
  if (!ready && hadActivity && micros() - lastActivityUs > RF_IDLE_FRAME_US) {
    noInterrupts();
    isrFrameReady = true;
    interrupts();
  }
}

static int findRecord(uint32_t code) {
  for (uint8_t i = 0; i < recordCount; ++i) {
    if (records[i].code == code) return i;
  }
  return -1;
}

static void saveRecords() {
  prefs.begin("rfctrl", false);
  prefs.putUChar("count", recordCount);
  for (uint8_t i = 0; i < recordCount; ++i) {
    char key[16];
    snprintf(key, sizeof(key), "code%u", i); prefs.putUInt(key, records[i].code);
    snprintf(key, sizeof(key), "out%u", i); prefs.putUChar(key, records[i].output);
    snprintf(key, sizeof(key), "act%u", i); prefs.putUChar(key, records[i].actionType);
    snprintf(key, sizeof(key), "cmd%u", i); prefs.putUChar(key, records[i].actionCommand);
    snprintf(key, sizeof(key), "node%u", i); prefs.putUChar(key, records[i].rs485Node);
    snprintf(key, sizeof(key), "en%u", i); prefs.putBool(key, records[i].enabled);
    snprintf(key, sizeof(key), "name%u", i); prefs.putString(key, records[i].name);
  }
  prefs.end();
}

static void clearRecords() {
  recordCount = 0;
  lastMatchedCode = 0;
  lastMatchedMs = 0;
  lastMatchedOutput = 255;
  lastMatchedIndex = -1;
  lastMatchedName[0] = '\0';
  saveRecords();
  Serial.println("[MEM] All learned buttons cleared");
}

static void loadRecords() {
  prefs.begin("rfctrl", true);
  recordCount = prefs.getUChar("count", 0);
  if (recordCount > MAX_CODES) recordCount = MAX_CODES;
  for (uint8_t i = 0; i < recordCount; ++i) {
    char key[16];
    snprintf(key, sizeof(key), "code%u", i); records[i].code = prefs.getUInt(key, 0);
    snprintf(key, sizeof(key), "out%u", i); records[i].output = prefs.getUChar(key, 255);
    snprintf(key, sizeof(key), "act%u", i); records[i].actionType = prefs.getUChar(key, 0);
    snprintf(key, sizeof(key), "cmd%u", i); records[i].actionCommand = prefs.getUChar(key, 0);
    snprintf(key, sizeof(key), "node%u", i); records[i].rs485Node = prefs.getUChar(key, 0);
    records[i].actionType = constrain(records[i].actionType, 0, 2);
    records[i].actionCommand = constrain(records[i].actionCommand, 0, 4);
    records[i].rs485Node = constrain(records[i].rs485Node, 0, RS485_NODE_COUNT - 1);
    snprintf(key, sizeof(key), "en%u", i); records[i].enabled = prefs.getBool(key, true);
    snprintf(key, sizeof(key), "name%u", i);
    String name = prefs.getString(key, "");
    if (name.length() == 0) name = "Button " + String(i + 1);
    strlcpy(records[i].name, name.c_str(), sizeof(records[i].name));
  }
  prefs.end();
}

static void loadWifiSettings() {
  prefs.begin("wifi", true);
  String ssid = prefs.getString("ssid", WIFI_SSID);
  String pass = prefs.getString("pass", WIFI_PASSWORD);
  prefs.end();
  strlcpy(wifiSsid, ssid.c_str(), sizeof(wifiSsid));
  strlcpy(wifiPassword, pass.c_str(), sizeof(wifiPassword));
}

static void saveWifiSettings(const String &ssid, const String &password) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", password);
  prefs.end();
  strlcpy(wifiSsid, ssid.c_str(), sizeof(wifiSsid));
  strlcpy(wifiPassword, password.c_str(), sizeof(wifiPassword));
}

static void loadWindowSettings() {
  prefs.begin("window", true);
  String localName = prefs.getString("localName", "Локальная RP2040");
  strlcpy(localRpName, localName.c_str(), sizeof(localRpName));
  String mainTarget = prefs.getString("mainTarget", "local");
  strlcpy(mainWindowTarget, mainTarget.c_str(), sizeof(mainWindowTarget));
  windowCount = prefs.getUChar("count", 2);
  windowZeroCurrentMa = prefs.getUShort("zeroMa", 80);
  windowMaxMoveMs = prefs.getUInt("maxMove", 45000);
  for (uint8_t i = 0; i < WINDOW_ACTUATORS; ++i) {
    char key[12];
    snprintf(key, sizeof(key), "max%u", i);
    windowMaxCurrentMa[i] = prefs.getUShort(key, 2500);
  }
  prefs.end();
  windowCount = constrain(windowCount, 1, 2);
}

static void saveWindowSettings() {
  prefs.begin("window", false);
  prefs.putString("localName", localRpName);
  prefs.putString("mainTarget", mainWindowTarget);
  prefs.putUChar("count", windowCount);
  prefs.putUShort("zeroMa", windowZeroCurrentMa);
  prefs.putUInt("maxMove", windowMaxMoveMs);
  for (uint8_t i = 0; i < WINDOW_ACTUATORS; ++i) {
    char key[12];
    snprintf(key, sizeof(key), "max%u", i);
    prefs.putUShort(key, windowMaxCurrentMa[i]);
  }
  prefs.end();
}

static void loadRs485Settings() {
  prefs.begin("rs485", true);
  for (uint8_t n = 0; n < RS485_NODE_COUNT; ++n) {
    char key[20];
    snprintf(key, sizeof(key), "en%u", n);
    rs485Nodes[n].enabled = prefs.getBool(key, n == 0);
    snprintf(key, sizeof(key), "addr%u", n);
    rs485Nodes[n].address = prefs.getUChar(key, n + 1);
    snprintf(key, sizeof(key), "name%u", n);
    String name = prefs.getString(key, "RP2040 " + String(n + 1));
    strlcpy(rs485Nodes[n].name, name.c_str(), sizeof(rs485Nodes[n].name));
    snprintf(key, sizeof(key), "uid%u", n);
    String uid = prefs.getString(key, "");
    strlcpy(rs485Nodes[n].uid, uid.c_str(), sizeof(rs485Nodes[n].uid));
    snprintf(key, sizeof(key), "zero%u", n);
    rs485Nodes[n].zeroCurrentMa = prefs.getUShort(key, 80);
    snprintf(key, sizeof(key), "move%u", n);
    rs485Nodes[n].maxMoveMs = prefs.getUInt(key, 45000);
    for (uint8_t a = 0; a < RS485_ACTUATORS; ++a) {
      snprintf(key, sizeof(key), "n%ua%u", n, a);
      rs485Nodes[n].maxCurrentMa[a] = prefs.getUShort(key, 2500);
    }
  }
  prefs.end();
}

static void saveRs485Settings() {
  prefs.begin("rs485", false);
  for (uint8_t n = 0; n < RS485_NODE_COUNT; ++n) {
    char key[20];
    snprintf(key, sizeof(key), "en%u", n);
    prefs.putBool(key, rs485Nodes[n].enabled);
    snprintf(key, sizeof(key), "addr%u", n);
    prefs.putUChar(key, rs485Nodes[n].address);
    snprintf(key, sizeof(key), "name%u", n);
    prefs.putString(key, rs485Nodes[n].name);
    snprintf(key, sizeof(key), "uid%u", n);
    prefs.putString(key, rs485Nodes[n].uid);
    snprintf(key, sizeof(key), "zero%u", n);
    prefs.putUShort(key, rs485Nodes[n].zeroCurrentMa);
    snprintf(key, sizeof(key), "move%u", n);
    prefs.putUInt(key, rs485Nodes[n].maxMoveMs);
    for (uint8_t a = 0; a < RS485_ACTUATORS; ++a) {
      snprintf(key, sizeof(key), "n%ua%u", n, a);
      prefs.putUShort(key, rs485Nodes[n].maxCurrentMa[a]);
    }
  }
  prefs.end();
}

static void sendRpCommand(const String &line) {
  rpSerial.print(line);
  rpSerial.print('\n');
  Serial.print("[RP2040] > ");
  Serial.println(line);
}

static void setRs485Tx(bool enabled) {
  digitalWrite(RS485_DE_RE_PIN, enabled ? HIGH : LOW);
  if (enabled) delayMicroseconds(RS485_TURNAROUND_US);
}

static void sendRs485Line(const String &line) {
  setRs485Tx(true);
  rs485Serial.print(line);
  rs485Serial.print('\n');
  rs485Serial.flush();
  delayMicroseconds(RS485_TURNAROUND_US);
  setRs485Tx(false);
  Serial.print("[RS485] > ");
  Serial.println(line);
}

static String rs485ConfigLine(uint8_t index) {
  const Rs485NodeConfig &node = rs485Nodes[index];
  String line = "@" + String(node.address) + " CFG";
  line += " ZEROMA=" + String(node.zeroCurrentMa);
  line += " MAXMOVEMS=" + String(node.maxMoveMs);
  for (uint8_t a = 0; a < RS485_ACTUATORS; ++a) {
    line += " A";
    line += String(a + 1);
    line += "MAX=";
    line += String(node.maxCurrentMa[a]);
  }
  return line;
}

static void sendRs485NodeConfig(uint8_t index) {
  if (index >= RS485_NODE_COUNT || !rs485Nodes[index].enabled) return;
  sendRs485Line(rs485ConfigLine(index));
}

static void sendWindowConfigToRp2040() {
  String line = "CFG WINDOWS=" + String(windowCount);
  line += " ZEROMA=" + String(windowZeroCurrentMa);
  line += " MAXMOVEMS=" + String(windowMaxMoveMs);
  for (uint8_t i = 0; i < WINDOW_ACTUATORS; ++i) {
    line += " A";
    line += String(i + 1);
    line += "MAX=";
    line += String(windowMaxCurrentMa[i]);
  }
  sendRpCommand(line);
}

static String windowCommandLine(uint8_t command) {
  if (command == 1) return "CMD OPEN";
  if (command == 2) return "CMD CLOSED";
  if (command == 3) return "CMD VENT";
  if (command == 4) return "CMD STOP";
  return "";
}

static String windowCommandLabel(uint8_t command) {
  if (command == 1) return "Открыто";
  if (command == 2) return "Закрыто";
  if (command == 3) return "Проветривание";
  if (command == 4) return "Стоп";
  return "-";
}

static String rfActionLabel(const ButtonRecord &record) {
  if (record.actionType == 1) {
    return "UART RP2040: " + windowCommandLabel(record.actionCommand);
  }
  if (record.actionType == 2) {
    String label = "RS-485 ";
    if (record.rs485Node < RS485_NODE_COUNT) {
      label += String(record.rs485Node + 1);
      label += " ";
      label += rs485Nodes[record.rs485Node].name;
    } else {
      label += "-";
    }
    label += ": ";
    label += windowCommandLabel(record.actionCommand);
    return label;
  }
  if (record.output < OUTPUT_COUNT) return "Выход " + String(record.output + 1);
  return "-";
}

static void executeRfAction(const ButtonRecord &record) {
  const String commandLine = windowCommandLine(record.actionCommand);
  if (record.actionType == 1) {
    if (commandLine.length()) sendRpCommand(commandLine);
    return;
  }
  if (record.actionType == 2) {
    if (!commandLine.length()) return;
    if (record.rs485Node >= RS485_NODE_COUNT) return;
    const Rs485NodeConfig &node = rs485Nodes[record.rs485Node];
    if (!node.enabled) return;
    sendRs485Line("@" + String(node.address) + " " + commandLine);
    return;
  }
  if (record.output < OUTPUT_COUNT) pulseOutput(record.output);
}

static bool sendWindowTargetCommand(const String &target, const String &commandLine) {
  if (!commandLine.length()) return false;
  if (target == "local") {
    sendRpCommand(commandLine);
    return true;
  }
  if (target.startsWith("rs")) {
    const int idx = target.substring(2).toInt();
    if (idx < 0 || idx >= RS485_NODE_COUNT) return false;
    if (!rs485Nodes[idx].enabled) return false;
    sendRs485Line("@" + String(rs485Nodes[idx].address) + " " + commandLine);
    return true;
  }
  return false;
}

static bool addRecord(uint32_t code) {
  if (findRecord(code) >= 0) {
    flashRgb(60, 60, 60);
    return true;
  }
  if (recordCount >= MAX_CODES) {
    Serial.println("[LEARN] Memory full");
    flashRgb(80, 0, 0);
    return false;
  }
  ButtonRecord &record = records[recordCount];
  record.code = code;
  record.output = 255;
  record.actionType = 0;
  record.actionCommand = 0;
  record.rs485Node = 0;
  record.enabled = true;
  snprintf(record.name, sizeof(record.name), "Button %u", recordCount + 1);
  ++recordCount;
  saveRecords();
  Serial.printf("[LEARN] Added 0x%06lX\n", static_cast<unsigned long>(code & 0xFFFFFF));
  flashRgb(0, 80, 0);
  return true;
}

static void startLearnMode() {
  learnMode = true;
  learnStartedMs = millis();
  Serial.println("[LEARN] Mode ON");
}

static void stopLearnMode(bool signalExit = false) {
  if (learnMode) Serial.println("[LEARN] Mode OFF");
  learnMode = false;
  if (signalExit) blinkWhiteThreeTimes();
}

static void toggleLearnMode() {
  if (learnMode) stopLearnMode(true);
  else startLearnMode();
}

static void handleCode(uint32_t code) {
  const uint32_t now = millis();
  lastSeenCode = code;
  lastSeenMs = now;
  Serial.printf("[RF] Code 0x%06lX\n", static_cast<unsigned long>(code & 0xFFFFFF));
  if (learnMode) {
    addRecord(code);
    return;
  }
  if (code == lastCode && now - lastCodeMs < RF_DEBOUNCE_MS) return;
  lastCode = code;
  lastCodeMs = now;
  const int index = findRecord(code);
  if (index < 0) {
    flashRgb(80, 0, 0);
    Serial.println("[RF] Unknown code");
    return;
  }
  const ButtonRecord &record = records[index];
  if (!record.enabled) return;
  lastMatchedCode = code;
  lastMatchedMs = now;
  lastMatchedOutput = record.output;
  lastMatchedIndex = index;
  strlcpy(lastMatchedName, record.name, sizeof(lastMatchedName));
  executeRfAction(record);
}

static void processRf() {
  if (!radioReady) return;
  pollRfFrameReady();
  noInterrupts();
  const bool ready = isrFrameReady;
  uint16_t pulseCount = isrPulseCount;
  uint32_t pulses[RF_MAX_PULSES];
  if (pulseCount > RF_MAX_PULSES) pulseCount = RF_MAX_PULSES;
  if (ready) {
    for (uint16_t i = 0; i < pulseCount; ++i) pulses[i] = isrPulseDurations[i];
    isrPulseCount = 0;
    isrHadActivity = false;
    isrFrameReady = false;
  }
  interrupts();
  if (!ready) return;
  uint32_t code = 0;
  if (decodeFrame(pulses, pulseCount, code)) handleCode(code);
}

static bool webAuth() {
  if (server.authenticate(WEB_USER, WEB_PASSWORD)) return true;
  server.requestAuthentication(BASIC_AUTH, "ESP32 CC1101");
  return false;
}

static String htmlEscape(const String &input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); ++i) {
    const char c = input[i];
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else out += c;
  }
  return out;
}

static String codeHex(uint32_t code) {
  char buf[12];
  snprintf(buf, sizeof(buf), "0x%06lX", static_cast<unsigned long>(code & 0xFFFFFF));
  return String(buf);
}

static uint16_t remoteIdFromCode(uint32_t code) {
  return static_cast<uint16_t>((code >> 8) & 0xFFFF);
}

static String remoteHex(uint16_t remoteId) {
  char buf[8];
  snprintf(buf, sizeof(buf), "0x%04X", remoteId);
  return String(buf);
}

static void appendPageHeader(String &html, const char *title) {
  html += F("<!doctype html><html><head><meta charset='utf-8'>");
  html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>");
  html += title;
  html += F("</title><style>");
  html += F("body{font-family:Arial,sans-serif;margin:24px;background:#f5f7fb;color:#18202a}");
  html += F("main{max-width:980px;margin:auto}table{width:100%;border-collapse:collapse;background:white}");
  html += F("th,td{padding:10px;border-bottom:1px solid #dbe1ea;text-align:left}th{background:#eef2f7}");
  html += F("input,select{width:100%;box-sizing:border-box;padding:8px;margin:4px 0 10px}button,.btn{display:inline-block;padding:9px 12px;margin:4px 2px;border:0;background:#1f6feb;color:white;text-decoration:none;border-radius:4px}");
  html += F(".danger{background:#b42318}.muted{color:#667085}.card{background:white;padding:14px;margin:14px 0;border:1px solid #dbe1ea;border-radius:6px}");
  html += F(".led{width:18px;height:18px;border-radius:50%;background:#111;border:1px solid #444;box-shadow:inset 0 0 4px #000;display:inline-block}");
  html += F(".led.on{background:#ffd21a;border-color:#b58b00;box-shadow:0 0 12px #ffd21a}.center{text-align:center}");
  html += F("</style></head><body><main><h1>Приемник ESP32 CC1101</h1>");
}

static void appendPageFooter(String &html) {
  html += F("</main></body></html>");
}

static void appendRs485Config(String &html) {
  html += F("<div class='card'><h2>RS-485 RP2040</h2>");
  html += F("<p><a class='btn' href='/rs485/discover'>Найти узлы</a></p>");
  html += F("<p class='muted'>DISCOVER: ");
  html += htmlEscape(rs485DiscoverLog.length() ? rs485DiscoverLog : "-");
  html += F("</p><div>");
  for (uint8_t n = 0; n < RS485_NODE_COUNT; ++n) {
    html += F("<button type='button' onclick='showNode(");
    html += String(n);
    html += F(")'>RP2040 ");
    html += String(n + 1);
    html += F("</button>");
  }
  html += F("</div>");

  for (uint8_t n = 0; n < RS485_NODE_COUNT; ++n) {
    Rs485NodeConfig &node = rs485Nodes[n];
    html += F("<div class='card rsnode' id='rsnode");
    html += String(n);
    html += F("'><h3>");
    html += htmlEscape(node.name);
    html += F("</h3><form method='post' action='/rs485/save'>");
    html += F("<input type='hidden' name='idx' value='");
    html += String(n);
    html += F("'><label><input type='checkbox' name='enabled' value='1'");
    if (node.enabled) html += F(" checked");
    html += F("> Включен</label>");
    html += F("<label>Название</label><input name='name' maxlength='31' value='");
    html += htmlEscape(node.name);
    html += F("'><label>Адрес RS-485</label><input name='addr' type='number' min='1' max='247' value='");
    html += String(node.address);
    html += F("'><label>UID RP2040</label><input name='uid' maxlength='32' value='");
    html += htmlEscape(node.uid);
    html += F("'><label>Порог отсутствия тока, мА</label><input name='zero' type='number' min='1' max='1000' value='");
    html += String(node.zeroCurrentMa);
    html += F("'><label>Максимальное время движения, мс</label><input name='move' type='number' min='1000' max='180000' value='");
    html += String(node.maxMoveMs);
    html += F("'><table><tr><th>Актуатор</th><th>Макс. ток, мА</th><th>Ток</th><th>INA219</th></tr>");
    for (uint8_t a = 0; a < RS485_ACTUATORS; ++a) {
      html += F("<tr><td>");
      html += String(a + 1);
      html += F("</td><td><input name='max");
      html += String(a);
      html += F("' type='number' min='100' max='10000' value='");
      html += String(node.maxCurrentMa[a]);
      html += F("'></td><td id='rs");
      html += String(n);
      html += F("cur");
      html += String(a);
      html += F("'>-</td><td id='rs");
      html += String(n);
      html += F("ina");
      html += String(a);
      html += F("'>-</td></tr>");
    }
    html += F("</table><p><button type='submit'>Сохранить и отправить</button></p></form>");
    html += F("<form method='post' action='/rs485/cmd'><input type='hidden' name='idx' value='");
    html += String(n);
    html += F("'><button name='mode' value='open'>Открыто</button><button name='mode' value='closed'>Закрыто</button><button name='mode' value='vent'>Проветривание</button><button class='danger' name='mode' value='stop'>Стоп</button><button name='mode' value='status'>Статус</button></form>");
    html += F("<form method='post' action='/rs485/address'><input type='hidden' name='idx' value='");
    html += String(n);
    html += F("'><label>Новый адрес по UID</label><input name='newaddr' type='number' min='1' max='247' value='");
    html += String(node.address);
    html += F("'><button type='submit'>Записать адрес в RP2040</button></form>");
    html += F("<p>Статус: <span id='rs");
    html += String(n);
    html += F("status'>-</span></p></div>");
  }
  html += F("</div>");
  html += F("<script>function showNode(i){document.querySelectorAll('.rsnode').forEach((e,k)=>e.style.display=k==i?'block':'none')}showNode(0);</script>");
}

static void handleRoot() {
  if (!webAuth()) return;
  String html;
  html.reserve(7000);
  appendPageHeader(html, "Window controller");
  html += F("<div class='card'><h2>Управление окнами</h2><p id='wstatus'>Загрузка статуса...</p>");
  html += F("<form method='post' action='/window/cmd'><label>Выберите окно</label><select id='target' name='target'><option value='local'");
  if (strcmp(mainWindowTarget, "local") == 0) html += F(" selected");
  html += F(">");
  html += htmlEscape(localRpName);
  html += F("</option>");
  for (uint8_t i = 0; i < RS485_NODE_COUNT; ++i) {
    if (!rs485Nodes[i].enabled) continue;
    String targetValue = "rs" + String(i);
    html += F("<option value='rs");
    html += String(i);
    html += F("'");
    if (targetValue == mainWindowTarget) html += F(" selected");
    html += F(">");
    html += htmlEscape(rs485Nodes[i].name);
    html += F(" / адрес ");
    html += String(rs485Nodes[i].address);
    html += F("</option>");
  }
  html += F("</select><button name='mode' value='open' style='width:100%;font-size:22px;padding:18px' type='submit'>Открыто</button>");
  html += F("<button name='mode' value='closed' style='width:100%;font-size:22px;padding:18px;background:#475467' type='submit'>Закрыто</button>");
  html += F("<button name='mode' value='vent' style='width:100%;font-size:22px;padding:18px' type='submit'>Проветривание</button>");
  html += F("<button name='mode' value='stop' class='danger' type='submit'>Стоп</button></form>");
  html += F("<p><a href='/config'>config</a></p></div>");
  html += F("<script>function n(v){return {open:'Открыто',closed:'Закрыто',vent:'Проветривание',none:'неизвестно'}[v]||v}async function upd(){try{let t=document.getElementById('target').value;let r=await fetch('/api/window?target='+encodeURIComponent(t),{cache:'no-store'});let s=await r.json();document.getElementById('wstatus').innerHTML='<b>Окно:</b> '+(s.name||'-')+'<br><b>Состояние:</b> '+(s.state||'unknown')+'<br><b>Цель:</b> '+n(s.target)+'<br><b>Положение:</b> '+n(s.position)+'<br><b>Авария:</b> '+(s.fault||'none')+(s.faultActuator?(' актуатор '+s.faultActuator):'');}catch(e){document.getElementById('wstatus').textContent='Нет связи';}}document.getElementById('target').addEventListener('change',upd);setInterval(upd,700);upd();</script>");
  appendPageFooter(html);
  server.send(200, "text/html; charset=utf-8", html);
}

static void handleConfig() {
  if (!webAuth()) return;
  String html;
  html.reserve(17000);
  appendPageHeader(html, "ESP32 CC1101 receiver");
  html += F("<p><a class='btn' href='/'>Главная</a></p>");
  html += F("<div class='card'><h2>Настройки окон</h2><form method='post' action='/window/save'>");
  html += F("<label>Имя локальной RP2040</label><input name='localName' maxlength='31' value='");
  html += htmlEscape(localRpName);
  html += F("'>");
  html += F("<label>Количество окон</label><select name='windows'><option value='1'");
  if (windowCount == 1) html += F(" selected");
  html += F(">1 окно</option><option value='2'");
  if (windowCount == 2) html += F(" selected");
  html += F(">2 окна</option></select>");
  html += F("<label>Порог отсутствия тока, мА</label><input name='zeroMa' type='number' value='");
  html += String(windowZeroCurrentMa);
  html += F("'><label>Максимальное время движения, мс</label><input name='maxMove' type='number' value='");
  html += String(windowMaxMoveMs);
  html += F("'><table><tr><th>Актуатор</th><th>Макс. ток, мА</th><th>Ток</th><th>INA219</th></tr>");
  for (uint8_t i = 0; i < WINDOW_ACTUATORS; ++i) {
    html += F("<tr><td>");
    html += String(i + 1);
    html += F("</td><td><input name='max");
    html += String(i);
    html += F("' type='number' value='");
    html += String(windowMaxCurrentMa[i]);
    html += F("'></td><td id='cur");
    html += String(i);
    html += F("'>-</td><td id='ina");
    html += String(i);
    html += F("'>-</td></tr>");
  }
  html += F("</table><p><button type='submit'>Сохранить настройки окон</button></p></form>");
  html += F("<p>Герконы: <span id='reeds'>-</span><br>CAP1188: <span id='cap'>-</span><br>Авария: <span id='wfault'>-</span></p></div>");
  appendRs485Config(html);
  html += F("<div class='card'><b>Состояние:</b> ");
  html += WiFi.getMode() == WIFI_AP ? "режим точки доступа" : "подключен к Wi-Fi";
  html += F("<br><b>IP-адрес:</b> ");
  html += WiFi.getMode() == WIFI_AP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  html += F("<br><b>Wi-Fi SSID:</b> ");
  html += htmlEscape(wifiSsid);
  html += F("<br><b>CC1101:</b> ");
  html += radioReady ? "OK" : "не найден, пульты отключены";
  html += F("<br><b>Режим обучения:</b> ");
  html += learnMode ? "ВКЛ" : "ВЫКЛ";
  html += F("<br><b>Последний принятый код:</b> <span id='lastSeen'>");
  html += lastSeenCode == 0 ? String("-") : codeHex(lastSeenCode);
  html += F("</span><br><b>Последняя распознанная кнопка:</b> <span id='lastButton'>");
  html += lastMatchedCode == 0 ? String("-") : htmlEscape(lastMatchedName);
  html += F("</span><br><b>Код кнопки:</b> <span id='lastMatched'>");
  html += lastMatchedCode == 0 ? String("-") : codeHex(lastMatchedCode);
  html += F("</span><br><b>Назначенное действие:</b> <span id='lastOutput'>");
  if (lastMatchedIndex >= 0 && lastMatchedIndex < recordCount) html += htmlEscape(rfActionLabel(records[lastMatchedIndex]));
  else html += F("-");
  html += F("</span><br><b>Прошло после нажатия:</b> <span id='lastAge'>-</span></div>");

  html += F("<p><a id='learnBtn' class='btn");
  if (learnMode) html += F(" danger");
  html += F("' href='/learn'>");
  html += learnMode ? "Закончить обучение" : "Добавить пульт";
  html += F("</a><a class='btn danger' href='/clear' onclick=\"return confirm('Удалить все сохраненные кнопки?')\">Очистить все</a>");
  html += F("<a class='btn danger' href='/restart' onclick=\"return confirm('Перезагрузить ESP32?')\">Рестарт</a></p>");

  html += F("<div class='card'><h2>Wi-Fi роутера</h2><form method='post' action='/wifi'>");
  html += F("<label>SSID</label><input name='ssid' maxlength='32' value=\"");
  html += htmlEscape(wifiSsid);
  html += F("\"><label>Пароль</label><input name='pass' maxlength='64' type='password' value=\"");
  html += htmlEscape(wifiPassword);
  html += F("\"><p><button type='submit'>Сохранить Wi-Fi</button></p>");
  html += F("<p class='muted'>Новые данные Wi-Fi начнут использоваться после рестарта ESP32.</p></form></div>");

  html += F("<form method='post' action='/save'><table><tr><th>#</th><th>Код</th><th>Название</th><th>Действие</th><th>Команда</th><th>RS-485</th><th>Лок. выход</th><th>Включено</th><th>Индикатор</th><th>Удалить</th></tr>");
  for (uint8_t i = 0; i < recordCount; ++i) {
    const uint16_t remoteId = remoteIdFromCode(records[i].code);
    if (i == 0 || remoteIdFromCode(records[i - 1].code) != remoteId) {
      uint8_t remoteNumber = 1;
      for (uint8_t j = 1; j <= i; ++j) {
        if (remoteIdFromCode(records[j].code) != remoteIdFromCode(records[j - 1].code)) ++remoteNumber;
      }
      uint8_t buttonsInRemote = 0;
      for (uint8_t j = i; j < recordCount && remoteIdFromCode(records[j].code) == remoteId; ++j) ++buttonsInRemote;
      html += F("<tr><th colspan='10'>Пульт ");
      html += String(remoteNumber);
      html += F(" / адрес ");
      html += remoteHex(remoteId);
      html += F(" / кнопок: ");
      html += String(buttonsInRemote);
      html += F("</th></tr>");
    }
    uint8_t buttonNumber = 1;
    for (uint8_t j = i; j > 0 && remoteIdFromCode(records[j - 1].code) == remoteId; --j) ++buttonNumber;
    html += F("<tr><td>");
    html += String(buttonNumber);
    html += F("</td><td>");
    html += codeHex(records[i].code);
    html += F("</td><td><input name='name");
    html += String(i);
    html += F("' maxlength='23' value=\"");
    html += htmlEscape(records[i].name);
    html += F("\"></td><td><select name='act");
    html += String(i);
    html += F("'><option value='0'");
    if (records[i].actionType == 0) html += F(" selected");
    html += F(">Локальный выход ESP32</option><option value='1'");
    if (records[i].actionType == 1) html += F(" selected");
    html += F(">Локальная RP2040 UART</option><option value='2'");
    if (records[i].actionType == 2) html += F(" selected");
    html += F(">RP2040 RS-485</option></select></td><td><select name='cmd");
    html += String(i);
    html += F("'><option value='0'");
    if (records[i].actionCommand == 0) html += F(" selected");
    html += F(">-</option><option value='1'");
    if (records[i].actionCommand == 1) html += F(" selected");
    html += F(">Открыто</option><option value='2'");
    if (records[i].actionCommand == 2) html += F(" selected");
    html += F(">Закрыто</option><option value='3'");
    if (records[i].actionCommand == 3) html += F(" selected");
    html += F(">Проветривание</option><option value='4'");
    if (records[i].actionCommand == 4) html += F(" selected");
    html += F(">Стоп</option></select></td><td><select name='node");
    html += String(i);
    html += F("'>");
    for (uint8_t node = 0; node < RS485_NODE_COUNT; ++node) {
      html += F("<option value='");
      html += String(node);
      html += "'";
      if (records[i].rs485Node == node) html += F(" selected");
      html += F(">");
      html += String(node + 1);
      html += F(" / addr ");
      html += String(rs485Nodes[node].address);
      html += F(" / ");
      html += htmlEscape(rs485Nodes[node].name);
      html += F("</option>");
    }
    html += F("</select></td><td><select name='out");
    html += String(i);
    html += F("'><option value='255'>Не назначен</option>");
    for (uint8_t out = 0; out < OUTPUT_COUNT; ++out) {
      html += F("<option value='");
      html += String(out);
      html += "'";
      if (records[i].output == out) html += F(" selected");
      html += F(">Выход ");
      html += String(out + 1);
      html += F(" GPIO ");
      html += String(OUTPUT_PINS[out]);
      html += F("</option>");
    }
    html += F("</select></td><td><input type='checkbox' name='en");
    html += String(i);
    html += F("' value='1'");
    if (records[i].enabled) html += F(" checked");
    html += F("></td><td class='center'><span class='led' id='led");
    html += String(i);
    html += F("'></span></td><td><input type='checkbox' name='del");
    html += String(i);
    html += F("' value='1'></td></tr>");
  }
  html += F("</table><p><button type='submit'>Сохранить</button></p></form>");
  html += F("<p class='muted'>Обучение: нажмите физическую кнопку или «Добавить пульт», затем нажимайте кнопки пульта. Повторное короткое нажатие или «Закончить обучение» возвращает рабочий режим.</p>");
  html += F("<script>let activeLed=-1;function setLed(i,on){const e=document.getElementById('led'+i);if(e)e.classList.toggle('on',on);}async function rfStatus(){try{const r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)return;const s=await r.json();document.getElementById('lastSeen').textContent=s.lastSeen||'-';document.getElementById('lastButton').textContent=s.lastButton||'-';document.getElementById('lastMatched').textContent=s.lastMatched||'-';document.getElementById('lastOutput').textContent=s.lastOutput||'-';document.getElementById('lastAge').textContent=s.lastMatchedAgeMs>=0?((s.lastMatchedAgeMs/1000).toFixed(1)+' s'):'-';const b=document.getElementById('learnBtn');if(b){b.textContent=s.learnMode?'Закончить обучение':'Добавить пульт';b.classList.toggle('danger',s.learnMode);}if(activeLed!==s.lastMatchedIndex){if(activeLed>=0)setLed(activeLed,false);activeLed=s.lastMatchedIndex;}if(s.lastMatchedAgeMs>=0&&s.lastMatchedAgeMs<700)setLed(s.lastMatchedIndex,true);else if(activeLed>=0)setLed(activeLed,false);}catch(e){}}setInterval(rfStatus,500);rfStatus();</script>");
  html += F("<script>async function winStatus(){try{const r=await fetch('/api/window',{cache:'no-store'});const s=await r.json();const cur=s.current||[];const ok=s.inaOk||[];for(let i=0;i<8;i++){let c=document.getElementById('cur'+i);if(c)c.textContent=(cur[i]??0)+' mA';let o=document.getElementById('ina'+i);if(o)o.textContent=ok[i]?'OK':'нет';}document.getElementById('reeds').textContent=(s.reed||[]).join(', ');document.getElementById('cap').textContent='0x'+Number(s.cap||0).toString(16);document.getElementById('wfault').textContent=(s.fault||'none')+(s.faultActuator?(' actuator '+s.faultActuator):'');}catch(e){}}setInterval(winStatus,700);winStatus();</script>");
  html += F("<script>async function rsStatus(){try{const r=await fetch('/api/rs485',{cache:'no-store'});const d=await r.json();(d.nodes||[]).forEach((n,i)=>{let e=document.getElementById('rs'+i+'status');if(e)e.textContent=n.status||'-';let s=n.json||{};let cur=s.current||[];let ok=s.inaOk||[];for(let a=0;a<4;a++){let c=document.getElementById('rs'+i+'cur'+a);if(c)c.textContent=(cur[a]??0)+' mA';let o=document.getElementById('rs'+i+'ina'+a);if(o)o.textContent=ok[a]?'OK':'нет';}})}catch(e){}}setInterval(rsStatus,1000);rsStatus();</script>");
  appendPageFooter(html);
  server.send(200, "text/html", html);
}

static void handleStatusApi() {
  if (!webAuth()) return;
  const uint32_t now = millis();
  String json;
  json.reserve(360);
  json += F("{\"learnMode\":");
  json += learnMode ? F("true") : F("false");
  json += F(",\"lastSeen\":\"");
  json += lastSeenCode == 0 ? String("") : codeHex(lastSeenCode);
  json += F("\",\"lastSeenAgeMs\":");
  json += lastSeenCode == 0 ? String(-1) : String(now - lastSeenMs);
  json += F(",\"lastButton\":\"");
  json += htmlEscape(lastMatchedName);
  json += F("\",\"lastMatched\":\"");
  json += lastMatchedCode == 0 ? String("") : codeHex(lastMatchedCode);
  json += F("\",\"lastMatchedAgeMs\":");
  json += lastMatchedCode == 0 ? String(-1) : String(now - lastMatchedMs);
  json += F(",\"lastMatchedIndex\":");
  json += String(lastMatchedIndex);
  json += F(",\"lastOutput\":\"");
  if (lastMatchedIndex >= 0 && lastMatchedIndex < recordCount) json += htmlEscape(rfActionLabel(records[lastMatchedIndex]));
  json += F("\"}");
  server.send(200, "application/json", json);
}

static void handleLearn() {
  if (!webAuth()) return;
  toggleLearnMode();
  server.sendHeader("Location", "/config");
  server.send(303);
}

static void handleClear() {
  if (!webAuth()) return;
  clearRecords();
  server.sendHeader("Location", "/config");
  server.send(303);
}

static void handleRestart() {
  if (!webAuth()) return;
  server.send(200, "text/html; charset=utf-8",
              "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
              "<title>Рестарт ESP32</title></head><body><h1>ESP32 перезагружается</h1><p>Подождите несколько секунд и обновите страницу.</p></body></html>");
  delay(300);
  ESP.restart();
}

static void handleWifiSave() {
  if (!webAuth()) return;
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  ssid.trim();
  if (ssid.length() == 0) {
    server.send(400, "text/plain; charset=utf-8", "SSID не может быть пустым");
    return;
  }
  if (ssid.length() >= WIFI_SSID_LEN || pass.length() >= WIFI_PASSWORD_LEN) {
    server.send(400, "text/plain; charset=utf-8", "SSID или пароль слишком длинный");
    return;
  }
  saveWifiSettings(ssid, pass);
  server.sendHeader("Location", "/config");
  server.send(303);
}

static void handleSave() {
  if (!webAuth()) return;
  ButtonRecord newRecords[MAX_CODES];
  uint8_t newCount = 0;
  for (uint8_t i = 0; i < recordCount; ++i) {
    if (server.hasArg("del" + String(i))) continue;
    ButtonRecord record = records[i];
    const String nameKey = "name" + String(i);
    const String outKey = "out" + String(i);
    const String actKey = "act" + String(i);
    const String cmdKey = "cmd" + String(i);
    const String nodeKey = "node" + String(i);
    const String enKey = "en" + String(i);
    if (server.hasArg(nameKey)) strlcpy(record.name, server.arg(nameKey).c_str(), sizeof(record.name));
    if (server.hasArg(outKey)) record.output = static_cast<uint8_t>(server.arg(outKey).toInt());
    if (server.hasArg(actKey)) record.actionType = static_cast<uint8_t>(constrain(server.arg(actKey).toInt(), 0, 2));
    if (server.hasArg(cmdKey)) record.actionCommand = static_cast<uint8_t>(constrain(server.arg(cmdKey).toInt(), 0, 4));
    if (server.hasArg(nodeKey)) record.rs485Node = static_cast<uint8_t>(constrain(server.arg(nodeKey).toInt(), 0, RS485_NODE_COUNT - 1));
    record.enabled = server.hasArg(enKey);
    newRecords[newCount++] = record;
  }
  memcpy(records, newRecords, sizeof(records));
  recordCount = newCount;
  saveRecords();
  server.sendHeader("Location", "/config");
  server.send(303);
}

static void handleWindowCommand() {
  if (!webAuth()) return;
  const String mode = server.arg("mode");
  const String target = server.hasArg("target") ? server.arg("target") : "local";
  strlcpy(mainWindowTarget, target.c_str(), sizeof(mainWindowTarget));
  saveWindowSettings();
  uint8_t command = 0;
  if (mode == "open") command = 1;
  else if (mode == "closed") command = 2;
  else if (mode == "vent") command = 3;
  else if (mode == "stop") command = 4;
  sendWindowTargetCommand(target, windowCommandLine(command));
  server.sendHeader("Location", "/");
  server.send(303);
}

static void handleWindowSave() {
  if (!webAuth()) return;
  if (server.hasArg("localName")) strlcpy(localRpName, server.arg("localName").c_str(), sizeof(localRpName));
  windowCount = constrain(server.arg("windows").toInt(), 1, 2);
  windowZeroCurrentMa = constrain(server.arg("zeroMa").toInt(), 1, 1000);
  windowMaxMoveMs = constrain(server.arg("maxMove").toInt(), 1000, 180000);
  for (uint8_t i = 0; i < WINDOW_ACTUATORS; ++i) {
    String key = "max" + String(i);
    if (server.hasArg(key)) windowMaxCurrentMa[i] = constrain(server.arg(key).toInt(), 100, 10000);
  }
  saveWindowSettings();
  sendWindowConfigToRp2040();
  server.sendHeader("Location", "/config");
  server.send(303);
}

static void handleWindowApi() {
  if (!webAuth()) return;
  const String target = server.hasArg("target") ? server.arg("target") : "local";
  String body;
  String status = lastRpStatus;
  String name = localRpName;
  uint32_t ageMs = millis() - lastRpStatusMs;
  if (target.startsWith("rs")) {
    const int idx = target.substring(2).toInt();
    if (idx >= 0 && idx < RS485_NODE_COUNT) {
      status = rs485Status[idx];
      name = rs485Nodes[idx].name;
      ageMs = millis() - rs485LastSeenMs[idx];
    }
  }
  if (status.startsWith("{") && status.endsWith("}")) {
    body = status;
    body.remove(body.length() - 1);
    body += F(",\"name\":\"");
    body += htmlEscape(name);
    body += F("\",\"ageMs\":");
    body += String(ageMs);
    body += F("}");
  } else {
    body = F("{\"state\":\"unknown\",\"target\":\"none\",\"position\":\"none\",\"fault\":\"no_rp2040\",\"faultActuator\":0,\"current\":[],\"reed\":[],\"inaOk\":[],\"cap\":0,\"name\":\"");
    body += htmlEscape(name);
    body += F("\",\"ageMs\":0}");
  }
  server.send(200, "application/json", body);
}

static void handleRs485Command() {
  if (!webAuth()) return;
  const uint8_t idx = constrain(server.arg("idx").toInt(), 0, RS485_NODE_COUNT - 1);
  const String mode = server.arg("mode");
  const uint8_t addr = rs485Nodes[idx].address;
  if (mode == "open") sendRs485Line("@" + String(addr) + " CMD OPEN");
  else if (mode == "closed") sendRs485Line("@" + String(addr) + " CMD CLOSED");
  else if (mode == "vent") sendRs485Line("@" + String(addr) + " CMD VENT");
  else if (mode == "stop") sendRs485Line("@" + String(addr) + " CMD STOP");
  else if (mode == "status") sendRs485Line("@" + String(addr) + " STATUS");
  server.sendHeader("Location", "/config");
  server.send(303);
}

static void handleRs485Save() {
  if (!webAuth()) return;
  const uint8_t idx = constrain(server.arg("idx").toInt(), 0, RS485_NODE_COUNT - 1);
  Rs485NodeConfig &node = rs485Nodes[idx];
  node.enabled = server.hasArg("enabled");
  node.address = constrain(server.arg("addr").toInt(), 1, 247);
  strlcpy(node.name, server.arg("name").c_str(), sizeof(node.name));
  strlcpy(node.uid, server.arg("uid").c_str(), sizeof(node.uid));
  node.zeroCurrentMa = constrain(server.arg("zero").toInt(), 1, 1000);
  node.maxMoveMs = constrain(server.arg("move").toInt(), 1000, 180000);
  for (uint8_t a = 0; a < RS485_ACTUATORS; ++a) {
    const String key = "max" + String(a);
    if (server.hasArg(key)) node.maxCurrentMa[a] = constrain(server.arg(key).toInt(), 100, 10000);
  }
  saveRs485Settings();
  sendRs485NodeConfig(idx);
  server.sendHeader("Location", "/config");
  server.send(303);
}

static void handleRs485Discover() {
  if (!webAuth()) return;
  rs485DiscoverLog = "";
  sendRs485Line("@0 DISCOVER");
  server.sendHeader("Location", "/config");
  server.send(303);
}

static void handleRs485Address() {
  if (!webAuth()) return;
  const uint8_t idx = constrain(server.arg("idx").toInt(), 0, RS485_NODE_COUNT - 1);
  Rs485NodeConfig &node = rs485Nodes[idx];
  const uint8_t newAddr = constrain(server.arg("newaddr").toInt(), 1, 247);
  if (strlen(node.uid) > 0) {
    sendRs485Line("@0 SETADDR UID=" + String(node.uid) + " ADDR=" + String(newAddr));
    node.address = newAddr;
    saveRs485Settings();
  }
  server.sendHeader("Location", "/config");
  server.send(303);
}

static void handleRs485Api() {
  if (!webAuth()) return;
  String json;
  json.reserve(3000);
  json += F("{\"nodes\":[");
  for (uint8_t i = 0; i < RS485_NODE_COUNT; ++i) {
    if (i) json += ',';
    json += F("{\"enabled\":");
    json += rs485Nodes[i].enabled ? F("true") : F("false");
    json += F(",\"address\":");
    json += String(rs485Nodes[i].address);
    json += F(",\"name\":\"");
    json += htmlEscape(rs485Nodes[i].name);
    json += F("\",\"ageMs\":");
    json += rs485LastSeenMs[i] ? String(millis() - rs485LastSeenMs[i]) : String(-1);
    json += F(",\"status\":\"");
    String status = rs485Status[i];
    status.replace("\\", "\\\\");
    status.replace("\"", "\\\"");
    json += status;
    json += F("\",\"json\":");
    if (rs485Status[i].startsWith("{") && rs485Status[i].endsWith("}")) json += rs485Status[i];
    else json += F("{}");
    json += F("}");
  }
  json += F("]}");
  server.send(200, "application/json", json);
}

static void beginWeb() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.print("[WIFI] Connecting");
  for (uint8_t i = 0; i < 40 && WiFi.status() != WL_CONNECTED; ++i) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WIFI] IP: ");
    Serial.println(WiFi.localIP());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("[WIFI] AP IP: ");
    Serial.println(WiFi.softAPIP());
  }
  server.on("/", HTTP_GET, handleRoot);
  server.on("/learn", HTTP_GET, handleLearn);
  server.on("/clear", HTTP_GET, handleClear);
  server.on("/restart", HTTP_GET, handleRestart);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/wifi", HTTP_POST, handleWifiSave);
  server.on("/api/status", HTTP_GET, handleStatusApi);
  server.on("/config", HTTP_GET, handleConfig);
  server.on("/window/cmd", HTTP_POST, handleWindowCommand);
  server.on("/window/save", HTTP_POST, handleWindowSave);
  server.on("/api/window", HTTP_GET, handleWindowApi);
  server.on("/rs485/cmd", HTTP_POST, handleRs485Command);
  server.on("/rs485/save", HTTP_POST, handleRs485Save);
  server.on("/rs485/discover", HTTP_GET, handleRs485Discover);
  server.on("/rs485/address", HTTP_POST, handleRs485Address);
  server.on("/api/rs485", HTTP_GET, handleRs485Api);
  server.begin();
}

static void readRp2040() {
  while (rpSerial.available()) {
    const char c = static_cast<char>(rpSerial.read());
    if (c == '\n' || c == '\r') {
      rpLine.trim();
      if (rpLine.length()) {
        lastRpStatus = rpLine;
        lastRpStatusMs = millis();
        Serial.print("[RP2040] < ");
        Serial.println(lastRpStatus);
      }
      rpLine = "";
    } else if (rpLine.length() < 900) {
      rpLine += c;
    }
  }
}

static void handleRs485Line(String line) {
  line.trim();
  if (!line.startsWith("@")) return;
  const int space = line.indexOf(' ');
  if (space < 0) return;
  const int addr = line.substring(1, space).toInt();
  const String payload = line.substring(space + 1);
  Serial.print("[RS485] < ");
  Serial.println(line);

  if (payload.startsWith("DISCOVER ")) {
    rs485DiscoverLog += payload;
    rs485DiscoverLog += " | ";
    const int uidPos = payload.indexOf("UID=");
    const int addrPos = payload.indexOf("ADDR=");
    if (uidPos >= 0 && addrPos >= 0) {
      String uid = payload.substring(uidPos + 4, payload.indexOf(' ', uidPos + 4) > 0 ? payload.indexOf(' ', uidPos + 4) : payload.length());
      const int foundAddr = payload.substring(addrPos + 5).toInt();
      for (uint8_t i = 0; i < RS485_NODE_COUNT; ++i) {
        if (rs485Nodes[i].address == foundAddr || strlen(rs485Nodes[i].uid) == 0) {
          strlcpy(rs485Nodes[i].uid, uid.c_str(), sizeof(rs485Nodes[i].uid));
          rs485Nodes[i].address = constrain(foundAddr, 1, 247);
          saveRs485Settings();
          break;
        }
      }
    }
    return;
  }

  for (uint8_t i = 0; i < RS485_NODE_COUNT; ++i) {
    if (rs485Nodes[i].address == addr) {
      rs485Status[i] = payload;
      rs485LastSeenMs[i] = millis();
      break;
    }
  }
}

static void readRs485() {
  while (rs485Serial.available()) {
    const char c = static_cast<char>(rs485Serial.read());
    if (c == '\n' || c == '\r') {
      if (rs485Line.length()) {
        handleRs485Line(rs485Line);
        rs485Line = "";
      }
    } else if (rs485Line.length() < 1000) {
      rs485Line += c;
    }
  }
}

static void handleLearnButton() {
  static bool pressedState = false;
  static bool longHandled = false;
  static uint32_t pressedAt = 0;
  static uint32_t lastChangeMs = 0;
  const bool pressed = digitalRead(PIN_LEARN_BUTTON) == LOW;
  const uint32_t now = millis();
  if (pressed != pressedState && now - lastChangeMs > 50) {
    lastChangeMs = now;
    pressedState = pressed;
    if (pressed) {
      pressedAt = now;
      longHandled = false;
    } else if (!longHandled && now - pressedAt < LEARN_BUTTON_LONG_PRESS_MS) {
      toggleLearnMode();
    }
  }
  if (pressedState && !longHandled && now - pressedAt >= LEARN_BUTTON_LONG_PRESS_MS) {
    longHandled = true;
    stopLearnMode();
    clearRecords();
    flashRgb(80, 0, 0);
  }
  if (learnMode && millis() - learnStartedMs > LEARN_TIMEOUT_MS) stopLearnMode();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("===============================================================");
  Serial.println("ESP32-S3 + CC1101 RF receiver controller");
  Serial.println("===============================================================");
  pinMode(PIN_LEARN_BUTTON, INPUT_PULLUP);
  setRgb(0, 0, 0);
  for (uint8_t i = 0; i < OUTPUT_COUNT; ++i) {
    pinMode(OUTPUT_PINS[i], OUTPUT);
    setOutput(i, false);
  }
  loadRecords();
  loadWifiSettings();
  loadWindowSettings();
  loadRs485Settings();
  rpSerial.begin(RP2040_UART_BAUD, SERIAL_8N1, RP2040_UART_RX_PIN, RP2040_UART_TX_PIN);
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  setRs485Tx(false);
  rs485Serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  Serial.print("[BOOT] Learned records: ");
  Serial.println(recordCount);
  radioReady = beginRadio();
  if (radioReady) {
    resetRfCapture();
  } else {
    Serial.println("[WARN] CC1101 not found, RF remotes are disabled");
  }
  beginWeb();
  sendWindowConfigToRp2040();
  sendRpCommand("STATUS");
  sendRs485Line("@0 DISCOVER");
}

void loop() {
  server.handleClient();
  readRp2040();
  readRs485();
  static uint32_t lastRpStatusAsk = 0;
  if (millis() - lastRpStatusAsk > 1000) {
    lastRpStatusAsk = millis();
    sendRpCommand("STATUS");
  }
  static uint32_t lastRs485Poll = 0;
  static uint8_t rs485PollIndex = 0;
  if (millis() - lastRs485Poll > 250) {
    lastRs485Poll = millis();
    for (uint8_t tries = 0; tries < RS485_NODE_COUNT; ++tries) {
      rs485PollIndex = (rs485PollIndex + 1) % RS485_NODE_COUNT;
      if (rs485Nodes[rs485PollIndex].enabled) {
        sendRs485Line("@" + String(rs485Nodes[rs485PollIndex].address) + " STATUS");
        break;
      }
    }
  }
  handleLearnButton();
  processRf();
  updateOutputs();
  updateRgb();
}
