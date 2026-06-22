#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <RadioLib.h>
#include <Wire.h>
#include <errno.h>
#include <limits.h>
#include "heltec_lora32_v4_pins.h"

// LoRa frequency in MHz
const float LORA_FREQ = 868.0;

// LoRa parameters matching Tomos Tracker
const float LORA_BANDWIDTH = 125.0;       // 125 kHz
const uint8_t LORA_SPREADING_FACTOR = 7;  // SF7
const uint8_t LORA_CODING_RATE = 5;       // CR4/5
const uint8_t LORA_SYNC_WORD = 0x12;      // Custom sync word
const int16_t LORA_POWER = 14;             // dBm

// Heltec WiFi LoRa 32 V4 OLED pins from Heltec's official V4 board definition.
const uint8_t OLED_ADDRESS = 0x3C;
const uint8_t OLED_SDA = 17;
const uint8_t OLED_SCL = 18;
const uint8_t OLED_RESET = 21;
const uint8_t OLED_VEXT = 36;
const uint16_t OLED_WIDTH = 128;
const uint16_t OLED_HEIGHT = 64;
const unsigned long DISPLAY_INTERVAL = 500; // At most two updates per second

// RadioLib objects
SPIClass spi;
Module* radio_module = nullptr;
SX1262* lora = nullptr;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// State tracking
bool lora_initialized = false;
volatile bool receivedFlag = false;
unsigned long last_status_print = 0;
const unsigned long STATUS_INTERVAL = 5000; // Print status every 5 seconds
uint32_t rxOkCount = 0;
uint32_t emptyPacketCount = 0;
uint32_t rxErrorCount = 0;
int lastRadioLibError = 0;
bool hasLastSeq = false;
uint32_t lastSeq = 0;
uint32_t lostPacketCount = 0;
uint32_t seqParseErrorCount = 0;
float lastRssi = 0.0f;
float lastSnr = 0.0f;
bool hasLastSignal = false;
bool displayInitialized = false;
unsigned long lastDisplayUpdate = 0;
uint32_t telemetryParseOkCount = 0;
uint32_t telemetryParseErrorCount = 0;

struct TomosTelemetry {
  bool valid = false;
  uint32_t seq = 0;
  bool fix = false;
  double latitude = 0.0;
  double longitude = 0.0;
  float speedKmh = 0.0f;
  float maxSpeedKmh = 0.0f;
  int satellites = 0;
  float hdop = 99.99f;
  float batteryVoltage = 0.0f;
  char time[16] = "--:--:--";
};

TomosTelemetry lastTelemetry;

float calculateLossPercent() {
  uint32_t expectedPackets = rxOkCount + lostPacketCount;
  if (expectedPackets == 0) {
    return 0.0f;
  }
  return 100.0f * lostPacketCount / expectedPackets;
}

void showDisplayStartup() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("TOMOS PIT RX");
  display.println("Waiting LoRa...");
  display.display();
}

void initializeDisplay() {
  // Vext powers the onboard OLED and is active LOW on this V4 revision.
  pinMode(OLED_VEXT, OUTPUT);
  digitalWrite(OLED_VEXT, LOW);
  delay(100);

  // Release the SSD1306 from reset before probing its I2C address.
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, HIGH);
  delay(1);
  digitalWrite(OLED_RESET, LOW);
  delay(10);
  digitalWrite(OLED_RESET, HIGH);
  delay(10);

  if (!Wire.begin(OLED_SDA, OLED_SCL, 400000)) {
    Serial.println("OLED init FAIL - I2C initialization failed");
    return;
  }

  Wire.beginTransmission(OLED_ADDRESS);
  if (Wire.endTransmission() != 0) {
    Serial.println("OLED init FAIL - display not found at I2C address 0x3C");
    return;
  }

  // Wire is already configured with the V4 pins, so periphBegin stays false.
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, true, false)) {
    Serial.println("OLED init FAIL - SSD1306 initialization failed");
    return;
  }

  displayInitialized = true;
  display.setTextWrap(false);
  showDisplayStartup();
  lastDisplayUpdate = millis();
  Serial.println("OLED init SUCCESS (SDA=17, SCL=18, RST=21, Vext=36)");
}

void updateDisplay(unsigned long now) {
  if (!displayInitialized || now - lastDisplayUpdate < DISPLAY_INTERVAL) {
    return;
  }
  lastDisplayUpdate = now;

  char line[24];
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("TOMOS PIT RX");

  if (!lastTelemetry.valid) {
    display.println("Waiting...");
    snprintf(line, sizeof(line), "RX: %lu", rxOkCount);
    display.println(line);
    snprintf(line, sizeof(line), "ERR: %lu", rxErrorCount + telemetryParseErrorCount);
    display.println(line);
  } else {
    snprintf(line, sizeof(line), "SPD: %.1f km/h", lastTelemetry.speedKmh);
    display.println(line);
    if (lastTelemetry.fix) {
      snprintf(line, sizeof(line), "GPS: %d H%.1f", lastTelemetry.satellites,
               lastTelemetry.hdop);
    } else {
      snprintf(line, sizeof(line), "GPS: NO FIX");
    }
    display.println(line);
    snprintf(line, sizeof(line), "LOSS: %.1f%%", calculateLossPercent());
    display.println(line);
    snprintf(line, sizeof(line), "RSSI: %.0f", lastRssi);
    display.println(line);
    snprintf(line, sizeof(line), "SNR: %.1f", lastSnr);
    display.println(line);
  }

  display.display();
}

void setFlag() {
  receivedFlag = true;
}

bool getFieldValue(const String& payload, const char* key, String& valueOut) {
  String marker = String(",") + key + "=";
  int valueStart = payload.indexOf(marker);
  if (valueStart < 0) {
    return false;
  }

  valueStart += marker.length();
  int valueEnd = payload.indexOf(',', valueStart);
  if (valueEnd < 0) {
    valueEnd = payload.length();
  }

  if (valueStart >= valueEnd) {
    return false;
  }

  valueOut = payload.substring(valueStart, valueEnd);
  return true;
}

bool parseUnsigned32(const String& value, uint32_t& out) {
  if (value.length() == 0 || value[0] == '-') {
    return false;
  }

  errno = 0;
  char* end = nullptr;
  unsigned long parsed = strtoul(value.c_str(), &end, 10);
  if (errno == ERANGE || end == value.c_str() || *end != '\0' ||
      parsed > UINT32_MAX) {
    return false;
  }

  out = static_cast<uint32_t>(parsed);
  return true;
}

bool parseIntValue(const String& value, int& out) {
  errno = 0;
  char* end = nullptr;
  long parsed = strtol(value.c_str(), &end, 10);
  if (errno == ERANGE || end == value.c_str() || *end != '\0' ||
      parsed < INT_MIN || parsed > INT_MAX) {
    return false;
  }

  out = static_cast<int>(parsed);
  return true;
}

bool parseDoubleValue(const String& value, double& out) {
  errno = 0;
  char* end = nullptr;
  double parsed = strtod(value.c_str(), &end);
  if (errno == ERANGE || end == value.c_str() || *end != '\0' || !isfinite(parsed)) {
    return false;
  }

  out = parsed;
  return true;
}

bool parseFloatValue(const String& value, float& out) {
  errno = 0;
  char* end = nullptr;
  float parsed = strtof(value.c_str(), &end);
  if (errno == ERANGE || end == value.c_str() || *end != '\0' || !isfinite(parsed)) {
    return false;
  }

  out = parsed;
  return true;
}

bool parseTomosTelemetry(const String& payload, TomosTelemetry& out,
                         String& errorOut) {
  out = TomosTelemetry();
  if (!payload.startsWith("TOMOS,")) {
    errorOut = "payload does not start with TOMOS,";
    return false;
  }

  String value;
  if (!getFieldValue(payload, "seq", value) || !parseUnsigned32(value, out.seq)) {
    errorOut = "missing or invalid seq";
    return false;
  }

  if (!getFieldValue(payload, "fix", value) || (value != "0" && value != "1")) {
    errorOut = "missing or invalid fix";
    return false;
  }
  out.fix = value == "1";

  if (!getFieldValue(payload, "lat", value) || !parseDoubleValue(value, out.latitude)) {
    errorOut = "missing or invalid lat";
    return false;
  }
  if (!getFieldValue(payload, "lon", value) || !parseDoubleValue(value, out.longitude)) {
    errorOut = "missing or invalid lon";
    return false;
  }
  if (!getFieldValue(payload, "spd", value) || !parseFloatValue(value, out.speedKmh)) {
    errorOut = "missing or invalid spd";
    return false;
  }
  if (!getFieldValue(payload, "max", value) || !parseFloatValue(value, out.maxSpeedKmh)) {
    errorOut = "missing or invalid max";
    return false;
  }
  if (!getFieldValue(payload, "sat", value) || !parseIntValue(value, out.satellites)) {
    errorOut = "missing or invalid sat";
    return false;
  }
  if (!getFieldValue(payload, "hdop", value) || !parseFloatValue(value, out.hdop)) {
    errorOut = "missing or invalid hdop";
    return false;
  }
  if (!getFieldValue(payload, "vbat", value) ||
      !parseFloatValue(value, out.batteryVoltage)) {
    errorOut = "missing or invalid vbat";
    return false;
  }
  if (!getFieldValue(payload, "time", value) || value.length() > 15) {
    errorOut = "missing or invalid time";
    return false;
  }

  value.toCharArray(out.time, sizeof(out.time));
  out.time[sizeof(out.time) - 1] = '\0';
  out.valid = true;
  return true;
}

bool parseSeqFromPayload(const String& payload, uint32_t& seqOut) {
  int seqStart = payload.indexOf("seq=");
  if (seqStart < 0) {
    return false;
  }

  seqStart += 4;
  if (seqStart >= static_cast<int>(payload.length())) {
    return false;
  }

  uint32_t value = 0;
  bool hasDigit = false;
  for (size_t i = static_cast<size_t>(seqStart); i < payload.length(); i++) {
    char character = payload[i];
    if (character == ',') {
      break;
    }
    if (character < '0' || character > '9') {
      return false;
    }

    uint8_t digit = static_cast<uint8_t>(character - '0');
    if (value > (0xFFFFFFFFUL - digit) / 10UL) {
      return false;
    }
    value = value * 10UL + digit;
    hasDigit = true;
  }

  if (!hasDigit) {
    return false;
  }

  seqOut = value;
  return true;
}

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  delay(500);

  Serial.println("\n\n===============================================");
  Serial.println("Tomos Pit Receiver - Heltec WiFi LoRa 32 V4");
  Serial.println("===============================================");
  initializeDisplay();
  Serial.printf("Initializing LoRa Receiver...\n");
  Serial.printf("Target Frequency: %.1f MHz\n", LORA_FREQ);
  Serial.printf("Bandwidth: %.0f kHz, SF: %d, CR: %d, Sync Word: 0x%02X\n",
                LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODING_RATE, LORA_SYNC_WORD);

  // Initialize SPI with specified pins
  Serial.printf("Initializing SPI (SCK=%d, MISO=%d, MOSI=%d, CS=%d)...\n",
                LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  spi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  // Create RadioLib Module
  radio_module = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, spi);

  // Create SX1262 object
  lora = new SX1262(radio_module);

  // Initialize SX1262
  Serial.println("Initializing SX1262...");
  int state = lora->begin(LORA_FREQ, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                          LORA_CODING_RATE, LORA_SYNC_WORD, LORA_POWER);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa init FAIL - Error code: %d\n", state);
    lora_initialized = false;
  } else {
    Serial.println("LoRa init SUCCESS");
    lora_initialized = true;

    // Use DIO1 to signal that a packet is ready to be read.
    lora->setPacketReceivedAction(setFlag);
    state = lora->startReceive();
    if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("Failed to start receive mode - Error code: %d\n", state);
      rxErrorCount++;
      lastRadioLibError = state;
      lora_initialized = false;
    } else {
      Serial.println("LoRa Receiver started - listening for packets");
    }
  }

  last_status_print = millis();
  Serial.println("===============================================\n");
}

void loop() {
  unsigned long now = millis();

  updateDisplay(now);

  if (now - last_status_print >= STATUS_INTERVAL) {
    float lossPercent = calculateLossPercent();

    if (hasLastSignal) {
      Serial.printf("RX status: ok=%lu empty=%lu errors=%lu seq_errors=%lu telem_ok=%lu telem_errors=%lu lost=%lu loss=%.1f%% last_seq=%lu last_rssi=%.1f last_snr=%.1f last_error=%d\n",
                    rxOkCount, emptyPacketCount, rxErrorCount, seqParseErrorCount,
                    telemetryParseOkCount, telemetryParseErrorCount, lostPacketCount,
                    lossPercent, lastSeq, lastRssi, lastSnr, lastRadioLibError);
    } else {
      Serial.printf("RX status: ok=%lu empty=%lu errors=%lu seq_errors=%lu telem_ok=%lu telem_errors=%lu lost=%lu loss=%.1f%% last_seq=%lu last_rssi=n/a last_snr=n/a last_error=%d\n",
                    rxOkCount, emptyPacketCount, rxErrorCount, seqParseErrorCount,
                    telemetryParseOkCount, telemetryParseErrorCount, lostPacketCount,
                    lossPercent, lastSeq, lastRadioLibError);
    }
    last_status_print = now;
  }

  if (!lora_initialized) {
    delay(100);
    return;
  }

  if (!receivedFlag) {
    delay(10);
    return;
  }

  receivedFlag = false;

  String payload;
  int state = lora->readData(payload);

  if (state == RADIOLIB_ERR_NONE) {
    float rssi = lora->getRSSI();
    float snr = lora->getSNR();
    float freq_error = lora->getFrequencyError();

    if (payload.length() > 0) {
      rxOkCount++;
      lastRssi = rssi;
      lastSnr = snr;
      hasLastSignal = true;

      TomosTelemetry telemetry;
      String telemetryError;
      bool telemetryValid = parseTomosTelemetry(payload, telemetry, telemetryError);
      if (telemetryValid) {
        lastTelemetry = telemetry;
        telemetryParseOkCount++;
      } else {
        telemetryParseErrorCount++;
        Serial.printf("TELEM PARSE ERROR: %s\n", telemetryError.c_str());
      }

      uint32_t seq = telemetry.seq;
      bool seqValid = telemetryValid || parseSeqFromPayload(payload, seq);
      if (seqValid) {
        if (!hasLastSeq) {
          lastSeq = seq;
          hasLastSeq = true;
        } else if (seq > lastSeq) {
          uint32_t sequenceGap = seq - lastSeq;
          if (sequenceGap > 1) {
            lostPacketCount += sequenceGap - 1;
          }
          lastSeq = seq;
        } else {
          Serial.printf("WARNING: duplicate/out-of-order seq=%lu last_seq=%lu\n",
                        seq, lastSeq);
        }
      } else {
        seqParseErrorCount++;
        Serial.println("WARNING: invalid or missing seq field");
      }

      float lossPercent = calculateLossPercent();

      Serial.printf("\n>>> PACKET RECEIVED #%lu <<<\n", rxOkCount);
      Serial.printf("LEN: %u\n", static_cast<unsigned int>(payload.length()));
      Serial.printf("Payload: %s\n", payload.c_str());
      Serial.print("HEX:");
      for (size_t i = 0; i < payload.length(); i++) {
        Serial.printf(" %02X", static_cast<uint8_t>(payload[i]));
      }
      Serial.println();
      Serial.printf("RSSI: %.2f dBm\n", rssi);
      Serial.printf("SNR: %.2f dB\n", snr);
      Serial.printf("Frequency Error: %.2f Hz\n", freq_error);
      if (telemetryValid) {
        Serial.printf("TOMOS DATA: fix=%d lat=%.6f lon=%.6f spd=%.1f max=%.1f sat=%d hdop=%.1f vbat=%.2f time=%s\n",
                      telemetry.fix ? 1 : 0, telemetry.latitude, telemetry.longitude,
                      telemetry.speedKmh, telemetry.maxSpeedKmh, telemetry.satellites,
                      telemetry.hdop, telemetry.batteryVoltage, telemetry.time);
        Serial.printf("{\"type\":\"telemetry\",\"time\":\"%s\",\"seq\":%lu,\"fix\":%s,\"lat\":%.6f,\"lon\":%.6f,\"spd\":%.1f,\"max\":%.1f,\"sat\":%d,\"hdop\":%.1f,\"vbat\":%.2f,\"rssi\":%.1f,\"snr\":%.1f,\"lost\":%lu,\"loss\":%.1f,\"rx_ok\":%lu}\n",
                      telemetry.time, telemetry.seq, telemetry.fix ? "true" : "false",
                      telemetry.latitude, telemetry.longitude, telemetry.speedKmh,
                      telemetry.maxSpeedKmh, telemetry.satellites, telemetry.hdop,
                      telemetry.batteryVoltage, rssi, snr, lostPacketCount,
                      lossPercent, rxOkCount);
      }
      Serial.printf("LINK: rx_ok=%lu lost=%lu loss=%.1f%% last_seq=%lu seq_errors=%lu\n",
                    rxOkCount, lostPacketCount, lossPercent, lastSeq,
                    seqParseErrorCount);
      Serial.println("---\n");
    } else {
      emptyPacketCount++;
      Serial.printf("EMPTY PACKET EVENT (count=%lu)\n", emptyPacketCount);
      Serial.printf("Debug RSSI: %.2f dBm, SNR: %.2f dB\n", rssi, snr);
    }
  } else {
    rxErrorCount++;
    lastRadioLibError = state;
    Serial.printf("Receive error - Error code: %d\n", state);
  }

  state = lora->startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    rxErrorCount++;
    lastRadioLibError = state;
    Serial.printf("Failed to restart receive mode - Error code: %d\n", state);
  }
}
