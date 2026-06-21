#include <Arduino.h>
#include <RadioLib.h>
#include "heltec_lora32_v4_pins.h"

// LoRa frequency in MHz
const float LORA_FREQ = 868.0;

// LoRa parameters matching Tomos Tracker
const float LORA_BANDWIDTH = 125.0;       // 125 kHz
const uint8_t LORA_SPREADING_FACTOR = 7;  // SF7
const uint8_t LORA_CODING_RATE = 5;       // CR4/5
const uint8_t LORA_SYNC_WORD = 0x12;      // Custom sync word
const int16_t LORA_POWER = 14;             // dBm

// RadioLib objects
SPIClass spi;
Module* radio_module = nullptr;
SX1262* lora = nullptr;

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

void setFlag() {
  receivedFlag = true;
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

  if (now - last_status_print >= STATUS_INTERVAL) {
    uint32_t expectedPackets = rxOkCount + lostPacketCount;
    float lossPercent = 0.0f;
    if (expectedPackets > 0) {
      lossPercent = 100.0f * lostPacketCount / expectedPackets;
    }

    if (hasLastSignal) {
      Serial.printf("RX status: ok=%lu empty=%lu errors=%lu seq_errors=%lu lost=%lu loss=%.1f%% last_seq=%lu last_rssi=%.1f last_snr=%.1f last_error=%d\n",
                    rxOkCount, emptyPacketCount, rxErrorCount, seqParseErrorCount,
                    lostPacketCount, lossPercent, lastSeq, lastRssi, lastSnr,
                    lastRadioLibError);
    } else {
      Serial.printf("RX status: ok=%lu empty=%lu errors=%lu seq_errors=%lu lost=%lu loss=%.1f%% last_seq=%lu last_rssi=n/a last_snr=n/a last_error=%d\n",
                    rxOkCount, emptyPacketCount, rxErrorCount, seqParseErrorCount,
                    lostPacketCount, lossPercent, lastSeq, lastRadioLibError);
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

      uint32_t seq = 0;
      if (parseSeqFromPayload(payload, seq)) {
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

      uint32_t expectedPackets = rxOkCount + lostPacketCount;
      float lossPercent = 0.0f;
      if (expectedPackets > 0) {
        lossPercent = 100.0f * lostPacketCount / expectedPackets;
      }

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
