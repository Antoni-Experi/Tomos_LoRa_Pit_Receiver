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

void setFlag() {
  receivedFlag = true;
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
    Serial.printf("RX status: %s, ok=%lu empty=%lu errors=%lu last_error=%d\n",
                  lora_initialized ? "waiting" : "radio unavailable",
                  rxOkCount, emptyPacketCount, rxErrorCount, lastRadioLibError);
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
