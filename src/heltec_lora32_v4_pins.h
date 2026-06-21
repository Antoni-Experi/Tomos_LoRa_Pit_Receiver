#ifndef HELTEC_LORA32_V4_PINS_H
#define HELTEC_LORA32_V4_PINS_H

// Heltec WiFi LoRa 32 V4 - SX1262 LoRa Chip Pin Configuration
// MCU: ESP32-S3

// SPI Pins (for LoRa)
#define LORA_SCK   9
#define LORA_MISO  11
#define LORA_MOSI  10
#define LORA_NSS   8   // Chip Select

// LoRa Control Pins
#define LORA_RST   12  // Reset
#define LORA_BUSY  13  // Busy
#define LORA_DIO1  14  // Interrupt (DIO1)

#endif // HELTEC_LORA32_V4_PINS_H
