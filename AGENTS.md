# AGENTS.md — Tomos Pit Receiver

## Projectdoel

Dit project ontwikkelt firmware voor de pit-side LoRa ontvanger van het Tomos Tracker project.

De ontvanger draait op een Heltec WiFi LoRa 32 V4 en moet point-to-point LoRa telemetry packets ontvangen van de Tomos Tracker.

De Tomos Tracker zendt GNSS/telemetry data zoals:

* packet sequence
* GPS fix status
* snelheid
* max snelheid
* satellieten
* HDOP
* batterijspanning
* lokale tijd

De pit receiver moet in deze fase vooral bewijzen dat de LoRa-link werkt:

* packet ontvangen
* payload tekst correct tonen
* RSSI tonen
* SNR tonen
* packet counters tonen
* later eventueel OLED/dashboard/pitsoftware-koppeling

Belangrijkste filosofie:

* eerst betrouwbare ontvangst bewijzen
* daarna pas OLED/dashboard toevoegen
* geen LoRaWAN
* geen WiFi/BLE in deze fase
* geen rondeberekening in firmware
* geen complexe pitsoftware in deze firmware

## Hardware

Board:

* Heltec WiFi LoRa 32 V4
* ESP32-S3 / ESP32-S3R2 familie
* SX1262 LoRa transceiver
* onboard OLED display
* USB-C
* 868 MHz LoRa variant voor EU/België

De officiële Heltec-pagina beschrijft de WiFi LoRa 32 V4 als een opvolger van de V3 met ESP32-S3 MCU, SX1262 LoRa chip, ingebouwde Wi-Fi/Bluetooth antenne, OLED-display en ondersteuning voor Arduino, PlatformIO, ESP-IDF en MicroPython.

De Heltec wiki vermeldt dat de V4 gebaseerd is op ESP32-S3R2 + SX1262, met Wi-Fi b/g/n, BLE, LoRa, 2 MB PSRAM en 16 MB external flash.

## PlatformIO

Als er geen betrouwbaar exact PlatformIO boardprofiel voor V4 beschikbaar is, gebruiken we voorlopig:

```ini
board = esp32-s3-devkitc-1
framework = arduino
```

Gebruik native USB CDC flags voor Serial Monitor op ESP32-S3:

```ini
build_flags =
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
```

Basisconfiguratie:

```ini
[env:heltec_wifi_lora32_v4]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino

monitor_speed = 115200
upload_speed = 921600

build_flags =
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1

lib_deps =
    jgromes/RadioLib
```

Pin de RadioLib-versie als stabiliteit belangrijk wordt. Het project werd initieel gebouwd met RadioLib 7.7.1.

## LoRa hardware

LoRa chip:

* SX1262

Gebruik deze pinnen voor de Heltec WiFi LoRa 32 V4 / V3-compatible SX1262 setup:

```cpp
#pragma once

#define PIN_LORA_NSS   8
#define PIN_LORA_SCK   9
#define PIN_LORA_MOSI  10
#define PIN_LORA_MISO  11
#define PIN_LORA_RST   12
#define PIN_LORA_BUSY  13
#define PIN_LORA_DIO1  14
```

Deze pinnen komen overeen met publiek gedeelde Heltec V4/V3 SX1262 pinconfiguraties:

* SCK = GPIO9
* MISO = GPIO11
* MOSI = GPIO10
* NSS = GPIO8
* DIO1 = GPIO14
* RST = GPIO12
* BUSY = GPIO13

Deze pinmap wordt ook genoemd in RadioLib-discussies en communityinformatie rond Heltec V4/V3.

## Belangrijke V4 RF-aandachtspunten

De Heltec WiFi LoRa 32 V4 heeft rond de SX1262 mogelijk extra RF-front-end control pinnen. In communitydocumentatie worden o.a. genoemd:

```cpp
FEM_EN = GPIO2
FEM_PA = GPIO46
```

Daarbij wordt `FEM_EN` beschreven als frontend enable die HIGH moet zijn.

Gebruik deze pinnen niet blind in elke wijziging, maar hou ze in gedachten als:

* LoRa init wel werkt maar echte ontvangst slecht is
* TX/RX vreemd gedrag vertoont
* RSSI/SNR onrealistisch blijven
* ontvangst alleen vlakbij werkt
* RadioLib OK lijkt maar payloads niet betrouwbaar binnenkomen

Voeg frontend-control alleen toe in een expliciete debuggingstap, niet tijdens algemene refactors.

## LoRa parameters

De pit receiver moet exact dezelfde LoRa-parameters gebruiken als de Tomos Tracker zender:

```cpp
frequency = 868.0 MHz
bandwidth = 125.0 kHz
spreadingFactor = 7
codingRate = 5
syncWord = 0x12
```

Gebruik point-to-point LoRa.

Niet gebruiken:

* LoRaWAN
* OTAA/ABP
* TTN
* ACK-protocol
* mesh protocol
* WiFi
* BLE

## Antenne

Voor elke LoRa TX-test moet een correcte 868 MHz LoRa-antenne aangesloten zijn.

Niet zenden zonder LoRa-antenne.

Voor RX is het minder gevaarlijk, maar zonder goede antenne zijn RSSI/SNR-metingen niet representatief.

## Bekende bug uit eerste receiver-versie

De eerste receiver-firmware toonde honderden “ontvangen packets” met lege payload:

```text
Payload:
RSSI: -6.50 dBm
SNR: 29.25 dB
Frequency Error: 0.00 Hz
```

Dit waren geen echte LoRa-pakketten.

Oorzaak:

```cpp
int state = lora->readData((uint8_t*)NULL, 0);
```

Dit mag niet gebruikt worden als “is er een nieuw packet beschikbaar?”-test.

Met lengte nul kan RadioLib succesvol terugkeren zonder gegevens te lezen. Daardoor werd elke loop onterecht als ontvangen packet geteld.

Daarna werd opnieuw `readData(payload)` aangeroepen, maar het vermeende packet was dan al afgehandeld of er was nooit echt payload beschikbaar. Resultaat: lege payloads en valse counters.

## Correcte RadioLib receive-aanpak

Gebruik bij voorkeur interrupt-driven receive met DIO1:

```cpp
volatile bool receivedFlag = false;

void setFlag() {
    receivedFlag = true;
}
```

Tijdens setup:

```cpp
radio.setPacketReceivedAction(setFlag);
radio.startReceive();
```

In loop:

```cpp
if (receivedFlag) {
    receivedFlag = false;

    String payload;
    int state = radio.readData(payload);

    if (state == RADIOLIB_ERR_NONE && payload.length() > 0) {
        // geldig packet
    } else if (state == RADIOLIB_ERR_NONE && payload.length() == 0) {
        // empty event, niet tellen als echt packet
    } else {
        // error counter verhogen
    }

    radio.startReceive();
}
```

Belangrijk:

* Roep `readData(payload)` precies één keer per echte packet-event aan.
* Tel alleen als geldig packet wanneer:

  * `state == RADIOLIB_ERR_NONE`
  * `payload.length() > 0`
* Controleer het resultaat van `startReceive()`.
* Gebruik RSSI/SNR alleen na een echt ontvangen packet.
* Beschouw constante RSSI/SNR bij lege payloads als verdacht.
* Tel lege payload-events apart als `emptyPacketCount`.

## Verwachte echte receiver-output

Bij succesvolle ontvangst moet de output lijken op:

```text
>>> PACKET RECEIVED #1 <<<
LEN: 84
Payload: TOMOS,seq=123,fix=1,spd=18.4,max=31.2,sat=11,hdop=1.4,vbat=3.88,time=15:04:22
HEX: 54 4F 4D 4F 53 2C ...
RSSI: -35.00 dBm
SNR: 12.50 dB
Frequency Error: ...
---
```

Een lege payload zoals hieronder is geen geldig telemetry packet:

```text
Payload:
RSSI: -6.50 dBm
SNR: 29.25 dB
```

## Counters

Hou minstens deze counters bij:

```cpp
uint32_t rxOkCount;
uint32_t emptyPacketCount;
uint32_t rxErrorCount;
int lastRadioLibError;
```

Statusregel elke 5 seconden:

```text
RX status: ok=0 empty=0 errors=0 last_error=0 waiting...
```

## Teststrategie

Eerste test:

1. Sluit 868 MHz LoRa-antennes aan op beide boards.
2. Start Tomos Tracker zender.
3. Start pit receiver.
4. Plaats beide boards eerst 1–3 meter uit elkaar.
5. Open Serial Monitor van receiver.
6. Verwacht ongeveer 1 packet per seconde.
7. Payload mag niet leeg zijn.
8. RSSI/SNR moeten realistisch zijn.

Daarna afstandstest:

1. Vergroot afstand geleidelijk.
2. Noteer RSSI en SNR.
3. Test door muren heen.
4. Test buiten.
5. Later test op Tomos met trillingen en ontstekingsruis.

## Werkregels voor Codex

Bij elke taak:

1. Maak kleine, controleerbare wijzigingen.
2. Wijzig alleen de bestanden die expliciet gevraagd worden.
3. Voeg geen OLED-display toe tenzij expliciet gevraagd.
4. Voeg geen WiFi/BLE toe.
5. Voeg geen LoRaWAN toe.
6. Voeg geen rondeberekening toe.
7. Gebruik point-to-point LoRa.
8. Verander LoRa-parameters niet zonder expliciete opdracht.
9. Verander pinnen niet zonder expliciete opdracht en motivatie.
10. Tel lege payloads niet als geldige packets.
11. Controleer altijd RadioLib return codes.
12. Laat firmware blijven draaien bij init- of receivefouten.
13. Geef na afloop kort:

    * welke bestanden gewijzigd zijn
    * wat de wijziging doet
    * of PlatformIO Build slaagt
    * hoe te testen

## Testcommando’s

Build:

```powershell
platformio run -e heltec_wifi_lora32_v4
```

Of wanneer `pio` niet in PATH staat:

```powershell
python -m platformio run -e heltec_wifi_lora32_v4
```

Upload:

```powershell
platformio run -e heltec_wifi_lora32_v4 --target upload --upload-port COMx
```

Monitor:

```powershell
platformio device monitor -e heltec_wifi_lora32_v4 --monitor-port COMx
```

Als PlatformIO via lokale `.venv` werd aangemaakt, commit `.venv/` niet.

Gebruik `.gitignore` voor:

```gitignore
.pio/
.vscode/
.venv/
venv/
env/
```

## Huidige ontwikkelfase

We zitten in de LoRa receiver bring-up fase.

Reeds gedaan:

* Nieuw PlatformIO receiver-project aangemaakt.
* RadioLib toegevoegd.
* SX1262 init werkt.
* Eerste receiver-loop had valse empty packet events.
* Analyse wijst op verkeerd gebruik van `readData((uint8_t*)NULL, 0)`.

Volgende stap:

* Receiver herschrijven naar interrupt-driven DIO1 receive met `setPacketReceivedAction()`.
* Alleen echte payloads tellen.
* Payload lengte, tekst, hex, RSSI en SNR printen.
* Daarna end-to-end test:
  `Tomos Tracker telemetry → LoRa TX → pit receiver → Serial payload`.
