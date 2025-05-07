// --- Dog Unit Code (Heltec Wireless Tracker V1.1) ---
// Includes GPS, LoRa TX/RX, Display, Buzzer (based on Owner Threshold),
// Critical Sections, and Long-Press Deep Sleep on Button 0

#include <Arduino.h>
#include "LoRaWan_APP.h"    // LoRa library
#include "HT_st7735.h"      // Display library for TFT
#include "HT_TinyGPS++.h"   // GPS library
#include "freertos/FreeRTOS.h" // Required for portMUX_TYPE and critical sections
#include "freertos/task.h"
#include "esp_sleep.h"      // Include for deep sleep functions

// --- Pin Definitions ---
#define LED_PIN 18      // Onboard LED for visual feedback
#define VEXT_PIN 3      // Need Vext for Display & GPS
#define BUZZER_PIN 45   // Buzzer Pin
#define GPS_RX_PIN 33   // ESP32 RX from GPS TX
#define GPS_TX_PIN 34   // ESP32 TX to GPS RX
#define BUTTON_PIN 0    // Onboard USER/PRG Button for sleep trigger

// --- LoRa Configuration (MUST MATCH OWNER UNIT) ---
#define RF_FREQUENCY          915000000 // Hz (US)
#define TX_OUTPUT_POWER       14        // dBm
#define LORA_BANDWIDTH        0         // 125 kHz
#define LORA_SPREADING_FACTOR 7         // SF7
#define LORA_CODINGRATE       1         // 4/5
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT   0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON  false
#define TX_TIMEOUT_VALUE      3000      // ms
#define RX_TIMEOUT_VALUE      1500      // RX Timeout for receiving pings

// --- Buzzer Pattern State Machine ---
bool isBuzzingActive = false;
int buzzerPhase = 0; // 0=Idle
int buzzerPulseCount = 0; // Pulses completed in current group
unsigned long lastBuzzChangeTime = 0;
// Durations for pattern
const unsigned long shortBuzzDuration = 500;
const unsigned long shortPauseDuration = 50;
const unsigned long longPauseDuration = 1500; // Pause between groups
const unsigned long longBuzzDuration = 500; // Duration of the second group's buzzes
const unsigned long longBetweenBuzzPause = 500; // Pause between second group's buzzes

// --- Application ---
#define GPS_PREAMBLE "12345-96458"       // Preamble for GPS data packets
#define GPS_TRANSMIT_INTERVAL 15000     // Send GPS data every 15 seconds
#define OWNER_PING_MESSAGE "PING"       // Prefix for expected PING message from Owner

// --- Button Debouncing & Long Press ---
int buttonState_dog;
int lastButtonState_dog = HIGH;
unsigned long lastDebounceTime_dog = 0;
const long debounceDelay_dog = 50;
unsigned long buttonPressStartTime_dog = 0;
bool buttonHeld_dog = false;
const long longPressDuration_dog = 3000; // 3 seconds for long press trigger

// --- Display Object ---
// Use extern as required by build environment conflict
HT_st7735 tft;

// --- GPS Object ---
TinyGPSPlus gps;

// --- RTOS Synchronization ---
static portMUX_TYPE gpsDataMutex = portMUX_INITIALIZER_UNLOCKED;

// --- LoRa State Machine ---
typedef enum { LOWPOWER, STATE_RX, STATE_TX } States_t;
States_t State = LOWPOWER;

// --- Global Variables ---
static RadioEvents_t RadioEvents;
char txpacket[64];
char rxPacket[30];
volatile bool txDone = false; volatile bool rxDone = false;
volatile bool txTimeout = false; volatile bool rxTimeout = false;
volatile bool loraInterruptFired = false;
uint32_t lastTxTime = 0;
int16_t receivedRssi = 0; uint16_t receivedSize = 0;
String displayLine1 = "Dog Unit Init...";
String displayLine2 = ""; String displayLine3 = ""; String displayLine4 = "";
uint32_t packetCounter = 0;
// GPS data copies - protected by mutex
double gpsLat_safe = 0.0; double gpsLon_safe = 0.0;
bool gpsFix = false; uint32_t gpsSatellites = 0;
// Threshold received from owner unit
int ownerRssiThreshold = -100; // Default threshold

// --- Callbacks ---
void OnTxDone( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnTxTimeout( void );
void OnRxTimeout( void );

// --- Helper Function Prototypes ---
void VextON(); void VextOFF();
void setupDisplay(); void setupGPS(); void setupLoRa();
void checkGPS(); void updateDisplay();
void sendGPSData(); void startReceive();
// void triggerBuzzer(bool active); // REMOVED - Replaced by manageBuzzerPattern
void manageBuzzerPattern(); // ADDED
void handleReceivedPing(uint8_t *payload, uint16_t size, int16_t rssi);
void checkButton_Dog(); // Added prototype

// --- Setup ---
void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting Dog Unit (Full Code w/ Sleep)...");

    pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer starts LOW
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Setup button pin (GPIO 0)
    VextON(); // Turn on power for Peripherals

    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    setupDisplay(); // Uses extern 'display'
    setupGPS();
    setupLoRa();

    displayLine1 = "Dog Unit Ready";
    displayLine2 = "Waiting for Fix...";
    updateDisplay();

    startReceive();
    lastTxTime = millis();
    lastButtonState_dog = digitalRead(BUTTON_PIN); // Read initial button state
    Serial.println("Setup Complete. Entering Loop.");
}

// --- Loop ---
void loop() {
    Radio.IrqProcess(); // Process LoRa interrupts FIRST
    checkGPS();         // Continuously check GPS data
    manageBuzzerPattern(); // Manage non-blocking buzzer sequence
    checkButton_Dog();  // Check button for long press sleep

    bool handledInterrupt = false;
    if (loraInterruptFired) { /* ... Handle LoRa flags as before ... */
        loraInterruptFired = false; handledInterrupt = true;
        if (txDone) { txDone = false; Serial.println("GPS TX Done Flag Processed."); digitalWrite(LED_PIN, LOW); State = LOWPOWER; startReceive();
        } else if (rxDone) { rxDone = false; Serial.println("Ping RX Done Flag Processed."); handleReceivedPing((uint8_t*)rxPacket, receivedSize, receivedRssi); State = LOWPOWER; startReceive();
        } else if (txTimeout) { txTimeout = false; Serial.println("GPS TX Timeout Flag Processed."); digitalWrite(LED_PIN, LOW); Radio.Sleep(); State = LOWPOWER; startReceive();
        } else if (rxTimeout) { rxTimeout = false; Serial.println("Ping RX Timeout Flag Processed."); State = LOWPOWER; startReceive();
        } else { if (State != STATE_TX) startReceive(); }
    }

    // Check if time to send GPS data
    if (!handledInterrupt && State != STATE_TX) { /* ... Send GPS if interval passed and fix OK ... */
        if (millis() - lastTxTime > GPS_TRANSMIT_INTERVAL) {
             bool currentFixStatus; portENTER_CRITICAL(&gpsDataMutex); currentFixStatus = gpsFix; portEXIT_CRITICAL(&gpsDataMutex);
             if (currentFixStatus) { State = STATE_TX; sendGPSData(); lastTxTime = millis(); }
             else { Serial.println("No GPS fix, skipping transmission."); lastTxTime = millis(); startReceive(); }
        } else if (State == LOWPOWER) { startReceive(); }
    }

    // Update display periodically
    static uint32_t lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 1000) { updateDisplay(); lastDisplayUpdate = millis(); }
    delay(5);
}

// --- LoRa Callback Implementations ---
void OnTxDone( void ) { txDone = true; loraInterruptFired = true; }
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ) {
    receivedSize = (size < sizeof(rxPacket) - 1) ? size : sizeof(rxPacket) - 1;
    memcpy(rxPacket, payload, receivedSize); rxPacket[receivedSize] = '\0';
    receivedRssi = rssi;
    rxDone = true; loraInterruptFired = true;
}
void OnTxTimeout( void ) { txTimeout = true; loraInterruptFired = true; }
void OnRxTimeout( void ) { rxTimeout = true; loraInterruptFired = true; }

// --- Helper Function Implementations ---
void VextON() { pinMode(VEXT_PIN, OUTPUT); digitalWrite(VEXT_PIN, LOW); Serial.println("Vext Power ON"); delay(100); }
void VextOFF() { pinMode(VEXT_PIN, OUTPUT); digitalWrite(VEXT_PIN, HIGH); Serial.println("Vext Power OFF"); }

void setupDisplay() { /* ... as before, using extern display ... */
    tft.st7735_init(); updateDisplay(); Serial.println("Display Initialized");
}
void setupGPS() { /* ... as before ... */
     Serial1.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); Serial.println("GPS Serial Initialized (Serial1 @ 115200, RX=33, TX=34)");
}
void setupLoRa() { /* ... as before ... */
    RadioEvents.TxDone = OnTxDone; RadioEvents.RxDone = OnRxDone; RadioEvents.TxTimeout = OnTxTimeout; RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init( &RadioEvents ); Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE );
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    Serial.println("LoRa Radio Initialized (TX/RX)"); Serial.printf(" Freq:%.3fMHz | Power:%ddBm | SF:%d | BW:%dkHz\n", RF_FREQUENCY/1e6, TX_OUTPUT_POWER, LORA_SPREADING_FACTOR, (125 * (1 << LORA_BANDWIDTH)));
}
void checkGPS() { /* ... as before, with mutex ... */
    bool fix_before; portENTER_CRITICAL(&gpsDataMutex); fix_before = gpsFix; portEXIT_CRITICAL(&gpsDataMutex);
    while (Serial1.available() > 0) { gps.encode(Serial1.read()); }
    bool currentFix = gps.location.isValid(); uint32_t currentSats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    bool locationUpdated = gps.location.isUpdated();
    portENTER_CRITICAL(&gpsDataMutex);
    gpsFix = currentFix; gpsSatellites = currentSats;
    if (gpsFix && locationUpdated) { gpsLat_safe = gps.location.lat(); gpsLon_safe = gps.location.lng(); }
    portEXIT_CRITICAL(&gpsDataMutex);
    if (fix_before != currentFix) { Serial.printf(">>> GPS Fix Status Changed: %s <<<\n", currentFix ? "Acquired" : "Lost"); }
}
void updateDisplay() { /* ... as before, shows Lat/Lon, RSSI, OwnerThreshold ... */
     portENTER_CRITICAL(&gpsDataMutex); bool currentFix = gpsFix; uint32_t currentSats = gpsSatellites; double currentLat = gpsLat_safe; double currentLon = gpsLon_safe; portEXIT_CRITICAL(&gpsDataMutex);
     int currentOwnerThreshold = ownerRssiThreshold; int16_t currentRssi = receivedRssi;
     String fixStatus = currentFix ? "YES" : "NO ";
     tft.st7735_fill_screen(ST7735_BLACK);
     tft.st7735_write_str(0, 0, displayLine1.c_str(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
     tft.st7735_write_str(0, 12, ("GPS:" + fixStatus + " Sats:" + String(currentSats)).c_str(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
     if(currentFix){ tft.st7735_write_str(0, 24, ("Lat:" + String(currentLat, 4)).c_str(), Font_7x10, ST7735_WHITE, ST7735_BLACK); tft.st7735_write_str(0, 36, ("Lon:" + String(currentLon, 4)).c_str(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
     } else { tft.st7735_write_str(0, 24, "Waiting for Coords", Font_7x10, ST7735_WHITE, ST7735_BLACK); tft.st7735_write_str(0, 36, "", Font_7x10, ST7735_WHITE, ST7735_BLACK); }
     tft.st7735_write_str(0, 48, ("R:" + String(currentRssi) + " T:" + String(currentOwnerThreshold)).c_str(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
}
void sendGPSData() { /* ... as before, with mutex ... */
    if(State != STATE_TX) { Serial.println("Error: sendGPSData called when not in TX state."); State = LOWPOWER; startReceive(); return; }
    portENTER_CRITICAL(&gpsDataMutex); double latToSend = gpsLat_safe; double lonToSend = gpsLon_safe; portEXIT_CRITICAL(&gpsDataMutex);
    snprintf(txpacket, sizeof(txpacket), "%s,%.6f,%.6f", GPS_PREAMBLE, latToSend, lonToSend); packetCounter++;
    Serial.println(">>> TRANSMITTING GPS PACKET NOW <<<"); Serial.printf("Sending packet #%lu: %s\n", packetCounter, txpacket);
    displayLine1 = "TX GPS #" + String(packetCounter);
    digitalWrite(LED_PIN, HIGH); Radio.Send((uint8_t *)txpacket, strlen(txpacket));
}
void startReceive() { /* ... as before ... */
    if (State == STATE_TX) return;
    if (State != STATE_RX) { State = STATE_RX; displayLine1 = "Listening..."; Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true); Radio.Rx(RX_TIMEOUT_VALUE); }
}
void handleReceivedPing(uint8_t *payload, uint16_t size, int16_t rssi) { /* ... as before, parses threshold, checks against ownerRssiThreshold, calls triggerBuzzer ... */
    int receivedThreshold = ownerRssiThreshold; bool thresholdUpdated = false;
    const char* pingPrefix = OWNER_PING_MESSAGE ",";
    if (size > strlen(pingPrefix) && strncmp((char*)payload, pingPrefix, strlen(pingPrefix)) == 0) {
        Serial.println("Received PING message from Owner."); char* thresholdStr = (char*)payload + strlen(pingPrefix); char* endPtr; long val = strtol(thresholdStr, &endPtr, 10);
        if (endPtr != thresholdStr && *endPtr == '\0' && val >= -240 && val <= -40) { receivedThreshold = (int)val; if (ownerRssiThreshold != receivedThreshold) { ownerRssiThreshold = receivedThreshold; thresholdUpdated = true; Serial.printf("Updated Owner Threshold to: %d dBm\n", ownerRssiThreshold); }
        } else { Serial.printf("Could not parse threshold from PING payload: %s\n", thresholdStr); }
    } else { Serial.printf("Received non-PING or short message: %s\n", (char*)payload); }
    displayLine1 = "PING Received"; displayLine4 = "R:" + String(rssi) + " T:" + String(ownerRssiThreshold);
    if (thresholdUpdated) { displayLine3 = "Thrsh Set:" + String(ownerRssiThreshold); } else { displayLine3 = ""; } // Keep previous line 3 (Lon) if no update
    if (rssi <= ownerRssiThreshold) { // Use <=
            // --- START BUZZER SEQUENCE (Non-Blocking) ---
            if (!isBuzzingActive) { // Start sequence only if not already running
                Serial.printf("RSSI %d <= Threshold %d! Starting Buzzer Sequence!\n", rssi, ownerRssiThreshold);
                isBuzzingActive = true;
                buzzerPhase = 1; // Start with phase 1
                buzzerPulseCount = 0;
                lastBuzzChangeTime = millis();
                digitalWrite(BUZZER_PIN, HIGH); // Turn buzzer ON for the first pulse
                // Serial.println("Buzzer Sequence Phase 1: ON"); // Optional debug
            } else {
                 // Sequence already running, don't restart, maybe log it
                 // Serial.printf("RSSI %d <= Threshold %d but buzzer sequence already active.\n", rssi, rssiThreshold);
            }
            // --- REMOVED old triggerBuzzer/delay calls ---
        } else {
             // If RSSI is OK now, should we stop an ongoing sequence? Optional.
             // if (isBuzzingActive) {
             //    isBuzzingActive = false; buzzerPhase = 0; digitalWrite(BUZZER_PIN, LOW);
             //    Serial.println("RSSI OK, stopping buzzer sequence.");
             // }
        }
}
// New non-blocking buzzer pattern function
void manageBuzzerPattern() {
    if (!isBuzzingActive) {
        digitalWrite(BUZZER_PIN, LOW); // Ensure off when idle
        return;
    }

    unsigned long currentTime = millis();

    switch (buzzerPhase) {
        // --- First Group: 3 x (50ms ON, 50ms OFF) ---
        case 1: // Buzz 1 ON
            if (currentTime - lastBuzzChangeTime >= shortBuzzDuration) {
                digitalWrite(BUZZER_PIN, LOW); buzzerPulseCount = 1; buzzerPhase = 2; lastBuzzChangeTime = currentTime;
            } break;
        case 2: // Pause 1 OFF
            if (currentTime - lastBuzzChangeTime >= shortPauseDuration) {
                 digitalWrite(BUZZER_PIN, HIGH); buzzerPhase = 3; lastBuzzChangeTime = currentTime;
            } break;
        case 3: // Buzz 2 ON
             if (currentTime - lastBuzzChangeTime >= shortBuzzDuration) {
                digitalWrite(BUZZER_PIN, LOW); buzzerPulseCount = 2; buzzerPhase = 4; lastBuzzChangeTime = currentTime;
            } break;
        case 4: // Pause 2 OFF
            if (currentTime - lastBuzzChangeTime >= shortPauseDuration) {
                 digitalWrite(BUZZER_PIN, HIGH); buzzerPhase = 5; lastBuzzChangeTime = currentTime;
            } break;
        case 5: // Buzz 3 ON
             if (currentTime - lastBuzzChangeTime >= shortBuzzDuration) {
                digitalWrite(BUZZER_PIN, LOW); buzzerPulseCount = 3; buzzerPhase = 7; lastBuzzChangeTime = currentTime;
                Serial.println("Short Buzz Group Done. Starting Long Pause.");
            } break;

        // --- Long Pause ---
        case 7: // Long Pause OFF (1.5 seconds)
            if (currentTime - lastBuzzChangeTime >= longPauseDuration) {
                buzzerPulseCount = 0; digitalWrite(BUZZER_PIN, HIGH); buzzerPhase = 8; lastBuzzChangeTime = currentTime;
                Serial.println("Long Pause Done. Starting Long Buzz Group.");
            } break;

        // --- Second Group: 3 x (50ms ON, 500ms OFF) ---
        case 8: // Buzz 4 ON (Using shortBuzzDuration for ON time)
            if (currentTime - lastBuzzChangeTime >= shortBuzzDuration) { // Short ON duration
                digitalWrite(BUZZER_PIN, LOW); buzzerPulseCount = 1; buzzerPhase = 9; lastBuzzChangeTime = currentTime;
            } break;
        case 9: // Long Pause 1 OFF (500ms)
            if (currentTime - lastBuzzChangeTime >= longBetweenBuzzPause) { // Long OFF duration
                 digitalWrite(BUZZER_PIN, HIGH); buzzerPhase = 10; lastBuzzChangeTime = currentTime;
            } break;
        case 10: // Buzz 5 ON
             if (currentTime - lastBuzzChangeTime >= shortBuzzDuration) {
                digitalWrite(BUZZER_PIN, LOW); buzzerPulseCount = 2; buzzerPhase = 11; lastBuzzChangeTime = currentTime;
            } break;
        case 11: // Long Pause 2 OFF
             if (currentTime - lastBuzzChangeTime >= longBetweenBuzzPause) {
                 digitalWrite(BUZZER_PIN, HIGH); buzzerPhase = 12; lastBuzzChangeTime = currentTime;
            } break;
         case 12: // Buzz 6 ON
             if (currentTime - lastBuzzChangeTime >= shortBuzzDuration) {
                digitalWrite(BUZZER_PIN, LOW); buzzerPulseCount = 3; buzzerPhase = 0; isBuzzingActive = false; // Sequence complete -> Idle
                lastBuzzChangeTime = currentTime; Serial.println("Long Buzz Group Done. Sequence Complete.");
            } break;

        default: // Error state, reset
             Serial.println("Buzzer Sequence Error: Unknown Phase -> Idle");
             digitalWrite(BUZZER_PIN, LOW); isBuzzingActive = false; buzzerPhase = 0;
             break;
    }
}

// Handles button press for Sleep trigger
void checkButton_Dog() {
    int reading = digitalRead(BUTTON_PIN);
    // Debounce
    if (reading != lastButtonState_dog) { lastDebounceTime_dog = millis(); }
    if ((millis() - lastDebounceTime_dog) > debounceDelay_dog) {
        if (reading != buttonState_dog) { // State has officially changed
            buttonState_dog = reading;
            if (buttonState_dog == LOW) { // Button Newly Pressed Down
                Serial.println("Dog Unit Button Pressed Down!");
                buttonPressStartTime_dog = millis(); buttonHeld_dog = true;
            } else { // Button Newly Released (HIGH)
                 Serial.println("Dog Unit Button Released!"); buttonHeld_dog = false;
                 // No short press action for Dog unit button
            }
        }
    }
    lastButtonState_dog = reading; // Save the current reading for next iteration

    // Check for long press WHILE button is *still* held down
    if (buttonHeld_dog && buttonState_dog == LOW && (millis() - buttonPressStartTime_dog >= longPressDuration_dog)) {
        Serial.println("Dog Unit Long press -> Entering Deep Sleep.");
        buttonHeld_dog = false; // Prevent re-triggering

        // --- DEEP SLEEP ACTION ---
        tft.st7735_fill_screen(ST7735_BLACK); // Clear tft
        tft.st7735_write_str(0, 0, "Sleeping...", Font_7x10, ST7735_RED, ST7735_BLACK);
        // updatetft isn't suitable here as it needs GPS data etc.
        // We need to manually push the buffer if st7735 lib requires it (assume it doesn't unlike OLED)

        digitalWrite(LED_PIN, HIGH); // LED indicator for sleep entry
        delay(500); // Allow tft/LED to show briefly

        // Optional: Turn off Vext before sleep? Setup turns it back on after RST.
        // VextOFF();

        // Prepare for deep sleep
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
        Serial.println("Dog Unit Going to sleep now...");
        delay(100); // Allow serial message to flush
        esp_deep_sleep_start(); // Enter deep sleep

        // --- Execution stops here until hardware reset ---
    }
}