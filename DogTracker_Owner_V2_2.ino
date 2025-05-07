#include <Arduino.h>
#include "LoRaWan_APP.h"       // Heltec LoRa library
#include "HT_SSD1306Wire.h"    // Heltec OLED library
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>           // Include for BLE descriptors
#include "esp_sleep.h"         // Include for deep sleep functions

// --- Pin Definitions (Heltec WiFi LoRa 32 V3.2) ---
#define BUZZER_PIN 46  // Use GPIO46
#define VEXT_PIN   36
#define LED_PIN    35
#define BUTTON_PIN 0   // Onboard PRG button

// --- LoRa Configuration ---
// ... (Keep LoRa defines exactly as they were in the working version) ...
#define RF_FREQUENCY          915000000
#define TX_OUTPUT_POWER       14
#define LORA_BANDWIDTH        0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE       1
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT   0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON  false
#define RX_TIMEOUT_VALUE      1000
#define TX_TIMEOUT_VALUE      3000

// --- Application Configuration ---
#define PING_INTERVAL      3000
#define GPS_DATA_PREAMBLE "12345-96458"
#define PING_MESSAGE      "PING"
int rssiThreshold; // Variable threshold

// --- Button Debouncing & Long Press ---
int buttonState;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const long debounceDelay = 50;
unsigned long buttonPressStartTime = 0; // Time when button was pressed down
bool buttonHeld = false;             // Flag: Is button currently held?
const long longPressDuration = 3000;  // 3 seconds for long press trigger

// --- BLE Configuration ---
#define SERVICE_UUID        "f65ee2a3-c6ed-4778-8813-bd82fcf9640b"
#define CHARACTERISTIC_UUID "d0018bd3-be79-4157-b37b-8ede34ee54bf"
BLEServer *pServer = NULL; BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false; String latestGpsData = "No Data Yet";

// --- OLED Display Object ---
extern SSD1306Wire display;

// --- LoRa State Machine ---
typedef enum { LOWPOWER, STATE_RX, STATE_TX } States_t;
States_t State = LOWPOWER;

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

// --- Global Variables ---
static RadioEvents_t RadioEvents;
char txPacket[30]; char rxPacket[128];
volatile bool loraInterruptFired = false; volatile bool txDone = false;
volatile bool rxDone = false; volatile bool txTimeout = false; volatile bool rxTimeout = false;
uint32_t lastPingTime = 0; int16_t lastRssi = 0; int8_t lastSnr = 0; uint16_t rxSize = 0;
String displayLine1 = "Owner Unit Init..."; String displayLine2 = "";
String displayLine3 = ""; String displayLine4 = "";

// --- Function Prototypes ---
void VextON(); void VextOFF();
void setupLoRa(); void setupDisplay(); void setupBLE();
void updateDisplay(); void sendPing();
void handleReceivedPacket(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void startReceive();
// void triggerBuzzer(bool active); // REMOVED - Replaced by manageBuzzerPattern
void manageBuzzerPattern(); // ADDED
void checkButton();

// --- BLE Server Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks { /* ...as before... */ };

// --- LoRa Callback Functions ---
// ... (OnTxDone, OnRxDone, OnTxTimeout, OnRxTimeout as before, setting flags) ...
void OnTxDone( void ) { Radio.Sleep(); txDone = true; loraInterruptFired = true; }
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ) { Radio.Sleep(); uint16_t copySize = (size < sizeof(rxPacket) - 1) ? size : sizeof(rxPacket) - 1; memcpy(rxPacket, payload, copySize); rxPacket[copySize] = '\0'; lastRssi = rssi; lastSnr = snr; rxSize = copySize; rxDone = true; loraInterruptFired = true; }
void OnTxTimeout( void ) { Radio.Sleep(); txTimeout = true; loraInterruptFired = true; }
void OnRxTimeout( void ) { Radio.Sleep(); rxTimeout = true; loraInterruptFired = true; }


// --- Setup ---
void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting Owner Unit w/ Button & Buzzer Pattern...");
    rssiThreshold = -100; // Start at -100 dBm
    VextON();
    pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer starts LOW
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    setupDisplay(); setupLoRa(); setupBLE();
    displayLine1 = "Owner Unit Ready"; displayLine2 = "Listening...";
    updateDisplay();
    startReceive();
    lastPingTime = millis(); lastButtonState = digitalRead(BUTTON_PIN);
    Serial.println("Setup Complete. Entering Loop.");
}

// --- Main Loop ---
void loop() {
    Radio.IrqProcess(); // Process LoRa interrupts FIRST
    checkButton();      // Check button state
    manageBuzzerPattern(); // Manage non-blocking buzzer sequence

    bool handledInterrupt = false;
    if (loraInterruptFired) { /* ... Handle LoRa flags as before ... */
        loraInterruptFired = false; handledInterrupt = true;
        if (txDone) { txDone = false; State = LOWPOWER; digitalWrite(LED_PIN, LOW); Serial.println("Ping TX complete, returning to RX"); startReceive();
        } else if (rxDone) { rxDone = false; Serial.printf("Processing RX: Payload='%s', RSSI=%d\n", rxPacket, lastRssi); handleReceivedPacket((uint8_t*)rxPacket, rxSize, lastRssi, lastSnr); State = LOWPOWER; Serial.println("RX processing done, returning to RX mode"); startReceive();
        } else if (txTimeout) { txTimeout = false; State = LOWPOWER; digitalWrite(LED_PIN, LOW); Serial.println("Ping TX timeout, returning to RX"); startReceive();
        } else if (rxTimeout) { rxTimeout = false; State = LOWPOWER; Serial.println("Data RX timeout, continuing RX"); startReceive();
        } else { if (State != STATE_TX) { Serial.println("Unknown LoRa interrupt, ensuring RX mode."); startReceive(); } }
    }

    // Send PING if needed
    if (!handledInterrupt && State != STATE_TX) { /* ... as before ... */
        if (millis() - lastPingTime > PING_INTERVAL) { State = STATE_TX; sendPing(); lastPingTime = millis();
        } else if (State == LOWPOWER) { startReceive(); }
    }

    // Update Display periodically
    static uint32_t lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 1000) { updateDisplay(); lastDisplayUpdate = millis(); }

    delay(5);
}

// --- Helper Functions ---
void VextON() { pinMode(VEXT_PIN, OUTPUT); digitalWrite(VEXT_PIN, LOW); Serial.println("Vext Power ON"); delay(100); }
void VextOFF() { pinMode(VEXT_PIN, OUTPUT); digitalWrite(VEXT_PIN, HIGH); Serial.println("Vext Power OFF"); }

void setupDisplay() {
    // Initialize the implicitly defined display object
    display.init();
    display.flipScreenVertically(); // From uploaded PDF code
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.clear();
    // UpdateDisplay() in setup() will draw initial text
    Serial.println("OLED Display Initialized (using extern 'display')");
}

void setupLoRa() { // Uses defines from top of file
    RadioEvents.TxDone = OnTxDone; RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout; RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init(&RadioEvents); Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    Serial.println("LoRa Radio Initialized");
}

void setupBLE() { // Uses defines from top of file
    Serial.println("Initializing BLE..."); BLEDevice::init("DogTrackerOwner");
    pServer = BLEDevice::createServer(); pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->setValue(latestGpsData); pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising(); pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true); pAdvertising->setMinPreferred(0x06); pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising(); Serial.println("BLE Service Started, Advertising...");
}

// Updated checkButton to include long press for sleep
void checkButton() {
    int reading = digitalRead(BUTTON_PIN);

    // Debounce logic (same as before)
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        // Stable state
        if (reading != buttonState) { // State has officially changed
            buttonState = reading;

            if (buttonState == LOW) { // Button Newly Pressed Down
                Serial.println("Button Pressed Down!");
                buttonPressStartTime = millis(); // Record time pressed
                buttonHeld = true;
            } else { // Button Newly Released (HIGH)
                 Serial.println("Button Released!");
                 if (buttonHeld) { // Was it previously held down?
                     buttonHeld = false; // No longer held
                     // Check if it was released *before* the long press duration triggered sleep
                     if (millis() - buttonPressStartTime < longPressDuration) {
                         // --- SHORT PRESS ACTION ---
                         Serial.println("Short press action!");
                         rssiThreshold -= 20;
                         if (rssiThreshold < -240) { rssiThreshold = -40; }
                         Serial.printf("New RSSI Threshold: %d dBm\n", rssiThreshold);
                         updateDisplay(); // Update display immediately
                     } else {
                         // Button was released *after* long press duration,
                         // sleep should have already been triggered by the check below.
                         Serial.println("Long press released (Sleep should have initiated).");
                     }
                 }
            }
        }
    }
    // Save the raw button reading.
    lastButtonState = reading;

    // Check for long press WHILE button is *still* held down
    // Add a check for buttonHeld to prevent triggering sleep multiple times if loop is fast
    if (buttonHeld && buttonState == LOW && (millis() - buttonPressStartTime >= longPressDuration)) {
        Serial.println("Long press detected! Entering Deep Sleep.");
        buttonHeld = false; // IMPORTANT: Prevent re-triggering sleep immediately

        // --- DEEP SLEEP ACTION ---
        // Optional: Prepare display or peripherals for sleep
        display.clear();
        display.drawString(0, 0, "Sleeping...");
        display.display();
        digitalWrite(LED_PIN, HIGH); // Turn LED on to indicate sleep entry
        delay(200); // Allow display/LED to show message briefly

        // Configure ESP32 to enter deep sleep indefinitely
        // Wakeup will require RST button press or power cycle
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL); // Good practice
        // Optionally enable button wakeup on GPIO 0 if needed later, but RST is simpler
        // esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0); // Wake on LOW level on GPIO0

        Serial.println("Going to sleep now...");
        delay(100); // Allow serial message to flush

        esp_deep_sleep_start(); // Enter deep sleep

        // --- Code execution stops here until reset ---
    }
}

void updateDisplay() { // Uses extern 'display' and global rssiThreshold
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, displayLine1);
    display.drawString(0, 12, displayLine2);
    display.drawString(0, 24, displayLine3);
    display.drawString(0, 36, "Thresh:" + String(rssiThreshold) + "dBm"); // Show current threshold
    display.drawString(0, 50, deviceConnected ? "BLE Connected" : "BLE Waiting");
    display.display();
}

void sendPing() {
     if (State != STATE_TX) {
         Serial.println("Error: Trying to ping when not in TX state.");
         State = LOWPOWER; startReceive(); return;
     }

     // Format the packet: PING,thresholdValue
     // Ensure rssiThreshold is accessed correctly (it's global, should be fine)
     snprintf(txPacket, sizeof(txPacket), "%s,%d", PING_MESSAGE, rssiThreshold); // Format PING,<threshold>

     Serial.printf("Sending LoRa PING Packet: %s\n", txPacket); // Log the actual packet being sent
     // displayLine1 = "LoRa: Sending PING"; // Status update handled by updateDisplay based on State
     digitalWrite(LED_PIN, HIGH);
     Radio.Send((uint8_t*)txPacket, strlen(txPacket));
     // State change happens in OnTxDone/OnTxTimeout callbacks
}

void startReceive() { // Uses defines from top of file
    if (State == STATE_TX) return;
    // Use RX_TIMEOUT_VALUE or 0 for continuous; PDF version used 0
    if (State != STATE_RX) { State = STATE_RX; Radio.Rx(0); /*displayLine1 = "Listening...";*/ }
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


void handleReceivedPacket(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    // Uses defines/variables: GPS_DATA_PREAMBLE, latestGpsData, pCharacteristic, deviceConnected,
    // displayLine1-4, rssiThreshold, BUZZER_PIN
    if (strncmp((char*)payload, GPS_DATA_PREAMBLE, strlen(GPS_DATA_PREAMBLE)) == 0) {
        // ... (Update BLE data, display lines 1-3 as before) ...
        latestGpsData = String((char*)payload); pCharacteristic->setValue(latestGpsData); if (deviceConnected) pCharacteristic->notify();
        float lat, lon;
        if (sscanf(latestGpsData.c_str(), GPS_DATA_PREAMBLE ",%f,%f", &lat, &lon) == 2) {
             displayLine1 = "GPS OK"; displayLine2 = "Lat: " + String(lat, 4); displayLine3 = "Lon:" + String(lon, 4) + " R:" + String(rssi);
        } else { displayLine1 = "GPS Parse Err"; displayLine2 = "Raw:" + latestGpsData.substring(0,18); displayLine3 = "RSSI: " + String(rssi) + "dBm"; }

        // Check threshold using VARIABLE rssiThreshold
        if (rssi <= rssiThreshold) { // Use <=
            // --- START BUZZER SEQUENCE (Non-Blocking) ---
            if (!isBuzzingActive) { // Start sequence only if not already running
                Serial.printf("RSSI %d <= Threshold %d! Starting Buzzer Sequence!\n", rssi, rssiThreshold);
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
    } else {
        // ... (Handle bad preamble, update display lines 1-3) ...
        displayLine1 = "Bad Preamble"; displayLine2 = String((char*)payload).substring(0, 18); displayLine3 = "RSSI: " + String(rssi) + "dBm";
    }
    // Note: Display Line 4 (Threshold) is updated by checkButton or the periodic updateDisplay
}

// triggerBuzzer function is removed
