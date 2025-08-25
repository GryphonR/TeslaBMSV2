#include "indicators.h"
#include "globals.h"
#include "pinouts.h"
#include "Libraries/Logger.h"

const int ENABLE_BUZZER = 0; // Enable buzzer control

// Error LED Patters
const bool errorPatterns[11][12] = {{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // One Short flash pattern
                                    {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // Two Short flash pattern
                                    {1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0},  // Three Short flash pattern
                                    {1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0},  // Four Short flash pattern
                                    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0},  // Five Short flash pattern
                                    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},  // Six Short flash pattern
                                    {1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0},  // Long flash pattern
                                    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // One Long flash pattern
                                    {1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},  // Two Long flash pattern
                                    {1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0},  // Three Long flash pattern
                                    {1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0}}; // Four Long flash pattern

uint8_t errorPattern;
uint8_t errorCount;

bool heartbeatState = false;
bool onboardState = false;
bool errorLedState = false;

uint8_t heartbearBrightness = 50; // Brightness for heartbeat LED
uint8_t errorLedBrightness = 50;   // Brightness for error LED

// Function prototypes
void errorLedLoop();
void heartbeatLoop();
void checkBmsStatus(); // Placeholder for BMB status check

// Setup function
void indicatorsSetup()
{
    pinMode(PIN_LED_BUILTIN, OUTPUT);
    pinMode(PIN_ERROR_LED, OUTPUT);
    pinMode(PIN_HEARTBEAT_LED, OUTPUT);
    pinMode(PIN_BUZZER_CONTROL, OUTPUT);
}

// Loop function
void indicatorsLoop()
{
    checkBmsStatus(); // Update error status as needed
    heartbeatLoop();
    errorLedLoop();
}

void heartbeatLoop()
{
    static unsigned long nextLedTime = 0;
    static uint8_t ledState = 0;
    if (millis() > nextLedTime)
    {
        // Logger::debug("Toggle heartbeat LED");
        if (ledState)
        {
            analogWrite(PIN_HEARTBEAT_LED, 0); // Turn off heartbeat LED
            // analogWrite(PIN_ERROR_LED, 0);     // Turn off heartbeat LED
            // digitalWrite(PIN_HEARTBEAT_LED, 0); // Turn off heartbeat LED
            ledState = 0;
            nextLedTime = millis() + 2990; // Set next toggle time
        }
        else
        {
            analogWrite(PIN_HEARTBEAT_LED, heartbearBrightness); // Turn on heartbeat LED
            // analogWrite(PIN_ERROR_LED, 50);     // Turn on heartbeat LED
            // digitalWrite(PIN_HEARTBEAT_LED, 1); // Turn on heartbeat LED
            ledState = 1;
            nextLedTime = millis()+ 10; // Set next toggle time
        }
    }
}

// Error LED logic
void errorLedLoop()
{
    if (bmsError)
    {
        static unsigned long nextErrorLedState = 0;
        if (millis() > nextErrorLedState)
        {
            nextErrorLedState = millis() + 500; // Set next toggle time
            if (errorCount < 12)
            {
                analogWrite(PIN_ERROR_LED, errorPatterns[bmsError][errorCount]? errorLedBrightness : 0); // Set error LED brightness based on pattern
                if(ENABLE_BUZZER){
                    digitalWrite(PIN_BUZZER_CONTROL, errorPatterns[bmsError][errorCount] ? HIGH : LOW); // Control buzzer based on error pattern
                }
                errorCount++;
            }
            else if (errorCount == 12)
            {                                       // turn builtin led off to indicate end of error pattern
                digitalWrite(PIN_LED_BUILTIN, LOW); // Turn off built-in LED
                nextErrorLedState -= 250;           // Half Length flash
                errorCount = 13;
            }
            else
            {
                digitalWrite(PIN_LED_BUILTIN, HIGH); // Turn on built-in LED
                nextErrorLedState += 250;            // one and a Half Length flash
                errorCount = 0;                      // Reset error count
            }
        }
    }
}

// Setters
void setOnboardLED(bool on)
{
    digitalWrite(PIN_LED_BUILTIN, on ? HIGH : LOW);
}


// Placeholder: Check BMB status and set error codes
void checkBmsStatus()
{
    static uint8_t lastBmsError = 0;
    if(bmsError != lastBmsError)
    {
        lastBmsError = bmsError;
        if (bmsError == ERROR_NONE)
        {
            errorPattern = 0; // No error
            errorCount = 0;
        }
        else
        {
            onboardState = false; // BMS has an error
            switch (bmsError)
            {
                case ERROR_BATTERY_COMMS:
                    errorPattern = 1; // Battery communication error
                    break;
                case ERROR_VOLTAGE:
                    errorPattern = 2; // Voltage error
                    break;
                case ERROR_CONTACTORS_NOT_CLOSING:
                    errorPattern = 3; // Contactors not closing
                    break;
                case ERROR_CONTACTORS_OPENED_EMERGENCY:
                    errorPattern = 4; // Contactors opened in emergency
                    break;
                case ERROR_CURRENT_READING:
                    errorPattern = 5; // Current reading error
                    break;
                case ERROR_OVER_VOLTAGE:
                    errorPattern = 6; // Over voltage
                    break;
                case ERROR_UNDER_VOLTAGE:
                    errorPattern = 7; // Under voltage
                    break;
                case ERROR_OVER_TEMPERATURE:
                    errorPattern = 8; // Over temperature
                    break;
                case ERROR_CAN:
                    errorPattern = 9; // CAN bus error
                    break;
                default:
                    errorPattern = 10; // Unknown error
            }
        }
    }
}