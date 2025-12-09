/**
 * @file CurrentSensing.cpp
 * @brief Functions for ANALOGUE Current Sensing.
 *
 * If a CAN based current sensor is used, it's handled in the CAN files
 * 
 */

#include "CurrentSensing.h"
#include "globals.h"
#include "pinouts.h"

#include <Filters.h> //https://github.com/JonHub/Filters

// Curent filter//
float filterFrequency = 5.0;
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);


// Reads a specific ADC pin and converts to raw mA based on offset/conversion
int32_t readAdcAsmA(uint8_t pin, uint16_t offset, float convFactor) {
    // Start continuous read (if not already running, simplified for this example)
    adc->adc0->startContinuous(pin);
    
    // Read raw value
    uint16_t rawAdc = (uint16_t)adc->adc0->analogReadContinuous();
    
    // Convert to mV
    // Note: Using getMaxValue() ensures we don't hardcode 4095 or 1023
    float voltageMv = (float)rawAdc * ADC_REF_MV / adc->adc0->getMaxValue();
    
    // Deadband check (if close to offset, return 0)
    if (abs(voltageMv - offset) < settings.CurDead) {
        return 0;
    }

    // Calculate Current
    // Original formula: (Voltage - Offset) / (Conv * 0.0000066)
    return (int32_t)((voltageMv - offset) / (convFactor * CURRENT_SCALE_FACTOR));
}

// Handles the Coulomb Counting (Amp-Seconds)
// Call this every loop to keep integration accurate!
void updateCoulombCounter(float currentMa) {
    unsigned long now = millis();
    float dt = (now - lasttime) / 1000.0; // Delta time in seconds
    
    // Only integrate if time has passed
    if (dt > 0) {
        // Add (Amps * Seconds) -> result is Amp-Seconds
        // Note: dividing mA by 1000 to get Amps
        ampsecond += (currentMa / 1000.0) * dt; 
        lasttime = now;
    }
}

// The main processing pipeline (Filter -> Invert -> Integrate)
void processCurrentValue(int32_t rawInput) {
    
    // 1. Invert if necessary
    if (settings.invertcur == 1) {
        rawInput *= -1;
    }

    // 2. Apply Low Pass Filter
    lowpassFilter.input(rawInput);
    currentact = lowpassFilter.output();
    
    // 3. Update the Coulomb Counter with the filtered value
    if (abs(currentact) > COULOMB_COUNTER_IGNORE_mA) { // Example noise gate
         updateCoulombCounter(currentact);
    } else {
         lasttime = millis(); // Reset time so we don't integrate the "gap"
    }

    // Final global update
    currentact = settings.ncur * currentact; 

    // --- Debugging ---
    if (debugCur != 0) {
        SERIAL_CONSOLE.print("Raw: "); SERIAL_CONSOLE.print(rawInput);
        SERIAL_CONSOLE.print(" | Filtered: "); SERIAL_CONSOLE.print(currentact);
        SERIAL_CONSOLE.print("mA | Ah: "); SERIAL_CONSOLE.println(ampsecond / 3600.0);
    }
}

// --- Main Current Functions ---

void getcurrent() {
    int32_t newRawCurrent = 0;

    // 1. ANALOG SENSING
    if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL || settings.cursens == CURR_SENSE_ANALOGUE_GUESSING) {
        
        bool useLowRange = true;

        // Determine which sensor to use
        if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL) {
            // Check if we are within the low range limits
            if (abs(currentact) < settings.changecur) {
                useLowRange = true;
            } else {
                useLowRange = false;
            }
        } 
        
        if (useLowRange) {
            sensor = 1; // Global tracker
            newRawCurrent = readAdcAsmA(PIN_ACUR_1, settings.offset1, settings.convlow);
        } else {
            sensor = 2; // Global tracker
            newRawCurrent = readAdcAsmA(PIN_ACUR_2, settings.offset2, settings.convhigh);
        }
        
        // Pass the analog reading into the pipeline
        processCurrentValue(newRawCurrent);
    }
    
    // 2. CANBUS SENSING 
    // If CANBUS is selected, this function usually does nothing 
    // because 'CAB500()' calls processCurrentValue() directly.
    // However, if you have any continuous tasks for CAN, put them here.
}


// Helper to average 20 readings for calibration
uint16_t performCalibration(uint8_t pin) {
    uint32_t totalMv = 0; // Use a local variable so we don't corrupt the setting if we fail
    
    adc->adc0->startContinuous(pin);
    SERIAL_CONSOLE.print("Calibrating Pin "); 
    SERIAL_CONSOLE.print(pin);
    SERIAL_CONSOLE.print(" : ");

    // Take 20 samples
    for (int i = 0; i < 20; i++) {
        // Read raw
        uint16_t raw = (uint16_t)adc->adc0->analogReadContinuous();
        // Convert to mV immediately
        uint16_t mv = raw * 3300 / adc->adc0->getMaxValue();
        
        totalMv += mv;
        
        SERIAL_CONSOLE.print(".");
        delay(100); // Keep the delay to average out noise over time
    }
    
    SERIAL_CONSOLE.println(" Done.");
    
    // Return the average
    return (uint16_t)(totalMv / 20);
}

// The main function you call from your setup or menu
void calcur() {
    SERIAL_CONSOLE.println("Starting Current Calibration...");
    
    // Calibrate Sensor 1
    settings.offset1 = performCalibration(PIN_ACUR_1);
    SERIAL_CONSOLE.print("Offset 1 New Value: ");
    SERIAL_CONSOLE.println(settings.offset1);
    
    // Calibrate Sensor 2
    settings.offset2 = performCalibration(PIN_ACUR_2);
    SERIAL_CONSOLE.print("Offset 2 New Value: ");
    SERIAL_CONSOLE.println(settings.offset2);
    
    // Ideally, save to EEPROM here so you don't have to calibrate every boot!
    // saveSettings(); 
}