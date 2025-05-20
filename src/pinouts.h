#pragma once
#include <Arduino.h>

// Simple BMS V2 wiring - some changes for BMS Comms Master//
const uint8_t PIN_ACUR_2 = A0; // current 1
const uint8_t PIN_ACUR_1 = A1; // current 2
const uint8_t PIN_IN1 = 16;   // input 1 - high active
const uint8_t PIN_IN2 = 17;   // input 2- high active
const uint8_t PIN_IN3 = 18;   // input 1 - high active
const uint8_t PIN_IN4 = 19;   // input 2- high active
const uint8_t PIN_OUT1 = 11;  // output 1 - high active - Positive Contactor
const uint8_t PIN_OUT2 = 12;  // output 2 - high active - Precharge Contactor
const uint8_t PIN_OUT3 = 20;  // output 3 - high active - Charge Relay
const uint8_t PIN_OUT4 = 21;  // output 4 - high active - Negative Contactor
const uint8_t PIN_OUT5 = 2;  // output 5 - Low active
const uint8_t PIN_OUT6 = 3;  // output 6 - Low active
const uint8_t PIN_OUT7 = 5;   // output 7 - Low active
const uint8_t PIN_OUT8 = 6;   // output 8 - Low active
const uint8_t PIN_LED_BUILTIN = 13;
const uint8_t PIN_BMB_FAULT = 11; //Battery Management Board Fault Indicator

// Additional pins for BMS Comms Master
const uint8_t PIN_BUZZER_CONTROL = 10;
const uint8_t PIN_OUT_5_STATUS = 26;
const uint8_t PIN_OUT_6_STATUS = 27;
const uint8_t PIN_OUT_7_STATUS = 28;
const uint8_t PIN_OUT_8_STATUS = 29;

const uint8_t PIN_HSD_SEL_0_A = 33;
const uint8_t PIN_HSD_SEL_1_A = 34;
const uint8_t PIN_HSD_SENSE_EN_A = 35;
const uint8_t PIN_HSD_SEL_MULTISENSE_A = 36;
const uint8_t PIN_HSD_SEL_0_B = 37;
const uint8_t PIN_HSD_SEL_1_B = 38;
const uint8_t PIN_HSD_SENSE_EN_B = 39;
const uint8_t PIN_HSD_SEL_MULTISENSE_B = 40;
// const uint8_t PIN_HSD_SEL_ = 41;