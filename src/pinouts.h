#pragma once
#include <Arduino.h>

// ==========================
// Analog Current Sensors
// ==========================
const uint8_t PIN_ACUR_2 = 14; // A0 current 1
const uint8_t PIN_ACUR_1 = 15; // A1 current 2

// ==========================
// Digital Inputs
// ==========================
const uint8_t PIN_IN1 = 16; // input 1 - high active
const uint8_t PIN_IN2 = 17; // input 2 - high active
const uint8_t PIN_IN3 = 18; // input 3 - high active
const uint8_t PIN_IN4 = 19; // input 4 - high active

// ==========================
// Digital Outputs (Contactors, Relays, etc.)
// ==========================
const uint8_t PIN_OUT1 = 11; // output 1 - high active - Positive Contactor
const uint8_t PIN_OUT2 = 12; // output 2 - high active - Precharge Contactor
const uint8_t PIN_OUT3 = 20; // output 3 - high active - Charge Relay
const uint8_t PIN_OUT4 = 21; // output 4 - high active - Negative Contactor
const uint8_t PIN_OUT5 = 2;  // output 5 - low active
const uint8_t PIN_OUT6 = 3;  // output 6 - low active
const uint8_t PIN_OUT7 = 5;  // output 7 - low active
const uint8_t PIN_OUT8 = 6;  // output 8 - low active

// ==========================
// CAN Bus Interfaces
// ==========================
const uint8_t PIN_CAN1_RX = 22; // CAN1 RX pin
const uint8_t PIN_CAN1_TX = 23; // CAN1 TX pin
const uint8_t PIN_CAN2_RX = 0;  // CAN2 RX pin
const uint8_t PIN_CAN2_TX = 1;  // CAN2 TX pin
const uint8_t PIN_CAN3_RX = 30; // CAN3 RX pin
const uint8_t PIN_CAN3_TX = 31; // CAN3 TX pin

// ==========================
// Serial Communication
// ==========================
const uint8_t PIN_SERIAL_ISO_RX = 7;  // Isolated Serial RX (Teensy Serial 2)
const uint8_t PIN_SERIAL_ISO_TX = 8;  // Isolated Serial TX (Teensy Serial 2)
const uint8_t PIN_SERIAL_2_RX = 28;   // Serial 2 RX (J5) (Teensy Serial 7)
const uint8_t PIN_SERIAL_2_TX = 29;   // Serial 2 TX (J5) (Teensy Serial 7)

// ==========================
// RS485 Communication
// ==========================
const uint8_t PIN_RS485_TX = 34;  // RS485 TX (Serial8)
const uint8_t PIN_RS485_RX = 35;  // RS485 RX (Serial8)
const uint8_t PIN_RS485_DIR = 9;  // RS485 Direction Control

// ==========================
// I2C Interface
// ==========================
const uint8_t PIN_I2C_SDA = 25; // I2C2 SDA
const uint8_t PIN_I2C_SCL = 24; // I2C2 SCL

// ==========================
// LEDs & Buzzer
// ==========================
const uint8_t PIN_HEARTBEAT_LED = 4;   // Heartbeat LED
const uint8_t PIN_LED_BUILTIN = 13;    // Built-in LED
const uint8_t PIN_ERROR_LED = 33;      // Fault Indicator LED
const uint8_t PIN_BUZZER_CONTROL = 10; // Buzzer

// ==========================
// High-Side Driver (HSD) Feedback
// ==========================
const uint8_t PIN_HSD_SEL_0_A = 32;
const uint8_t PIN_HSD_SEL_1_A = 27;
const uint8_t PIN_HSD_SENSE_EN_A = 26;
const uint8_t PIN_HSD_SEL_MULTISENSE_A = 41; // A17

const uint8_t PIN_HSD_SEL_0_B = 37;
const uint8_t PIN_HSD_SEL_1_B = 38;
const uint8_t PIN_HSD_SENSE_EN_B = 39;
const uint8_t PIN_HSD_SEL_MULTISENSE_B = 40; // A16

const uint8_t PIN_HSD_FAULT_RESET = 36;