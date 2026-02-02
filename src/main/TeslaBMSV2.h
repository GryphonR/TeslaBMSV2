#pragma once

#include <Arduino.h>

// Function declarations
void bmsLoop();
void balancing();
void printbmsstat();
void inputdebug();
void outputdebug();
void gaugeUpdate();
void updateSOC();
void SOCcharged(int y);
// currentlimit is implemented in power/limits.cpp
#include "power/limits.h"
void getcurrent();
void alarmupdate();
void dashupdate();
void resetwdog();
void VEcan();
void chargercomms();
void setBMSstatus(int status, const char *message);

    // Variable declarations
    // extern int firmver;
    // extern BMSModuleManager bms;
    // extern EEPROMSettings settings;
    // extern ADC *adc;
    // extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
    // extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
    // extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
