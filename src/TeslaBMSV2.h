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
void Prechargecon();
void currentlimit();
void getcurrent();
void alarmupdate();
void dashupdate();
void resetwdog();
void VEcan();
void chargercomms();
void setBMSstatus(int status, const char *message);
void contactorControl();

    // Variable declarations
    // extern int firmver;
    // extern BMSModuleManager bms;
    // extern EEPROMSettings settings;
    // extern ADC *adc;
    // extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
    // extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
    // extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
