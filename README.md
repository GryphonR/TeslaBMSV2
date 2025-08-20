# TeslaBMSV2 for SimpBMS
Porting to Teensy 4

## Changes from Original
- Converted to VSCode PlatformIO project from Arduino IDE project

## Porting Progress
- [ ] Reboot Crash Report
    - [x] Compiling
    - [ ] Tested
- [ ] Under/Over Voltage Monitoring  
    - Removed for now - it triggerd an EEPROM Update.    
- [ ] Watchdog Timer Implementation
    - [x] Compiling
    - [ ] Tested
- [ ] Switch to FlexCAN_T4 Library
    - [x] Compiling ***Removed filtering
    - [ ] Tested


### Useful Links
https://openinverter.org/forum/viewtopic.php?t=131&start=75

ESP PORT, not sure which batteries:    
Code https://github.com/jamiejones85/ESP32-BMS  
Hardware https://oshwlab.com/EV-Team/spaceballs

'Fix' To CAN_T4 not recieveing some frames: https://openinverter.org/forum/viewtopic.php?p=62789#p62789

Tesla Module BMS Connecor part number appears to be: 15-97-5101 