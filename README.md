# Tesla BMS firmware for eChook BMS Comms Master Board
Forked from Tom-evnut/TeslaBMSV2 and increasingly heavily adapted.

Target hardware: Teensy 4.1

## Changes from Original
- Converted to VSCode PlatformIO project from Arduino IDE project
- Adapted to Teensy 4.1 from Teensy 3.2
- Refactoring for clarity (Turning into a full re-architecting)
- Breaking code into multiple files for readability
- Pinout changes for BMS Comms Master board
- Adding BMS Comms Master board features not found in simpBms
    - Extra networks (2x CAN, RS-485)
    - High Side driver monitoring
    - Indication LEDs and Buzzer
    - 0.91" OLED I2C Display
 
## Progress Tracking

### Porting Progress
- [ ] Reboot Crash Report
    - [x] Compiling
    - [ ] Tested
- [ ] Under/Over Voltage Monitoring  
    - Removed for now - it triggerd an EEPROM Update.    
- [ ] Watchdog Timer Implementation
    - [x] Compiling
    - [x] Tested
- [ ] Switch to FlexCAN_T4 Library
    - [x] Compiling ***Removed filtering
    - [ ] Tested
     
### New Feature Progrss
- [x] Pinout Changes
- [x] LED and Buzzer Indicators (Rudimentary)
- [x] Logging to SD
- [x] I2C OLED Display
- [ ] Extra CAN networks
- [ ] RS-485

## Testing
- [x] Error LED
- [x] Heartbeat LED
- [x] Buzzer
- [x] Oled Display
- [ ] Isolated Serial
- [ ] Isolated CAN (CAN1)
- [ ] Aux CAN (CAN2)
- [ ] RJ45 CAN (CAN3)
- [ ] Aux Serial
- [ ] Current Reading
- [ ] Digital Inputs
- [ ] HSD Outputs
- [ ] HSD Monitoring
- [ ] LSD outputs


### Useful Links
https://openinverter.org/forum/viewtopic.php?t=131&start=75

ESP PORT, not sure which batteries:    
Code https://github.com/jamiejones85/ESP32-BMS  
Hardware https://oshwlab.com/EV-Team/spaceballs

'Fix' To CAN_T4 not recieveing some frames: https://openinverter.org/forum/viewtopic.php?p=62789#p62789

## Tesla module harnessing

Tesla Module BMS Connecor part number appears to be: 
Molex Mini-Fit TPA, 10ckt 15-97-5101 
https://www.molex.com/en-us/products/part-detail/15975101

Mouser Links:

15975101 MINIFIT TPA 10CKT Receptical Housing: https://www.mouser.co.uk/ProductDetail/Molex/15-97-5101  
TPA Key: https://www.mouser.co.uk/ProductDetail/Molex/15-97-9101  
CPA Key: https://www.mouser.co.uk/ProductDetail/538-15-97-0071  
Pins: https://www.mouser.co.uk/ProductDetail/Molex/39-00-0038




