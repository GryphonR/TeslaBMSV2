# Welcome
These documents cover the code designed to use the eChook BMS Comms Master Board with Tesla battery modules.

## Project Introduction
This code has been forked from the SimpBMS TeslaBMSV2 project. While the initial idea was some light adaptations just to get it working with the Teensy 4.1 (Original Target, Teensy 3.2) and the eChook BMS Comms Master Board, it quickly expanded into more of a complete re-architecting as I restructured the code to make it more modular and readable (in my opinion), expanded the logging functionality to aid state tracing and debugging, extending safety checks, and added a more robust contactor management implementation - amongst other changes!

This code is not a plug and play solution to battery management - it will need code level edits and recompiling for any application, and I make no claims that it will work as intended. Perform your own testing and use it at your own risk.

## License
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.  
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## Setup

The system setup has changed very little from the SimpBMS, meaning the original [SimpBMS manual](https://github.com/Tom-evnut/SimpBMS/blob/master/Simp%20BMS%20Setup%20Manual%20V0.29.pdf) is a useful resource.  
Configuring the contactors however has changed. Docs TODO!

## Logging

The logging ability of the SimpBMS felt a little limited. The eChook board has two other output options; the SD card on the Teensy 4.1 PCB, and an optional I2C 0.91" OLED display plugged into the 4 pin header below the Teensy. These have different usecases - the OLED is designed for immediate feedback during system setup and configuration, to be removed once the system is deployed, and the SD card is for long term logging, trackig state and battery changes over time.
To aid this the eChook board has a built in Real Time Clock, and an optional CR2032 3V button cell on the reverse of the boar to maintain the time when power is lost.

To configure the log levels for each device, look at the start of the setup function:
```
  Logger::setSerialLoglevel(Logger::Info); // Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4
  Logger::setSdLoglevel(Logger::Info); // Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4
  Logger::setOledLoglevel(Logger::Info); // Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4
```