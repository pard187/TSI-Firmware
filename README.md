# TSI-Firmware

This branch of the TSI-Firmware has been refactored to make deployment to additional computers easier. It is also the location for all new code written by the 2020-2021 team. 

# Installation

1. Install MPLAB X IDE from https://www.microchip.com/mplab/mplab-x-ide

2. Install Harmony Configurator **v2.06** from https://www.microchip.com/mplab/mplab-harmony/mplab-harmony-v2
  ↳ It is highly recommended you install to C:\microchip\harmony . In our experience, other installation locations tend not to work. 
  
3. Install XC32 cross compiler **v2.05** from https://www.microchip.com/development-tools/pic-and-dspic-downloads-archive

4. Clone this branch to C:\microchip\harmony .
  ↳ Again,  highly recommended you use this file structure.
  
5. Upon opening MPLAB, you will need to install the Harmony Plugin by going to Tools -> Plugins -> Downloaded and hitting the Add Plugins... button. Then locate the .nbm file at harmony\v2_06\utilities\mhc\

6. Open the project folder at harmony\TSI-Firmware\firmware\TSIPCBFirmware.X and set it as the main project

7. Verify Harmony was installed by selecting Tools -> Embedded -> MPLAB Harmony Configurator

