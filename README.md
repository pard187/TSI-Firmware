# TSI-Firmware

This branch has been refactored to keep track of the TSI firmware versions and to make deployment to additional computers easier. It is the location for the most recent firmware code written by the 2022 team while also keeping the 2020-2021 teams work for reference. Note that some of the older code versions used the Harmony Configurator which requires a different setup process. It is not used by the 2022 firmware and is not recommended for this project anyway.

# Installation and Running

1. Install MPLAB X IDE **(v5.40 recommended)** from https://www.microchip.com/mplab/mplab-x-ide.

2. Install XC32 cross compiler **(v2.41 recommended)** from https://www.microchip.com/development-tools/pic-and-dspic-downloads-archive.

3. Clone this branch to your local computer. <br>
   ↳ Might need to add the project path to MPLAB after cloning, depending on the OS.

4. Open the project folder at TSI-Firmware/2022_firmware/New_PIC_Firmware.X from MPLAB X IDE and set it as the main project.

5. Build the main project. <br>
   ↳ Make sure the project was built successfully before proceeding. If there are any errors, they should be fixed here.

6. Now go ahead and connect the PICkit 3 to the computer, the LEDs on the PICkit should turn on. The PICkit also has to be connected to the corresponding pins on the TSI and the PIC32 has to be powered up (has to receive 3.3 V).

7. Run the main project and when prompted to choose a hardware tool, you should be able to choose the PICkit 3. <br>
   ↳ If the PICkit is not there, go back and check that everything is as described in step 6. If it's still not, then there is a chance the PICkit is bad so try replacing it.
