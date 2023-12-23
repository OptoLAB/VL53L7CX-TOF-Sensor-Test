# VL53L7CX TOF Sensor Test
 Simple test demo for VL53L7CX MultiZone TOF Sensor 
## About project

This is simple demo project for ST’s VL53L7CX MultiZone time of flight (TOF) sensor. Project present simple readout demo for TCD TCD1304AP CCD sensor with 3648 light sensitive pixels. Firmware is built around timer modules using MPLAB Harmony v3. Communication with PC is achieved using USB in CDC mode (256000 baud rate). "GET" command initiates data tranfer. "SET" command can bi used for adjusting integration time (10us-655.35ms), veritcal resolution (6, 8, 10 or 12 bits) and horizontal resolution (number of measurement points/pixels). 

Folder img contains some oscilloscope screenshots of important signals obtained during firmware development.

Demo project is built using following ecosystem:

- [STM32 M4 Clicker](https://www.mikroe.com/clicker-stm32f4) development board and [LightRanger 11 Click](https://www.mikroe.com/lightranger-11-click) sensor board by Mikroelektronika
- [STM32CubeIDE 1.14.0](https://www.st.com/en/development-tools/stm32cubeide.html) – Integrated Development Environment for STM32 with [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) initialization code generator by STMicroelectronics.
- [VL53L7CX ULD driver](https://www.st.com/en/embedded-software/stsw-img036.html) - an optimized driver for VL53L7CX
- ST-LINK-V2 - programmer for the STM8 and STM32 microcontroller families.

More info can be found [here](https://www.optolab.ftn.uns.ac.rs/index.php/education/project-base/297-multizone-tof-sensor-demo-vl53l7cx)

Demo video on [youtube](https://www.youtube.com/watch?v=VrheGO3DJa8) 
