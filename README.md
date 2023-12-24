# VL53L7CX TOF Sensor Test
 Simple test demo for VL53L7CX MultiZone TOF Sensor 
## About project

This is simple demo project for ST’s VL53L7CX MultiZone time of flight (TOF) sensor. Project represent simple readout demo for VL53L7CX multizone (8x8 zones) TOF sensor using STM32F415RG mcu. Communication with PC is achieved using USB in CDC mode. "GET" command initiates data tranfer. "SET" command can be used for adjusting resolution (4x4 or 8x8, 1 byte), ranging frequency (60Hz max, 1 byte), integration time (2-1000 ms, 2 bytes), sharpener percent (0-99%, 1 byte) and data to transfer (0-distance, 1-sigma, 2-reflectance, 3-target status, 4-num of detected targets, 5-signal per SPAD, 6-amibient per SPAD, 7-Num of SPADs enabled). 

<p align="center">
<img src="https://github.com/OptoLAB/VL53L7CX-TOF-Sensor-Test/blob/main/img/VL53L7CX.jpg" width="600"/>
</p>

Demo project is built using following ecosystem:

- [STM32 M4 Clicker](https://www.mikroe.com/clicker-stm32f4) development board and [LightRanger 11 Click](https://www.mikroe.com/lightranger-11-click) sensor board by Mikroelektronika
- [STM32CubeIDE 1.14.0](https://www.st.com/en/development-tools/stm32cubeide.html) – Integrated Development Environment for STM32 with [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) initialization code generator by STMicroelectronics.
- [VL53L7CX ULD driver](https://www.st.com/en/embedded-software/stsw-img036.html) - an optimized driver for VL53L7CX
- ST-LINK-V2 - programmer for the STM8 and STM32 microcontroller families.

More info can be found [here](https://www.optolab.ftn.uns.ac.rs/index.php/education/project-base/297-multizone-tof-sensor-demo-vl53l7cx)

Demo video on [youtube](https://www.youtube.com/watch?v=VrheGO3DJa8) 
