# GNOR
![alt tag](https://github.com/UCFInnovationLab/GNOR/blob/master/orange.jpg) ![alt tag](https://github.com/UCFInnovationLab/GNOR/blob/master/boathull.gif)
##### GNOR Description
The Great Navel Orange Race (GNOR) is an annual competition held at UCF every year for the second intro to engineering course. The project involves students building a boat, submarine, or other watercraft that autonomously carries an orange around the reflection pond.
The TI Innovation lab is providing students with TI microcontroller boards (MSP430f5529) and sensors (TI Sensor Hub) for use in their watercrafts. These components empower students to control servos, provide signals for high power relays and ESCs, measure angle change relative to starting angle, look at accelerometer data, and more.
This repository provides everything needed to get started using these components. This includes example code, pinouts, and more. If you have any questions or are having trouble getting started, you can find help at the UCF Innovation Lab in ENGII room 112 10AM-10PM M-F and Saturday 10-5. 

#####Boat Kit
![alt tag](https://github.com/UCFInnovationLab/GNOR/blob/master/GNOR%20Kit.jpg)
The boat kit that we are providing includes the following:
* 1x TI MSP430f5529 Microcontroller Board
* 1x TI BoostXL Sensor Hub
* 1x Servo Motor
* 1x Adapter Board

##### Code Composer *Local*
* Go to http://www.ti.com/tool/ccstudio and download CCS latest version
* Install on your computer.


##### MSP430f5529 LaunchPad pin descriptions
* P1.0 - LED1 LaunchPad (GND reference)
* P1.1 - Button2 LaunchPad (GND - no pullup)
* P1.2 - Timer Capture (TA0.1)
* P1.3 - Timer Capture (TA0.2)
* P1.4 - PWM Out (Timer TA0.3)
* P1.5 - PWM Out (Timer TA0.4)
* P2.0 - 9150 Int_Motion
* P2.1 - Button1 LaunchPad (GND - no pullup)
* P2.4 - PWM Out (Timer TA2.1)
* P2.5 - PWM Out (Timer TA2.2)
* P3.0 - i2c SDA
* P3.1 - i2c SCL
* P3.3 - UART UCA0_TXD
* P3.4 - UART UCA0_RXD
* P4.4 - UART UCA1_TXD (connected to Application USB COM port through emulator)
* P4.5 - UART UCA1_RXD (connected to Application USB COM port through emulator)
* P4.7 - LED2 LaunchPad (GND reference)
* P6.2 - LED SensorHub (GND reference)
* P6.3 - Push Button SensorHub 1 (GND - 10k pullup)
* P6.4 - Push Button SensorHub 2 (GND - 10k pullup)
