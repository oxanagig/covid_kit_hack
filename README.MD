# Ellume COVID-19 Test Kit Hack - iPhone Camera  Shutter Remote Control

The code is a quick and dirty hack of the Ellume Bluetooth COVID-19 kit. It becomes an iPhone Camera shutter remote control.

For people who don't know this kit, Ellume produced a one-time use [Bluetooth enabled COVID test kit](https://www.ellumecovidtest.com), and the most expensive part is probably the Nordic rf52810 Bluetooth SOC, which is used to read the test strip and upload the result to your phone through Bluetooth Low Energy. Here is the teardown video from The Signal Path. 

[![Watch the video](https://img.youtube.com/vi/UvArprBmdFA/default.jpg)](https://youtu.be/UvArprBmdFA)

I decide to take a step further. I have reversed-engineered the schematic of the PCB and reprogrammed the firmware to repurpose this whole kit as my iPhone Camera remote shutter. :stuck_out_tongue_winking_eye:

## Demo Video

[![Watch the video](https://img.youtube.com/vi/n731NR8O8hs/default.jpg)](https://youtu.be/n731NR8O8hs)

## PCB and SoC Wiring(sorry about my handwriting :grimacing: ):
![wiring](/images/covid%20test%20kit%20pinout.png)


## How it works

### :page_facing_up: Firmware
The code is modified from the BLE HID keyboard example from Nordic SDK, and when the only button on this device is pressed down, it sends the volume down button key through the HID interface to the iPhone([here is the accessory design guideline from Apple](https://developer.apple.com/accessories/Accessory-Design-Guidelines.pdf)). 

If you are an iPhone user, you probably know that when you have the Camera app open, the volume buttons are also the shutter button. 

### :round_pushpin: Button
The only button on this device serves as both the power button and the shutter button after pairing with a phone. The push button won't latch, but once the button is pressed the battery power will be delivered to the SoC, then I have to set the pin P0.00 high, which serves as a soft latch. At the same time, pin P0.25 is set up as input with the internal pull-up, so it can detect the state of the button. 

Since there is no other button, I have programmed a countdown timer to turn off the device. If there is user interaction for 3 minutes, it will pull the pin P0.25 low to turn off the transistor that connects the Vdd and Vbattery.


When the remote is not connected to iPhone, long press button will delete all the exisitng bondings. 

### :battery: Battery
The whole application runs on a small CR2032 coin cell battery. It is important to know when to change the battery, the BLE has battery info advertised to the iPhone, and at the same time, the red indicator LED will also be lit if the battery level is less than 20%. The way it detects the battery level is using the SAR ADC to sample the VDD every 2 seconds, and based on the discharge curve of CR2032 it will estimate the battery capacity based on the voltage, and again this is a quick hack, it is by no means very accurate. 
![low_battery1](/images/low%20battery%201.jpeg)

### :hammer: Toolcahin and how to build the project
This project uses the GCC Makefile setup from the Nordic NRF5 SDK 17.1.0 example project. 

I have been a fan of the VS code lightweight development enviroment, so I have setup all the build, debug and flash task within the VS code. All these configuration has been committed into the repo in the /application/.vscode/ folder. It might need some customization based on your setup. That being said you are more than welcome to use the make CLI to build and flash the SoC. 

I used the J-Link wtih 4 pin SWD interface to flash the SoC, but any programmer with ARM SWD interface should work.