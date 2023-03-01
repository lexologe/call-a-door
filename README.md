# Call-a-Door
Call your door release

## Purpose
This repository contains all software parts to create a firmware for the LILYGO T-CALL SIM800C AXP192 that toggles a GPIO on incoming calls to open a door release.

It should ...
* open door release on incoming calls
* run forever
* recover from failures without interaction
* reconnect to mobile network when connection is lost

## Requirements

Hardware:
* LILYGO T-Call SIM800C AXP192 module
* and parts that are used to connect to a Ritto Twinbus 7630 (see this blog https://www.nicht-trivial.de/index.php/2018/02/14/ritto-zu-mqtt/)
  * FISM Fixed Isolated Modul“ (Würth Elektronik 177920531 24 V 5 V 0.2 A 1 W)
  * CD4066
  * BC547B
  * 470uF capacitor
  * 1kOhm resistor
Software:
* heavily based on https://github.com/Xinyuan-LilyGO/LilyGo-T-Call-SIM800
* TinyGSM
* StreamDebugger
* and AXP202X_Library

## How to build

Install VSCode and add PlatformIO extention. Build and upload the firmware with VSCode. Monitor gives some additional information about the GSM status of the module. SIM card should be activated and should not have a PIN.

## Disclaimer

The code in this repository is not intended to raise any costs on your SIM provider. If it does, the author cannot take any responsibility for that.