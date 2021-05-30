# Using Arduino Command Line Interface, arduino-cli

This is a basic guide to using `arduino-cli` which follows the getting started page [here](https://arduino.github.io/arduino-cli/latest/getting-started/)
Tristan Hill - May 29th, 2021


This example uses the Arduino Nano 3.0 328p - Old Bootloader - `arduino:avr:nano:cpu=atmega328old`

# Install `arduino-cli`

## Install the `core` for the board  you are going to use.

## Insert USB and check for a connected board:
```
arduino-cli board list

Port         Type              Board Name FQBN Core
/dev/ttyS0   Serial Port       Unknown             
/dev/ttyUSB0 Serial Port (USB) Unknown 
```

## Compile the Source Code (.ino):
```
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old arduino_cli_helloworld.ino

```

## Upload the Compiled Code to the Board:
```
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328old arduino_cli_helloworld.ino

```
## Configure Minicom to listen to serial connection
```
sudo minicom -s
```
Use the menus to configure the serial connection to match the sample code. `USB0` refers to a custum configuration made in minicom. 

## Test the code by listening to the serial connection:
```
minicom USB0      

Welcome to minicom 2.7.1

OPTIONS: I18n 
Compiled on Aug 13 2017, 15:25:34.
Port /dev/ttyUSB0, 23:22:11

Press CTRL-A Z for help on special keys

Hello World from Arduino-CLI!
Hello World from Arduino-CLI!
Hello World from Arduino-CLI!
Hello World from Arduino-CLI!
Hello World from Arduino-CLI!
Hello World from Arduino-CLI!
Hello World from Arduino-CLI!
.
.
.
```

