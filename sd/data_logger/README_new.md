# GSET - Data Acquisition Rocket
  Summer 2021 - Tennessee Technological University
  Tristan Hill - April 24, 2021

## History  
  - This code began as example code from the Arduino library
  created 24 Nov 2010, modified 9 Apr 2012 by Tom Igoe
  - The example code 'DataLogger' was modified for this project
  - Example code from the Arduino library BNO055 was also used. 
  - The example 'Bunny' has been modified for this project. 
  - Modified by Tristan Hill - 06/24/2019 
  - Revised by Tristan Hill - 04/20/2021
  - switched to the MKR1010 Wifi board - 04/20/2021
  - tested BNO055 - working - values look good, not validated - 04/20/2021
  - test SD Card Breakout - working - data in files look good, not validated - 04/20/2021
  - added all the sensors as shown in the example code "read_all_data"
  - added and formatted README.md - 04/29/2021
  - added entry number and file number to data log format on SD card - 04/29/2021 
  
## Hardware Information
  
### Available Hardware
  - Arduino Nano 3.0 328p
  - BNO055 - Absolute Orientation Sensor
  - GY-521 (MPU 6050 MEMS) - 3 Axis Accelerometer, 3 Axis Gyroscope, and Temperature Sensor 
  - GY-68 (BMP180) - Barometric Pressure Sensor
  - DPS310 - Barometric Pressure, Temperature Sensor
  - 5v Ready SD Breakout Board + 16 GB micro SD card
  - Lipo Battery - 1s, 3.7v, 150 mAh 


### BNO055 - i2c Sensor Board 
   
  This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
  which provides a common 'type' for sensor data and some helper functions.
  
  To use this driver you will also need to download the Adafruit_Sensor
  library and include it in your libraries folder.
  
  You should also assign a unique ID to this sensor for use with
  the Adafruit Sensor API so that you can identify this particular
  sensor in any data logs, etc.  To assign a unique ID, simply
  provide an appropriate value in the constructor below (12345
  is used by default in this example).
  
#### BNO055 layout and Sensor Orientation
```  
       +----------+
       |         *| RST   PITCH  ROLL  HEADING
   ADR |*        *| SCL
   INT |*        *| SDA     ^            /->
   PS1 |*        *| GND     |            |
   PS0 |*        *| 3VO     Y    Z-->    \-X
       |         *| VIN
       +----------+
``` 
#### Conections to MKR1010wifi     
```
  MKR1010 | BNO055 Sensor Board
  
  SDA (11)-----SDA
  SCL (12)-----SCL
  
  5v-----------Vin
            (nc)3v 
  Gnd----------Gnd     
```

### 5v Ready Micro-SD Card Breakout Board (from Adafruit.com) 

#### Connections to MKR1010wifi 
```
    MKR1010 | MicroSD Breakout
    
    GPIO(7)-----CS (set this pin in software)
    MOSI(8)-----D1
    SCK(9)-----CLK
    MISO(10-----D0 
    
    5v----------5v
            (nc)3v 
    Gnd--------Gnd
```
#### Connections to MEGA2560
```
  MEGA | MicroSD Breakout
  
  SS(53)-------CS (set this pin in software?)
  SCK(52)-----CLK 
  MOSI(51)-----D1
  MISO(50)-----D0
  
  5v-----------5v
           (nc)3v 
  Gnd---------Gnd
``` 
  
