# GSET - Data Acquisition Rocket
  Summer 2021 - Tennessee Technological University
  Tristan Hill - April 24, 2021
  
  This code began as example code from the Arduino library
  The original example code 'DataLogger'  was modified for this project
  created  24 Nov 2010
  modified 9 Apr 2012
  by Tom Igoe

  Example code from the Arduino library BNO055 was also used. 
  The example 'Bunny' has been modified for this project. 
  
  Modified by Tristan Hill - 06/24/2019 
  To use this with the mega you have to use the wiring shown below
  
  Revised by Tristan Hill - 04/20/2021
  This year we switched to the MKR1010 Wifi board

  BN0055 - working - values look good, not validated - 04/20/2021
  
  SD Card Breakout - working - data in files look good, not validated - 04/20/2021

  
  Caution:  This code is still a rough mashup of the two peices of example code, and much TLC is still need. 

  - I have added all the sensors as shown in the example code "read_all_data"
  

  ***** BNO055 - i2c Sensor Board - info *****
   
  This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
  which provides a common 'type' for sensor data and some helper functions.
  
  To use this driver you will also need to download the Adafruit_Sensor
  library and include it in your libraries folder.
  
  You should also assign a unique ID to this sensor for use with
  the Adafruit Sensor API so that you can identify this particular
  sensor in any data logs, etc.  To assign a unique ID, simply
  provide an appropriate value in the constructor below (12345
  is used by default in this example).
  
  Connections
  ===========
  Connect SCL to analog 5
  Connect SDA to analog 4
  Connect VDD to 3.3-5V DC
  Connect GROUND to common ground
  

  ***** SD Card Breakout Board - info ***** 

  The circuit:
  * SD card attached to SPI bus as follows:
  ** MOSI - pin 11
  ** MISO - pin 12
  ** CLK - pin 13
  ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

  ***** Wiring Diagram ***** 
  
  MEGA | MicroSD Breakout (not currently used)
  
  SS(53)-------CS
  SCK(52)-----CLK 
  MOSI(51)-----D1
  MISO(50)-----D0
  
  5v-----5v
    (nc)3v 
  Gnd---Gnd
  
  MKR1010 | MicroSD Breakout (currently used)
  
  GPIO(7)-----CS
  MOSI(8)-----D1
  SCK(9)-----CLK
  MISO(10-----D0 
  
  MISO(10)-----D0 (you choose this pin)
  
  5v-----5v
    (nc)3v 
  Gnd---Gnd
  
  MKR1010 | BNO055 Sensor Board
  
  SDA (11)-----SDA
  SCL (12)-----SCL
  
  5v----Vin
    (nc)3v 
  Gnd---Gnd
 
  BNO055 Board layout and Sensor Orientation
  
       +----------+
       |         *| RST   PITCH  ROLL  HEADING
   ADR |*        *| SCL
   INT |*        *| SDA     ^            /->
   PS1 |*        *| GND     |            |
   PS0 |*        *| 3VO     Y    Z-->    \-X
       |         *| VIN
       +----------+

  
