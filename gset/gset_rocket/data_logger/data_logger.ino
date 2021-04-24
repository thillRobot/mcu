
/*
  GSET - Data Acquisition Rocket - Summer 2021 - Tennessee Technological University
  Tristan Hill - April 20, 2021
  
  This code began as example code from the Arduino library
  The original example code 'DataLogger'  was modified for this project
  created  24 Nov 2010
  modified 9 Apr 2012
  by Tom Igoe

  Example code from the Arduino library BNO055 was also used. 
  The example 'Bunny' has been modified for this project. 
  
  Modified by Tristan Hill - 06/24/2019 
  To use this with the mega you have to use the wiring shown below
  
  Revised ny Tristan Hill - 04/20/2021
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

  
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SPI.h>
#include <SD.h>

//const int chipSelect = 53; // used on MEGA
const int chipSelect = 7; // used on MKR , not setting this  can cause the SD to write to ALMOST work
int entry_number = 0;
int file_number = 0;
String file_string;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(11520);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  //Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetails();

  file_string="datalog";
  file_string+= String(file_number);
  file_string+=".txt";
  

  
}

void loop() {

  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  /* Get a new sensor event */
  //sensors_event_t event;
  //bno.getEvent(&event);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);
  /*
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);
  */
  
  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
  
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem

  // make a string for assembling the data to log:
  String dataString = "Data Log Entry Number:";
  dataString += String(entry_number);
  
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    dataString += "Accl:";
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    dataString += "Orient:";
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
    dataString += "Mag:";
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    dataString += "Gyro:";
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    dataString += "Rot:";
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    dataString += "Linear:";
  }
  else {
    Serial.print("Unk:");
    dataString += "Unk:";
  }
  

  dataString += String(x);
  dataString += ",";
  dataString += String(y);
  dataString += ",";
  dataString += String(z);
  dataString += ",";
  

  /*
  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
  */
  
  //dataString="Test";

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(file_string, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog file:");
    Serial.println(file_string);
  }

  entry_number++;
}
