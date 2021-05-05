/****************************************************************************************/
/*  GSET - Data Acquisition Rocket - Summer 2021 - Tennessee Technological University   */
/*  Tristan Hill - 2021                                                                 */
/*                                                                                      */
/*  see README.md for version history and hardware information                          */          
/****************************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SPI.h>
#include <SD.h>

const int chipSelect = 7; // used on MKR , not setting this can cause the SD to write to ALMOST work
int entry_number = 0;
int file_number = 3; // change this number to create a new file
String file_string;

// Set the delay between fresh samples 
//#define BNO055_SAMPLERATE_DELAY_MS (100)
#define LOOP_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/*************************************************************/
/*  setup function 'setup'                                   */
/*************************************************************/
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

  file_string="Datalog";
  file_string+= String(file_number);
  file_string+=".txt";
  
}

/*************************************************************/
/*  main function 'loop'                                     */
/*************************************************************/
void loop() {

  printHeader();

  printData();
  
  printFooter();

  delay(LOOP_DELAY_MS);
  
}

/*****************************************************************************/
/*  Formats and writes the data entry header and calibration to file         */
/*****************************************************************************/
bool printHeader() {
  // get the board temp for the data entry header
  int8_t boardTemp = bno.getTemp();
  uint8_t system, gyro, accel, mag = 0;

  // instantiate a string for assembling the data entry header
  //String dataString = "";
  String dataString = "DataLog Entry:"+String(entry_number)+"\nBNO055 Temp:"+String(boardTemp);

  bno.getCalibration(&system, &gyro, &accel, &mag);
  dataString += "\nCalibration: Sys= "+String(system)+", Gyro="+String(gyro)+", Accel="+String(accel)+", Mag="+String(mag);

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
  
  return true;
}

/**************************************************************************/
/*  Formats and writes the data entry footer to file                      */
/**************************************************************************/
bool printFooter(void) {
  
  String dataString = "DataLog Entry:"+String(entry_number)+": Complete";
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

  return true;
}

/*****************************************************************************/
/*  Formats and writes the data entry header and calibration to file         */
/*****************************************************************************/
bool printData(void) {
  // instanstiate objects for different sensor types 
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

  return true;
}

/*****************************************************************************/
/*  Formats and writes a single sensor event to file                         */
/*****************************************************************************/
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem

  // instantiate a string for assembling the data log
  String dataString = "";

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    dataString += "Accelerometer:";
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    dataString += "Orientation:";
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
    dataString += "MagneticField:";
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    dataString += "SENSOR_TYPE_GYROSCOPE:";
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    dataString += "RotationVector:";
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    dataString += "LinearAcceleration:";
  }
  else {
    Serial.print("Unknown:");
    dataString += "Unknown:";
  }
  
  // append the data feild to the string separated by commas
  dataString += String(x)+","+String(y)+","+String(z);

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

}

/**************************************************************************/
/*    Displays some basic information on this sensor from the unified     */
/*    sensor API sensor_t type (see Adafruit_Sensor for more information) */   
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