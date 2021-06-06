/****************************************************************************************/
/*  sensor/BNO055 - Tennessee Technological University                                  */
/*  Tristan Hill - 2021                                                                 */
/*  Access sensor data from BNO055 Absolute Orientation Module                          */    
/*  This example is based on the example Adafruit_BNO055/read_all_data                  */
/*  and it has been tested on the NANO328p (and the MKR1010)                            */      
/****************************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SPI.h>
#include <SD.h>

//Set the delay between samples 
#define LOOP_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)                                  
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // (id, address)

/*************************************************************/
/*  setup function 'setup'                                   */
/*************************************************************/
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(38400);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //Serial.begin(115200);
  Serial.println("BNO055 Test \n Initializing... "); Serial.println("");

  /* Initialize the sensor */
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

}

/*************************************************************/
/*  main function 'loop'                                     */
/*************************************************************/
void loop() {

  printData();

  delay(LOOP_DELAY_MS);
  
}

/**********************************************************/
/*  Formats and writes the data entry to the file         */
/**********************************************************/
bool printData(void) {
  
  // print the entry number and calibration data before printing sensor data

  // instanstiate objects for different sensor types 
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  // print the bulk of the data to the file
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
  double x = -1000000, y = -1000000 , z = -1000000; //easy to spot dummy values

  // instantiate a string for assembling the data log
  String buffer;

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    buffer += "Accelerometer:";
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    buffer += "Orientation:";
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
    buffer += "MagneticField:";
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    buffer += "SENSOR_TYPE_GYROSCOPE:";
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    buffer += "RotationVector:";
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    buffer += "LinearAcceleration:";
  }
  else {
    Serial.print("Unknown:");
    buffer += "Unknown:";
  }
  
  // append the data feild to the string separated by commas
  buffer += String(x)+","+String(y)+","+String(z);

  // write the string to the serial output for debugging
  Serial.println(buffer);

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