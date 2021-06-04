/****************************************************************************************/
/*  data_logger - Tennessee Technological University                                    */
/*  Tristan Hill - May 31, 2021                                                         */
/*  Write random test data to a csv file on an SD card                                  */    
/*                                                                                      */                         
/*  see README.md for version history and hardware information                          */          
/****************************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SPI.h>
#include <SD.h>

// Set the delay between fresh samples 
//#define BNO055_SAMPLERATE_DELAY_MS (100)
#define LOOP_DELAY_MS (100)

const int chip_select = 10; // 10 for nano, 7 used on MKR, not setting this can cause the SD to write to ALMOST work
int entry_number = 0;     // number of the first row in the data file 
int file_number = 0;      // change this number to create a new file
String file_string;       // global variables - should this be done differently?
bool delete_file = true;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)                                  
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // (id, address)

/*************************************************************/
/*  setup function 'setup'                                   */
/*************************************************************/
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(38400);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  initFile();

  //Serial.begin(115200);
  Serial.println("Data Logger Test"); Serial.println("");

  ///* Initialize the sensor */
  //if(!bno.begin())
  //{
  //  /* There was a problem detecting the BNO055 ... check your connections */
  //  Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //  while(1);
  //}
   
  delay(1000);

  ///* Use external crystal for better accuracy */
  //bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  //displaySensorDetails();

}

/*************************************************************/
/*  main function 'loop'                                     */
/*************************************************************/
void loop() {

  printData();

  delay(LOOP_DELAY_MS);
  
}

/*************************************************************/
/*  this function 'initFile' opens a file for the datalog     */
/*************************************************************/
void initFile(void)
{

  Serial.println("Checking for SD Card...");
  // check if the card is present and can be initialized
  if (!SD.begin(chip_select)) {
    Serial.println("SD card failed or not present");
    while (1); // wait forever if card fails?
  }
  Serial.println("Card Initialized");

  file_string="datalog"+String(file_number)+".txt";  // global variable for now

  // check to see if the file already exists on the SD card
  if (SD.exists(file_string)&&delete_file)
  {
    Serial.println(file_string+" already exist, deleting file before writing data");
    SD.remove(file_string);
  }else if(SD.exists(file_string))
  {
    Serial.println(file_string+" already exist, data will be appended to file");
  }else
  {
    Serial.println(file_string+" does not exist, a new file will be created");
  } 
  //Serial.println(buffer);

  //instantiate a string for assembling the data file header
  String buffer= "Data Logger Test Filename: "+ file_string + "\r\n";

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File file_id = SD.open(file_string, FILE_WRITE);

  // if the file is available, write the string to it:
  if (file_id) {
    file_id.println(buffer);
    file_id.close(); //close the file before opening another.
  }
  // if the file did not open, change the header message to an error:
  else {
    Serial.println("Error opening file: "+file_string);
  }

  // write the string to the serial output for debugging
  Serial.println(buffer);

  // seed the random number generator with an a2d reading
  randomSeed(analogRead(0));

}

/**********************************************************/
/*  Formats and writes the data entry to the file         */
/**********************************************************/
bool printData(void) {
  
  int rnum;
  rnum = random(1000);
  
  String buffer= "Random Number: ";
  buffer += rnum;

  File file_id = SD.open(file_string, FILE_WRITE);

  // if the file is available, write the string to it:
  if (file_id) {
    file_id.println(buffer);
    file_id.close(); //close the file before opening another.
  }
  // if the file did not open, change the header message to an error:
  else {
    Serial.println("Error opening file: "+file_string);
  }

  // write the string to the serial output for debugging
  Serial.println(buffer);

  return true;
}


