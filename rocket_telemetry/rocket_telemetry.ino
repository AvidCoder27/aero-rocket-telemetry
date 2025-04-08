#include <Adafruit_ICM20649.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

/*
  SD card datalogger
  SD card attached to SPI bus as follows:
 ** SDO - pin 11
 ** SDI - pin 12
 ** CLK - pin 13
 ** CS - pin 10
*/

const int sdChipSelect = 10;

Adafruit_ICM20649 icm; // imu
Adafruit_Sensor *icm_accel, *icm_gyro;

void setup() {
  // Open serial communications
  Serial.begin(9600);
  // wait for Serial Monitor
  while (!Serial);
  
  // open i2c comms with imu
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20649 chip");
    while (true) {
      delay(10);
    }
  }

  Serial.println("ICM20649 found");
  
  icm_accel = icm.getAccelerometerSensor();
  icm_accel->printSensorDetails();

  icm_gyro = icm.getGyroSensor();
  icm_gyro->printSensorDetails();

  Serial.print("Initializing SD card...");

  if (!SD.begin(sdChipSelect)) {
    Serial.println("initialization failed.");
    // Things to check:
    // 1. is a card inserted?
    // 2. is your wiring correct?
    // 3. did you change the chipSelect pin to match your shield or module?
    // Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!
    while (true) {
      delay(10);
    }
  }

  Serial.println("initialization complete");
}

void loop() {
  // make a string for assembling the data to log
  // [time ms],[accel x],[accel y],[accel z],[gyro x],[gyro y],[gyro z]
  String dataString = String(millis());

  // get accel data
  sensors_event_t accel;
  sensors_event_t gyro;
  icm_accel->getEvent(&accel);
  icm_gyro->getEvent(&gyro);
  // accel in m/s^2
  dataString += ',' + accel.acceleration.x + ',' + accel.acceleration.y + ',' + accel.acceleration.z;
  // gyro in rad/sec
  dataString += ',' + gyro.gyro.x + ',' + gyro.gyro.y + ',' + gyro.gyro.z; 

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.print("DataString: ");
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("!!!!error opening datalog.txt!!!!");
  }
}
