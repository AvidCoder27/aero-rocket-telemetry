#include <Adafruit_BMP280.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_ICM20X.h>
#include <SD.h>

// Defining DEBUG shuts of serial monitor
// #define DEBUG // comment out for release

/*
  SD card attached to SPI bus as follows:
  ** SDO - pin 11
  ** SDI - pin 12
  ** CLK - pin 13 (SCK)
  ** CS  - pin 10

  IMU pinout: (https://learn.adafruit.com/adafruit-icm20649-wide-range-6-dof-imu-accelerometer-and-gyro/pinouts)
  default i2c address at 0x68 (adafruit lib handles it)
  ** SCL - i2c clock pin (has pull-up)
  ** SDA - i2c data pin (has pull-up)
  ** AD0 - pull high to switch address to 0x69

  BMP pinout: (https://learn.adafruit.com/adafruit-bmp280-barometric-pressure-plus-temperature-sensor-breakout/pinouts)
  default i2c address at 0x77 (adafruit lib handles it)
  ** SCK/SCL - i2c clock pin, id 
  ** SDI/SDA - i2c data pin

  SPI Clock is pin 13

  I2C clock is pin A5 (D19).
  Both i2c devices connect to that on their clock pin.
  I2C data is SDA on pin A4 (D18).
  Both i2c devices connect to that on their data pin.
*/

const int SD_CHIP_SELECT = 10;

Adafruit_ICM20649 imu; // imu
Adafruit_Sensor *imu_accel, *imu_gyro;

File log_file;

void waitForever() {
  while(1) {
    #ifdef DEBUG
    Serial.println("Error!");
    #endif
    delay(500);
  }
}

void sd_init() {
  #ifdef DEBUG
  Serial.print("Initializing SD card... ");
  #endif
  if (!SD.begin(SD_CHIP_SELECT)) {
    #ifdef DEBUG
    Serial.println("SD init failed");
    #endif
    // Things to check:
    // 1. is a card inserted?
    // 2. is your wiring correct?
    // 3. did you change the chipSelect pin to match your shield or module?
    // Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!
    waitForever();
  }
  #ifdef DEBUG
  Serial.println("SD success!");
  #endif
}

void open_log_file() {
  #ifdef DEBUG
  Serial.print("Opening log file... ");
  #endif
  log_file = SD.open("datalog.txt", FILE_WRITE);
  if (!log_file) {
    #ifdef DEBUG
    Serial.println("Failed to open datalog.txt");
    #endif
    waitForever();
  }
  #ifdef DEBUG
  Serial.println("Log file opened!");
  #endif
}

void init_imu() {
  #ifdef DEBUG
  Serial.print("Initializing IMU... ");
  #endif

  // open i2c comms with imu
  if (!imu.begin_I2C()) {
    #ifdef DEBUG
    Serial.println("Failed to find IMU");
    #endif
    waitForever();
  }

  imu_accel = imu.getAccelerometerSensor();
  imu_gyro = imu.getGyroSensor();
  #ifdef DEBUG
  Serial.println("IMU success!");
  #endif
}

void setup() {
  #ifdef DEBUG
  // Open serial communications
  Serial.begin(9600);
  // wait for Serial Monitor
  while (!Serial) delay(10);
  Serial.println("Started serial communications");
  #endif

  init_imu();
  sd_init();
  open_log_file();

  #ifdef DEBUG
  Serial.println("Initialization complete!");
  #endif
}

void loop() {  
  // get sensor data
  sensors_event_t accel_evt, gyro_evt;
  imu_accel->getEvent(&accel_evt);
  imu_gyro->getEvent(&gyro_evt);
  
  // make a string for assembling the data to log
  // [time],[accel x],[accel y],[accel z],[gyro x],[gyro y],[gyro z]

  // time in milliseconds
  // accel in m/s^2
  // gyro in rad/sec
  // pressure in hPa
  // altitude in meters

  log_file = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it
  if (log_file) {
    log_file.print(millis());
    log_file.print(",");
    log_file.print(accel_evt.acceleration.x);
    log_file.print(",");
    log_file.print(accel_evt.acceleration.y);
    log_file.print(",");
    log_file.print(accel_evt.acceleration.z);
    log_file.print(",");
    log_file.print(gyro_evt.gyro.x);
    log_file.print(",");
    log_file.print(gyro_evt.gyro.y);
    log_file.print(",");
    log_file.println(gyro_evt.gyro.z);
    log_file.close();
  }
  // if the file isn't open, pop up an error
  else {
    #ifdef DEBUG
    Serial.println("ERROR opening datalog.txt");
    #endif
  }
}