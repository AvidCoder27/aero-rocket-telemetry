#include <Adafruit_BMP280.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_ICM20X.h>
#include <SD.h>
#include <Wire.h>

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

#define SD_CHIP_SELECT 10

#ifdef DEBUG
const char* str_init = "init";
const char* str_imu = "IMU";
const char* str_bmp = "BMP";
const char* str_sd = "SD";
const char* BAD = "bad!";
const char* GOOD = "good";
const char* DOTS = "...";
const char SPACE = ' ';
#endif
const char* FILE_NAME = "DATALOG.TXT";
const char COMMA = ',';

File log_file;

Adafruit_ICM20649 imu; // imu
Adafruit_Sensor *imu_accel, *imu_gyro;

Adafruit_BMP280 bmp(&Wire); // barometric pressure
Adafruit_Sensor *bmp_pressure, *bmp_temperature;

void init_imu() {
  #ifdef DEBUG
  Serial.print(str_init);
  Serial.print(SPACE);
  Serial.print(str_imu);
  Serial.print(DOTS);
  #endif

  // open i2c comms with imu
  if (!imu.begin_I2C()) {
    #ifdef DEBUG
    Serial.println(BAD);
    #endif
    waitForever();
  }

  imu_accel = imu.getAccelerometerSensor();
  imu_gyro = imu.getGyroSensor();
  #ifdef DEBUG
  Serial.println(GOOD);
  #endif
}

void init_bmp() {
  #ifdef DEBUG
  Serial.print(str_init);
  Serial.print(SPACE);
  Serial.print(str_bmp);
  Serial.print(DOTS);
  #endif

  if (!bmp.begin()) {
    #ifdef DEBUG
    Serial.print(BAD);
    Serial.print(SPACE);
    Serial.println(bmp.sensorID(),16);
    #endif
    waitForever();
    // ID of 0xFF probably means a bad address, a BMP 180 or BMP 085
    // ID of 0x56-0x58 represents a BMP 280
    // ID of 0x60 represents a BME 280
    // ID of 0x61 represents a BME 680
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */

  bmp_pressure = bmp.getPressureSensor();
  bmp_temperature = bmp.getTemperatureSensor();
  #ifdef DEBUG
  Serial.println(GOOD);
  #endif
}

void waitForever() {
  while(1) {
    #ifdef DEBUG
    Serial.println(BAD);
    #endif
    delay(500);
  }
}

void sd_init() {
  #ifdef DEBUG
  Serial.print(str_init);
  Serial.print(SPACE);
  Serial.print(str_sd);
  Serial.print(DOTS);
  #endif
  if (!SD.begin(SD_CHIP_SELECT)) {
    #ifdef DEBUG
    Serial.println(BAD);
    #endif
    // Things to check:
    // 1. is a card inserted?
    // 2. is your wiring correct?
    // 3. did you change the chipSelect pin to match your shield or module?
    // Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!
    waitForever();
  }
  #ifdef DEBUG
  Serial.println(GOOD);
  #endif
}

void open_log_file() {
  #ifdef DEBUG
  Serial.print(str_init);
  Serial.print(SPACE);
  Serial.print(FILE_NAME);
  Serial.print(DOTS);
  
  #endif
  log_file = SD.open(FILE_NAME, FILE_WRITE);
  if (!log_file) {
    #ifdef DEBUG
    Serial.println(BAD);
    #endif
    waitForever();
  }
  #ifdef DEBUG
  Serial.println(GOOD);
  #endif
  log_file.close();
}

void setup() {
  #ifdef DEBUG
  // Open serial communications
  Serial.begin(9600);
  // wait for Serial Monitor
  while (!Serial) delay(10);
  Serial.println(str_init);
  #endif

  sd_init();
  open_log_file();
  init_imu();
  init_bmp();

  #ifdef DEBUG
  Serial.print(str_init);
  Serial.print(SPACE);
  Serial.println(GOOD);
  #endif
}

void loop() {
  // get sensor data
  sensors_event_t accel_evt, gyro_evt, pressure_evt, temp_evt;
  bmp_temperature->getEvent(&temp_evt);
  bmp_pressure->getEvent(&pressure_evt);
  imu_accel->getEvent(&accel_evt);
  imu_gyro->getEvent(&gyro_evt);
  
  // make a string for assembling the data to log
  // [time],[accel x],[accel y],[accel z],[gyro x],[gyro y],[gyro z]

  // time in milliseconds
  // accel in m/s^2
  // gyro in rad/sec
  // pressure in hPa
  // altitude in meters

  log_file = SD.open(FILE_NAME, FILE_WRITE);
  // if the file is available, write to it
  if (log_file) {
    log_file.print(millis());
    log_file.print(COMMA);
    log_file.print(accel_evt.acceleration.x);
    log_file.print(COMMA);
    log_file.print(accel_evt.acceleration.y);
    log_file.print(COMMA);
    log_file.print(accel_evt.acceleration.z);
    log_file.print(COMMA);
    log_file.print(gyro_evt.gyro.x);
    log_file.print(COMMA);
    log_file.print(gyro_evt.gyro.y);
    log_file.print(COMMA);
    log_file.print(gyro_evt.gyro.z);
    log_file.print(COMMA);
    log_file.print(pressure_evt.pressure);
    log_file.print(COMMA);
    log_file.print(temp_evt.temperature);
    log_file.println();
    log_file.close();
  }
  // if the file isn't open, pop up an error
  else {
    #ifdef DEBUG
    Serial.print(BAD);
    Serial.print(SPACE);
    Serial.println(FILE_NAME);
    #endif
  }
}