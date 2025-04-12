#include <Adafruit_BMP280.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_ICM20X.h>
#include <SD.h>

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

Adafruit_BMP280 bmp; // barometric pressure
Adafruit_Sensor *bmp_pressure;

Adafruit_ICM20649 imu; // imu
Adafruit_Sensor *imu_accel, *imu_gyro;

File log_file;

void waitForever() {
  while(1) {
    delay(10);
  }
}

void sd_init() {
  Serial.print("Initializing SD card... ");
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("SD init failed");
    // Things to check:
    // 1. is a card inserted?
    // 2. is your wiring correct?
    // 3. did you change the chipSelect pin to match your shield or module?
    // Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!
    waitForever();
  }
  Serial.println("SD success!");
}

void open_log_file() {
  Serial.print("Opening log file... ");
  log_file = SD.open("datalog.txt", FILE_WRITE);
  if (!log_file) {
    Serial.println("Failed to open datalog.txt");
  }
  Serial.println("Log file opened!");
}

void init_imu() {
  Serial.print("Initializing IMU... ");

  // open i2c comms with imu
  if (!imu.begin_I2C()) {
    Serial.println("Failed to find IMU");
    waitForever();
  }

  imu_accel = imu.getAccelerometerSensor();
  imu_gyro = imu.getGyroSensor();
  Serial.println("IMU success!");
}

void init_bmp() {
  Serial.print("Initializing BMP... ");
  if (!bmp.begin()) {
    Serial.print("Failed to find BMP, SensorID was: 0x");
    Serial.println(bmp.sensorID(),16);
    waitForever();
    // ID of 0xFF probably means a bad address, a BMP 180 or BMP 085
    // ID of 0x56-0x58 represents a BMP 280
    // ID of 0x60 represents a BME 280
    // ID of 0x61 represents a BME 680
  }

  // bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  //               Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  //               Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  //               Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  //               Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_pressure = bmp.getPressureSensor();
  Serial.println("BMP success!");
}

void setup() {
  // Open serial communications
  Serial.begin(9600);
  // wait for Serial Monitor
  while (!Serial) delay(10);
  Serial.println("Started serial communications");

  Wire.end();
  delay(10);
  Wire.begin();
  delay(10);

  delay(100);
  // init_bmp();
  delay(100);
  init_imu();
  delay(100);

  sd_init();
  open_log_file();

  Serial.println("Initialization complete!");
}

void loop() {  
  return;
  // get sensor data
  sensors_event_t accel_evt, gyro_evt, pressure_evt;
  bmp_pressure->getEvent(&pressure_evt);
  imu_accel->getEvent(&accel_evt);
  imu_gyro->getEvent(&gyro_evt);
  
  // make a string for assembling the data to log
  // [time],[accel x],[accel y],[accel z],[gyro x],[gyro y],[gyro z],[altitude by pressure]

  // time in milliseconds
  // accel in m/s^2
  // gyro in rad/sec
  // pressure in hPa
  // altitude in meters

  Serial.print(millis());
  Serial.print(",");
  Serial.print(accel_evt.acceleration.x);
  Serial.print(",");
  Serial.print(accel_evt.acceleration.y);
  Serial.print(",");
  Serial.print(accel_evt.acceleration.z);
  Serial.print(",");
  Serial.print(gyro_evt.gyro.x);
  Serial.print(",");
  Serial.print(gyro_evt.gyro.y);
  Serial.print(",");
  Serial.print(gyro_evt.gyro.z);
  Serial.print(",");
  Serial.println(pressure_evt.pressure);

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
    log_file.print(gyro_evt.gyro.z);
    log_file.print(",");
    log_file.println(pressure_evt.pressure);
  }
  // if the file isn't open, pop up an error
  else {
    Serial.println("ERROR opening datalog.txt");
  }
}