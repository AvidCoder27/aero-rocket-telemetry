#include <Adafruit_BMP280.h>
#include <Adafruit_ICM20649.h>
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

Adafruit_ICM20649 imu; // imu
Adafruit_Sensor *imu_accel, *imu_gyro;

Adafruit_BMP280 bmp; // barometric pressure
Adafruit_Sensor *bmp_pressure;

File log_file;

inline void waitForever() {
  while(1) {
    delay(10);
  }
}

inline void sd_init() {
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

inline void imu_init() {
  Serial.print("Initializing IMU... ");

  // open i2c comms with imu
  if (!imu.begin_I2C()) {
    Serial.println("Failed to find IMU");
    waitForever();
  }

  imu_accel = imu.getAccelerometerSensor();
  imu_accel->printSensorDetails();

  imu_gyro = imu.getGyroSensor();
  imu_gyro->printSensorDetails();

  Serial.println("IMU success!");
}

inline void bmp_init() {
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

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_pressure = bmp.getPressureSensor();
  bmp_pressure->printSensorDetails();

  Serial.println("BMP success!");
}

void open_log_file() {
  Serial.print("Opening log file... ");
  log_file = SD.open("datalog.txt", FILE_WRITE);
  if (!log_file) {
    Serial.println("Failed to open datalog.txt");
  }
  Serial.println("Log file opened!");
}

void setup() {
  // Open serial communications
  Serial.begin(9600);
  // wait for Serial Monitor
  while (!Serial);

  sd_init();
  open_log_file();
  imu_init();
  bmp_init();

  Serial.println("Initialization complete!");
}

void loop() {
  // get sensor data
  sensors_event_t accel_evt, gyro_evt, pressure_evt;
  bmp_pressure->getEvent(&pressure_evt);
  imu_accel->getEvent(&accel_evt);
  imu_gyro->getEvent(&gyro_evt);
  
  // make a string for assembling the data to log
  // [time],[accel x],[accel y],[accel z],[gyro x],[gyro y],[gyro z],[pressure]

  // time in milliseconds
  // accel in m/s^2
  // gyro in rad/sec
  // pressure in hPa
  String dataString = "";
  dataString += millis() + ',' + accel_evt.acceleration.x + ',' + accel_evt.acceleration.y + ',' + accel_evt.acceleration.z + ',' + gyro_evt.gyro.x + ',' + gyro_evt.gyro.y + ',' + gyro_evt.gyro.z + ',' + pressure_evt.pressure;

  // if the file is available, write to it
  if (log_file) {
    log_file.println(dataString);    
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error
  else {
    Serial.println("ERROR opening datalog.txt !!!!");
  }
}