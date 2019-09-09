/******************************************************************************
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Bruno Tibério ( bruno.tiberio@tecnico.ulisboa.pt )
 * Copyrigth (C) 2019 F. Ferreira da Silva (francisco.ferreira.silva@tecnico.ulisboa.pt)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ------------------------------------------------------------------------------
 * Below is the original comments from where the code was adapted
 * ------------------------------------------------------------------------------
 * SparkFun 9DoF Razor M0 Example Firmware
 * Jim Lindblom @ SparkFun Electronics
 * Original creation date: November 22, 2016
 * https://github.com/sparkfun/9DOF_Razor_IMU/Firmware
 *
 * This example firmware for the SparkFun 9DoF Razor IMU M0
 * demonstrates how to grab accelerometer, gyroscope, magnetometer,
 * and quaternion values from the MPU-9250's digital motion processor
 * (DMP). It prints those values to a serial port and, if a card is
 * present, an SD card.
 *
 * Values printed can be configured using the serial port. Settings
 * can be modified using the included "config.h" file.
 *
 * Resources:
 * SparkFun MPU9250-DMP Arduino Library:
 *   https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library
 * FlashStorage Arduino Library
 *   https://github.com/cmaglie/FlashStorage
 *
 * Development environment specifics:
 *   Firmware developed using Arduino IDE 1.6.12
 *
 * Hardware:
 *   SparkFun 9DoF Razor IMU M0 (SEN-14001)
 *   https://www.sparkfun.com/products/14001
 *
 ******************************************************************************/

#include <Arduino.h>
// Razor class
#include "src/Razor.h"
// config.h manages default logging parameters and can be used
// to adjust specific parameters of the IMU
#include "src/config.h"

#include "src/MadgwickAHRS.h"

// SD Library manages file and hardware control
#ifdef ENABLE_SD_LOGGING
#include <SD.h>
#endif

// Flash storage (for nv storage on ATSAMD21)
#ifdef ENABLE_NVRAM_STORAGE
#include <FlashStorage.h>
#endif

/////////////////////////////
// Logging Control Globals //
/////////////////////////////
// Defaults all set in config.h
bool enableSerialLogging = ENABLE_UART_LOGGING;
bool enableTimeLog = ENABLE_TIME_LOG;
bool enableCalculatedValues = ENABLE_CALCULATED_LOG;
bool enableAccel = ENABLE_ACCEL_LOG;
bool enableGyro = ENABLE_GYRO_LOG;
bool enableCompass = ENABLE_MAG_LOG;
bool enableQuat = ENABLE_QUAT_LOG;
bool enableEuler = ENABLE_EULER_LOG;
bool enableHeading = ENABLE_HEADING_LOG;
unsigned short accelFSR = IMU_ACCEL_FSR;
unsigned short gyroFSR = IMU_GYRO_FSR;
unsigned short fifoRate = DMP_SAMPLE_RATE;

void initHardware(void);
bool initIMU(void);
void printIMUData(void);

///////////////////////
// LED Blink Control //
///////////////////////
//bool ledState = false;
uint32_t lastBlink = 0;
void blinkLED()
{
  static bool ledState = false;
  digitalWrite(HW_LED_PIN, ledState);
  ledState = !ledState;
}

#ifdef ENABLE_NVRAM_STORAGE
///////////////////////////
// Flash Storage Globals //
///////////////////////////
// Logging parameters are all stored in non-volatile memory.
// They should maintain the previously set config value.
FlashStorage(flashEnableSDLogging, bool);
FlashStorage(flashFirstRun, bool);
FlashStorage(flashEnableSD, bool);
FlashStorage(flashEnableSerialLogging, bool);
FlashStorage(flashenableTime, bool);
FlashStorage(flashEnableCalculatedValues, bool);
FlashStorage(flashEnableAccel, bool);
FlashStorage(flashEnableGyro, bool);
FlashStorage(flashEnableCompass, bool);
FlashStorage(flashEnableQuat, bool);
FlashStorage(flashEnableEuler, bool);
FlashStorage(flashEnableHeading, bool);
FlashStorage(flashAccelFSR, unsigned short);
FlashStorage(flashGyroFSR, unsigned short);
FlashStorage(flashLogRate, unsigned short);
#endif

Razor imu; // Create an instance of the MPU9250_DMP class
Madgwick filter;

void setup()
{
  // Initialize LED, interrupt input, and serial port.
  // LED defaults to off:
  initHardware();

  // Initialize the MPU-9250. Should return true on success:
  if (!initIMU())
  {
    LOG_PORT.println("Error connecting to MPU-9250");
    while (1)
      ; // Loop forever if we fail to connect
    // LED will remain off in this state.
  }
  filter.begin(100.0f);
  LOG_PORT.println(sizeof(struct dataFrame));
}

void loop()
{

  // Then check IMU for new data, and log it
  if (!imu.fifoAvailable()) // If no new data is available
    return;                 // return to the top of the loop
  // Read from the digital motion processor's FIFO
  if (imu.dmpUpdateFifo() != INV_SUCCESS)
    return; // If that fails (uh, oh), return to top

  // If enabled, read from the compass.
  if (imu.updateCompass() != INV_SUCCESS)
    return; // If compass read fails (uh, oh) return to top
  imu.compensate_sensor_errors();
  filter.updateIMU(imu.output_packet.gyro[0], imu.output_packet.gyro[1], imu.output_packet.gyro[2], imu.output_packet.acc[0], imu.output_packet.acc[1], imu.output_packet.acc[2]);
  //filter.update(imu.output_packet.gyro[0], imu.output_packet.gyro[1], imu.output_packet.gyro[2], imu.output_packet.acc[0], imu.output_packet.acc[1], imu.output_packet.acc[2], imu.output_packet.mag[0], imu.output_packet.mag[1], imu.output_packet.mag[2] );
  imu.output_packet.euler[0] = filter.getRoll();
  imu.output_packet.euler[1] = filter.getPitch();
  imu.output_packet.euler[2] = filter.getYaw();
  //imu.printPacket();
  imu.sendPacket();
  //LOG_PORT.println(sizeof(struct dataFrame));
  //LOG_PORT.println(sizeof(float));
  //LOG_PORT.println(sizeof(uint8_t));
  //LOG_PORT.println(imu.packetSize);
  //LOG_PORT.println(sizeof(imu.output_packet.acc));
}

void printIMUData(void)
{
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);

  LOG_PORT.println("Accel: " + String(accelX) + ", " +
                   String(accelY) + ", " + String(accelZ) + " g");
  LOG_PORT.println("Gyro: " + String(gyroX) + ", " +
                   String(gyroY) + ", " + String(gyroZ) + " dps");
  LOG_PORT.println("Mag: " + String(magX) + ", " +
                   String(magY) + ", " + String(magZ) + " uT");
  LOG_PORT.println("Time: " + String(imu.time) + " ms");
  LOG_PORT.println();
}

void initHardware(void)
{
  // Set up LED pin (active-high, default to off)
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);

  // Set up MPU-9250 interrupt input (active-low)
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);

  // Set up serial log port
  LOG_PORT.begin(SERIAL_BAUD_RATE);
}

bool initIMU(void)
{
  // imu.begin() should return 0 on success. Will initialize
  // I2C bus, and reset MPU-9250 to defaults.
  if (imu.begin() != INV_SUCCESS)
    return false;

  // Set up MPU-9250 interrupt:
  imu.enableInterrupt(); // Enable interrupt output
  imu.setIntLevel(1);    // Set interrupt to active-low
  imu.setIntLatched(1);  // Latch interrupt output

  // Configure sensors:
  // Set gyro full-scale range: options are 250, 500, 1000, or 2000:
  imu.setGyroFSR(gyroFSR);
  // Set accel full-scale range: options are 2, 4, 8, or 16 g
  imu.setAccelFSR(accelFSR);
  // Set gyro/accel LPF: options are5, 10, 20, 42, 98, 188 Hz
  imu.setLPF(IMU_AG_LPF);
  // Set gyro/accel sample rate: must be between 4-1000Hz
  // (note: this value will be overridden by the DMP sample rate)
  imu.setSampleRate(IMU_AG_SAMPLE_RATE);
  // Set compass sample rate: between 4-100Hz
  imu.setCompassSampleRate(IMU_COMPASS_SAMPLE_RATE);

  // Configure digital motion processor. Use the FIFO to get
  // data from the DMP.
  unsigned short dmpFeatureMask = 0;
  if (ENABLE_GYRO_CALIBRATION)
  {
    // Gyro calibration re-calibrates the gyro after a set amount
    // of no motion detected
    dmpFeatureMask |= DMP_FEATURE_GYRO_CAL;
    dmpFeatureMask |= DMP_FEATURE_SEND_CAL_GYRO;
  }
  else
  {
    // Otherwise add raw gyro readings to the DMP
    dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
  }
  // Add accel and quaternion's to the DMP
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
  dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;

  // Initialize the DMP, and set the FIFO's update rate:
  imu.dmpBegin(dmpFeatureMask, fifoRate);

  return true; // Return success
}
