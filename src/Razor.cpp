/*
 * Razor.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: bruno
 */

#include "Razor.h"
#include <Arduino.h>

#define DEBUG 0

// char * sprintFloat(float val, char *accString) {
//   memset(accString, 0, 7 * sizeof(char));
//  return dtostrf(val, 6, 2, accString);
// }

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void matrix_Vector_Multiply(const float a[3][3], const float b[3],
                            float out[3])
{
  for (int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}

// ================= 8-BIT CRC ===================
//  code taken from https://github.com/FrankBoesing/FastCRC
/** Constructor
 */
FastCRC8::FastCRC8()
{
  seed = 0x00;
}
FastCRC8::~FastCRC8(){
    /// TODO Auto-generated destructor stub
};

/** MAXIM 8-Bit CRC
 * equivalent to _crc_ibutton_update() in crc16.h from avr_libc
 * @param data Pointer to Data
 * @param datalen Length of Data
 * @return CRC value
 */
uint8_t FastCRC8::maxim_upd(const uint8_t *data, uint16_t datalen)
{
  uint8_t crc = seed;
  if (datalen)
    do
    {
      crc = crc_table_maxim[crc ^ *data];
      data++;
    } while (--datalen);
  seed = crc;
  return crc;
}
uint8_t FastCRC8::maxim(const uint8_t *data, const uint16_t datalen)
{
  // poly=0x31 init=0x00 refin=true refout=true xorout=0x00  check=0xa1
  seed = 0x00;
  return maxim_upd(data, datalen);
}

Razor::Razor(const int idx)
{
  setId(idx);
  this->output_packet.acc[0] = 0;
  this->output_packet.acc[1] = 0;
  this->output_packet.acc[2] = 0;
  this->output_packet.gyro[0] = 0;
  this->output_packet.gyro[1] = 0;
  this->output_packet.gyro[2] = 0;
  this->output_packet.mag[0] = 0;
  this->output_packet.mag[1] = 0;
  this->output_packet.mag[2] = 0;
  memset(&this->acc_settings, 0, sizeof(struct accSettings));

  // calibrated only for 2g scale!!!
  this->acc_settings.ax_scale = 16391.0f;
  this->acc_settings.ay_scale = 16398.0f;
  this->acc_settings.az_scale = 16527.0f;
  this->acc_settings.ax_offset = 204.5f;
  this->acc_settings.ay_offset = 125.0f;
  this->acc_settings.az_offset = 393.0f;

  memset(&this->mag_settings, 0, sizeof(struct magSettings));
  this->mag_settings.mTransform[0][0] = 1;
  this->mag_settings.mTransform[1][1] = 1;
  this->mag_settings.mTransform[2][2] = 1;
}
Razor::~Razor()
{
  /// TODO Auto-generated destructor stub
}

void Razor::compensate_sensor_errors()
{
  // get sensytivites for gyro and mag
  const static float gyroSense = (float)this->getGyroSens();
  const static float magSense = (float)this->getMagSens();

  // compensate accelerometer errors using calibration values
  this->output_packet.acc[0] = ((float)this->ax - this->acc_settings.ax_offset) / this->acc_settings.ax_scale;
  this->output_packet.acc[1] = ((float)this->ay - this->acc_settings.ay_offset) / this->acc_settings.ay_scale;
  this->output_packet.acc[2] = ((float)this->az - this->acc_settings.az_offset) / this->acc_settings.az_scale;

  // get gyroscope values in deg.s^-1
  this->output_packet.gyro[0] = (float)(this->gx) / gyroSense;
  this->output_packet.gyro[1] = (float)(this->gy) / gyroSense;
  this->output_packet.gyro[2] = (float)(this->gz) / gyroSense;

  // Compensate mag error
  // Correct axes orientation first
  this->output_packet.mag[0] = (float)(this->my) / magSense;
  this->output_packet.mag[1] = (float)(this->mx) / magSense;
  this->output_packet.mag[2] = -1.0f * ((float)(this->mz) / magSense);

  // Mag_readings = M_iron_effects(Rot_w_to_body*Mag_real) - Mag_offset;

  // remove offset from readed data
  this->output_packet.mag[0] = (this->output_packet.mag[0] - this->mag_settings.mx_offset);
  this->output_packet.mag[1] = (this->output_packet.mag[1] - this->mag_settings.my_offset);
  this->output_packet.mag[2] = (this->output_packet.mag[2] - this->mag_settings.mz_offset);

  // copy mag values to temp_mag
  // memcpy(this->temp_output_mag, this->output_packet.mag, 3 * sizeof(float));

  // multiply by soft iron correction matrix
  // matrix_Vector_Multiply(this->mag_settings.mTransform, this->temp_output_mag, this->output_packet.mag);
}

void Razor::printAccSettings()
{
  struct accSettings curr = getAccSettings();
  LOG_PORT.println("..............ACC OFFSETS...............");
  LOG_PORT.print("ax_offset: ");
  LOG_PORT.println(curr.ax_offset);
  LOG_PORT.print("ay_offset: ");
  LOG_PORT.println(curr.ay_offset);
  LOG_PORT.print("az_offset: ");
  LOG_PORT.println(curr.az_offset);
  LOG_PORT.println("..............ACC SCALES...............");
  LOG_PORT.print("ax_scale: ");
  LOG_PORT.println(curr.ax_scale);
  LOG_PORT.print("ay_scale: ");
  LOG_PORT.println(curr.ay_scale);
  LOG_PORT.print("az_scale: ");
  LOG_PORT.println(curr.az_scale);
}

void Razor::printMagSettings()
{
  struct magSettings curr = getMagSettings();
  int I;
  LOG_PORT.println("..............MAG OFFSETS...............");
  LOG_PORT.print("mx_offset: ");
  LOG_PORT.println(curr.mx_offset);
  LOG_PORT.print("my_offset: ");
  LOG_PORT.println(curr.my_offset);
  LOG_PORT.print("mz_offset: ");
  LOG_PORT.println(curr.mz_offset);
  LOG_PORT.println("......MAG TRANSFORMATION MATRIX.........");
  for (I = 0; I < 3; I++)
  {
    LOG_PORT.print(curr.mTransform[I][0]);
    LOG_PORT.print("\t");
    LOG_PORT.print(curr.mTransform[I][1]);
    LOG_PORT.print("\t");
    LOG_PORT.println(curr.mTransform[I][2]);
  }
}
void Razor::printPacket()
{
  struct dataFrame *packet = &this->output_packet;
  LOG_PORT.println("--------------Current packet-------------");
  LOG_PORT.println(packet->sync01, HEX);
  LOG_PORT.println(packet->sync02, HEX);
  LOG_PORT.println(packet->ID);
  // Serial.println(packet->index);
  LOG_PORT.print(String(packet->acc[0]));
  LOG_PORT.print('\t');
  LOG_PORT.print(String(packet->acc[1]));
  LOG_PORT.print('\t');
  LOG_PORT.print(String(packet->acc[2]));
  LOG_PORT.println('\t');
  LOG_PORT.print(String(packet->gyro[0]));
  LOG_PORT.print('\t');
  LOG_PORT.print(String(packet->gyro[1]));
  LOG_PORT.print('\t');
  LOG_PORT.print(String(packet->gyro[2]));
  LOG_PORT.println('\t');
  LOG_PORT.print(String(packet->mag[0]));
  LOG_PORT.print('\t');
  LOG_PORT.print(String(packet->mag[1]));
  LOG_PORT.print('\t');
  LOG_PORT.print(String(packet->mag[2]));
  LOG_PORT.println('\t');
  LOG_PORT.print(packet->euler[0]);
  LOG_PORT.println('\t');
  LOG_PORT.print(packet->euler[1]);
  LOG_PORT.println('\t');
  LOG_PORT.print(packet->euler[2]);
  LOG_PORT.println('\t');
  LOG_PORT.println(packet->crc8, HEX);
  LOG_PORT.println("---------------------------------------");
}

void Razor::sendPacket()
{
  struct dataFrame *packet = &this->output_packet;
#if DEBUG
  this->printPacket();
#endif
  packet->crc8 = 0;
  // packet->crc8 = crc.maxim((uint8_t *)packet, this->packetSize);
#if DEBUG
  this->printPacket();
#endif
  LOG_PORT.write((uint8_t *)packet, this->packetSize);
  // packet->index++;
}
