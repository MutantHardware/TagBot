/*
 * Gyroscope Library for Arduino
 *
 * License: MIT
 * Copyright (c) 2020 Romain JL. FETICK
 * Forked and modified by João V.M Grando
 * Modifications:
 * - Changed the `constructor` and `begin` method to work with different I2C implementations.
 * - Renamed some methods for clarity and consistency.
 *
 * Authors:
 * Romain JL. Fétick (github.com/rfetick) - simplifications and corrections
 * Tockn (github.com/tockn) - initial author (v1.5.2)
 * 
 * Original repository: https://github.com/rfetick/MPU6050_light
 */

#include "Gyroscope.h"
#include "Arduino.h"

/* Wrap an angle in the range [-limit,+limit] (special thanks to Edgar Bonet!) */
static float wrap(float angle,float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}

/* INIT and BASIC FUNCTIONS */

byte Gyroscope::begin(TwoWire &w, uint8_t GyroAddress){
  wire = &w;
  address =  GyroAddress;
  setFilterGyroCoef(DEFAULT_GYRO_COEFF);
  setGyroOffsets(0,0,0);
  setAccOffsets(0,0,0);
 
  // changed calling register sequence [https://github.com/rfetick/Gyroscope_light/issues/1] -> thanks to augustosc
  byte status = writeData(Gyroscope_PWR_MGMT_1_REGISTER, 0x01); // check only the first connection with status
  writeData(Gyroscope_SMPLRT_DIV_REGISTER, 0x00);
  writeData(Gyroscope_CONFIG_REGISTER, 0x00);
  setGyroConfig(1);
  setAccConfig(0);
  
  this->update();
  angleX = this->AccAngleX();
  angleY = this->AccAngleY();
  preInterval = millis(); // may cause lack of angular accuracy if begin() is much before the first update()
  return status;
}

byte Gyroscope::writeData(byte reg, byte data){
  wire->beginTransmission(address);
  wire->write(reg);
  wire->write(data);
  byte status = wire->endTransmission();
  return status; // 0 if success
}

// This method is not used internaly, maybe by user...
byte Gyroscope::readData(byte reg) {
  wire->beginTransmission(address);
  wire->write(reg);
  wire->endTransmission(true);
  wire->requestFrom(address,(uint8_t) 1);
  byte data =  wire->read();
  return data;
}

/* SETTER */

byte Gyroscope::setGyroConfig(int config_num){
  byte status;
  switch(config_num){
    case 0: // range = +- 250 deg/s
	  gyro_lsb_to_degsec = 131.0;
	  status = writeData(Gyroscope_GYRO_CONFIG_REGISTER, 0x00);
	  break;
	case 1: // range = +- 500 deg/s
	  gyro_lsb_to_degsec = 65.5;
	  status = writeData(Gyroscope_GYRO_CONFIG_REGISTER, 0x08);
	  break;
	case 2: // range = +- 1000 deg/s
	  gyro_lsb_to_degsec = 32.8;
	  status = writeData(Gyroscope_GYRO_CONFIG_REGISTER, 0x10);
	  break;
	case 3: // range = +- 2000 deg/s
	  gyro_lsb_to_degsec = 16.4;
	  status = writeData(Gyroscope_GYRO_CONFIG_REGISTER, 0x18);
	  break;
	default: // error
	  status = 1;
	  break;
  }
  return status;
}

byte Gyroscope::setAccConfig(int config_num){
  byte status;
  switch(config_num){
    case 0: // range = +- 2 g
	  acc_lsb_to_g = 16384.0;
	  status = writeData(Gyroscope_ACCEL_CONFIG_REGISTER, 0x00);
	  break;
	case 1: // range = +- 4 g
	  acc_lsb_to_g = 8192.0;
	  status = writeData(Gyroscope_ACCEL_CONFIG_REGISTER, 0x08);
	  break;
	case 2: // range = +- 8 g
	  acc_lsb_to_g = 4096.0;
	  status = writeData(Gyroscope_ACCEL_CONFIG_REGISTER, 0x10);
	  break;
	case 3: // range = +- 16 g
	  acc_lsb_to_g = 2048.0;
	  status = writeData(Gyroscope_ACCEL_CONFIG_REGISTER, 0x18);
	  break;
	default: // error
	  status = 1;
	  break;
  }
  return status;
}

void Gyroscope::setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void Gyroscope::setAccOffsets(float x, float y, float z){
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}

void Gyroscope::setFilterGyroCoef(float gyro_coeff){
  if ((gyro_coeff<0) or (gyro_coeff>1)){ gyro_coeff = DEFAULT_GYRO_COEFF; } // prevent bad gyro coeff, should throw an error...
  filterGyroCoef = gyro_coeff;
}

void Gyroscope::setFilterAccCoef(float acc_coeff){
  setFilterGyroCoef(1.0-acc_coeff);
}

/* CALC OFFSET */

void Gyroscope::calcOffsets(bool is_calc_gyro, bool is_calc_acc){
  if(is_calc_gyro){ setGyroOffsets(0,0,0); }
  if(is_calc_acc){ setAccOffsets(0,0,0); }
  float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro
  
  for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
    this->fetchData();
	ag[0] += accX;
	ag[1] += accY;
	ag[2] += (accZ-1.0);
	ag[3] += gyroX;
	ag[4] += gyroY;
	ag[5] += gyroZ;
	delay(1); // wait a little bit between 2 measurements
  }
  
  if(is_calc_acc){
    accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
    accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
    accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
  }
  
  if(is_calc_gyro){
    gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
    gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
    gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
  }
}

/* UPDATE */

void Gyroscope::fetchData(){
  wire->beginTransmission(address);
  wire->write(Gyroscope_ACCEL_OUT_REGISTER);
  wire->endTransmission(false);
  wire->requestFrom(address,(uint8_t) 14);

  int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]

  for(int i=0;i<7;i++){
	rawData[i]  = wire->read() << 8;
    rawData[i] |= wire->read();
  }

  accX = ((float)rawData[0]) / acc_lsb_to_g - accXoffset;
  accY = ((float)rawData[1]) / acc_lsb_to_g - accYoffset;
  accZ = (!upsideDownMounting - upsideDownMounting) * ((float)rawData[2]) / acc_lsb_to_g - accZoffset;
  temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gyroX = ((float)rawData[4]) / gyro_lsb_to_degsec - gyroXoffset;
  gyroY = ((float)rawData[5]) / gyro_lsb_to_degsec - gyroYoffset;
  gyroZ = ((float)rawData[6]) / gyro_lsb_to_degsec - gyroZoffset;
}

void Gyroscope::update(){
  // retrieve raw data
  this->fetchData();
  
  // estimate tilt angles: this is an approximation for small angles!
  float sgZ = (accZ>=0)-(accZ<0); // allow one angle to go from -180 to +180 degrees
  angleAccX =   atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG; // [-180,+180] deg
  angleAccY = - atan2(accX,     sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG; // [- 90,+ 90] deg

  unsigned long Tnew = millis();
  float dt = (Tnew - preInterval) * 1e-3;
  preInterval = Tnew;

  // Correctly wrap X and Y angles (special thanks to Edgar Bonet!)
  // https://github.com/gabriel-milan/TinyGyroscope/issues/6
  angleX = wrap(filterGyroCoef*(angleAccX + wrap(angleX +     gyroX*dt - angleAccX,180)) + (1.0-filterGyroCoef)*angleAccX,180);
  angleY = wrap(filterGyroCoef*(angleAccY + wrap(angleY + sgZ*gyroY*dt - angleAccY, 90)) + (1.0-filterGyroCoef)*angleAccY, 90);
  angleZ += gyroZ*dt; // not wrapped (to do???)
}
