/**
 * @file Gyroscope.h
 * @brief Library for interfacing with the MPU6050 sensor
 * 
 * This library is a fork of the MPU6050_light library by Romain JL. FETICK, licensed under the MIT License.
 * It provides methods to read gyroscope and accelerometer data from the MPU6050 sensor and calculate tilt angles.
 * 
 * License: MIT
 * 
 * Copyright (c) 2020 Romain JL. FETICK
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
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
 * Forked and Modified by João V.M Grando (https://github.com/MutantHardware/)
 * 
 * Modifications:
 * - Changed the constructor and begin method to work with different I2C implementations.
 * - Renamed methods for clarity and consistency.
 *
 * The original library can be found at: https://github.com/rfetick/MPU6050_light
 * 
 * The register map for the MPU6050 is available at:
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 *
 * Mapping of the different gyro and accelerometer configurations:
 *
 * GYRO_CONFIG_[0,1,2,3] range = ± [250, 500, 1000, 2000] deg/s
 *                       sensitivity = [131, 65.5, 32.8, 16.4] LSB/(deg/s)
 *
 * ACC_CONFIG_[0,1,2,3] range = ± [ 2,   4,   8,  16] times the gravity (9.81 m/s²)
 *                       sensitivity = [16384, 8192, 4096, 2048] LSB/gravity
 */

#ifndef Gyroscope_H
#define Gyroscope_H

#include "Arduino.h"
#include "Wire.h"

//#define Gyroscope_ADDR                  0x69  
#define Gyroscope_ADDR                  0x68 
#define Gyroscope_SMPLRT_DIV_REGISTER   0x19
#define Gyroscope_CONFIG_REGISTER       0x1a
#define Gyroscope_GYRO_CONFIG_REGISTER  0x1b
#define Gyroscope_ACCEL_CONFIG_REGISTER 0x1c
#define Gyroscope_PWR_MGMT_1_REGISTER   0x6b

#define Gyroscope_GYRO_OUT_REGISTER     0x43
#define Gyroscope_ACCEL_OUT_REGISTER    0x3B

#define RAD_2_DEG             57.29578 // [deg/rad]
#define CALIB_OFFSET_NB_MES   500
#define TEMP_LSB_2_DEGREE     340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET       12412.0

#define DEFAULT_GYRO_COEFF    0.98

class Gyroscope{
  public:
    // INIT and BASIC FUNCTIONS
	
    byte begin(TwoWire &w, uint8_t GyroAddress);
	
	byte writeData(byte reg, byte data);
    byte readData(byte reg);
	
	void calcOffsets(bool is_calc_gyro=true, bool is_calc_acc=true);
	void calcGyroOffsets(){ calcOffsets(true,false); }; // retro-compatibility with v1.0.0
	void calcAccOffsets(){ calcOffsets(false,true); }; // retro-compatibility with v1.0.0
	
	void setAddress(uint8_t addr){ address = addr; };
	uint8_t getAddress(){ return address; };
	
	// MPU CONFIG SETTER
	byte setGyroConfig(int config_num);
	byte setAccConfig(int config_num);
	
    void setGyroOffsets(float x, float y, float z);
	void setAccOffsets(float x, float y, float z);
	
	void setFilterGyroCoef(float gyro_coeff);
	void setFilterAccCoef(float acc_coeff);

	// MPU CONFIG GETTER
	float getGyroXoffset(){ return gyroXoffset; };
    float getGyroYoffset(){ return gyroYoffset; };
    float getGyroZoffset(){ return gyroZoffset; };
	
	float getAccXoffset(){ return accXoffset; };
	float getAccYoffset(){ return accYoffset; };
	float getAccZoffset(){ return accZoffset; };
	
	float getFilterGyroCoef(){ return filterGyroCoef; };
	float getFilterAccCoef(){ return 1.0-filterGyroCoef; };
	
	// DATA GETTER
    float Temperature(){ return temp; };

    float AccX(){ return accX; };
    float AccY(){ return accY; };
    float AccZ(){ return accZ; };

    float GyroX(){ return gyroX; };
    float GyroY(){ return gyroY; };
    float GyroZ(){ return gyroZ; };
	
	float AccAngleX(){ return angleAccX; };
    float AccAngleY(){ return angleAccY; };

    float AngleX(){ return angleX; };
    float AngleY(){ return angleY; };
    float AngleZ(){ return angleZ; };

	// INLOOP UPDATE
	void fetchData(); // user should better call 'update' that includes 'fetchData'
    void update();
	
	// UPSIDE DOWN MOUNTING
	bool upsideDownMounting = true;

  private:
    TwoWire *wire;
	// uint8_t address = Gyroscope_ADDR;
	uint8_t address = Gyroscope_ADDR;
	float gyro_lsb_to_degsec, acc_lsb_to_g;
    float gyroXoffset, gyroYoffset, gyroZoffset;
	float accXoffset, accYoffset, accZoffset;
    float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
    float angleAccX, angleAccY;
    float angleX, angleY, angleZ;
    long preInterval;
    float filterGyroCoef;     
};

#endif

