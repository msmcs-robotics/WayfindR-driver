/**
 * Example: MPU6050 IMU Driver
 *
 * Shows how to implement the IMUInterface for a specific sensor.
 * The MPU6050 is a common, inexpensive 6-axis IMU.
 */

#ifndef MPU6050_IMU_H
#define MPU6050_IMU_H

#include "flight_controller_wrapper.h"
#include <Wire.h>

class MPU6050_IMU : public IMUInterface {
public:
    // I2C address (0x68 default, 0x69 if AD0 high)
    MPU6050_IMU(uint8_t address = 0x68) : _address(address) {}

    void init() override {
        Wire.begin();
        Wire.setClock(400000);  // 400kHz I2C

        // Wake up MPU6050
        writeRegister(0x6B, 0x00);  // PWR_MGMT_1: Wake up
        delay(100);

        // Configure gyroscope (500 deg/s full scale)
        writeRegister(0x1B, 0x08);  // GYRO_CONFIG: FS_SEL = 1
        _gyroScale = 500.0f / 32768.0f;  // deg/s per LSB

        // Configure accelerometer (4g full scale)
        writeRegister(0x1C, 0x08);  // ACCEL_CONFIG: AFS_SEL = 1
        _accelScale = 4.0f / 32768.0f;  // g per LSB

        // Configure low-pass filter (44Hz)
        writeRegister(0x1A, 0x03);  // CONFIG: DLPF_CFG = 3

        // Set sample rate (1kHz / (1 + 1) = 500Hz)
        writeRegister(0x19, 0x01);  // SMPLRT_DIV

        Serial.println("MPU6050 initialized");
    }

    void read(IMUData& data) override {
        uint8_t buffer[14];

        Wire.beginTransmission(_address);
        Wire.write(0x3B);  // ACCEL_XOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom(_address, (uint8_t)14);

        if (Wire.available() == 14) {
            for (int i = 0; i < 14; i++) {
                buffer[i] = Wire.read();
            }

            // Parse accelerometer (bytes 0-5)
            int16_t ax_raw = (buffer[0] << 8) | buffer[1];
            int16_t ay_raw = (buffer[2] << 8) | buffer[3];
            int16_t az_raw = (buffer[4] << 8) | buffer[5];

            // Skip temperature (bytes 6-7)

            // Parse gyroscope (bytes 8-13)
            int16_t gx_raw = (buffer[8] << 8) | buffer[9];
            int16_t gy_raw = (buffer[10] << 8) | buffer[11];
            int16_t gz_raw = (buffer[12] << 8) | buffer[13];

            // Apply calibration offsets
            gx_raw -= _gyroOffsetX;
            gy_raw -= _gyroOffsetY;
            gz_raw -= _gyroOffsetZ;

            // Convert to physical units
            data.accel_x = ax_raw * _accelScale;
            data.accel_y = ay_raw * _accelScale;
            data.accel_z = az_raw * _accelScale;

            data.gyro_x = gx_raw * _gyroScale;
            data.gyro_y = gy_raw * _gyroScale;
            data.gyro_z = gz_raw * _gyroScale;

            data.valid = true;
        } else {
            data.valid = false;
        }
    }

    const char* getName() override { return "MPU6050"; }

    // Calibrate gyroscope offsets (call while stationary)
    void calibrate(int samples = 1000) {
        Serial.println("Calibrating MPU6050 (keep still)...");

        int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

        for (int i = 0; i < samples; i++) {
            uint8_t buffer[6];

            Wire.beginTransmission(_address);
            Wire.write(0x43);  // GYRO_XOUT_H
            Wire.endTransmission(false);
            Wire.requestFrom(_address, (uint8_t)6);

            if (Wire.available() == 6) {
                for (int j = 0; j < 6; j++) {
                    buffer[j] = Wire.read();
                }

                gx_sum += (int16_t)((buffer[0] << 8) | buffer[1]);
                gy_sum += (int16_t)((buffer[2] << 8) | buffer[3]);
                gz_sum += (int16_t)((buffer[4] << 8) | buffer[5]);
            }

            delay(2);  // ~500Hz sampling
        }

        _gyroOffsetX = gx_sum / samples;
        _gyroOffsetY = gy_sum / samples;
        _gyroOffsetZ = gz_sum / samples;

        Serial.printf("Gyro offsets: X=%d Y=%d Z=%d\n",
                      _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ);
    }

private:
    uint8_t _address;
    float _gyroScale;
    float _accelScale;
    int16_t _gyroOffsetX = 0, _gyroOffsetY = 0, _gyroOffsetZ = 0;

    void writeRegister(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(_address);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }
};

#endif // MPU6050_IMU_H
