/*
 * Copyright (c) 2026, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "imu_node.hpp"

#include <cmath>
#include <chrono>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "imu_node"
#define RCCHECK(fn) if ((fn) != RCL_RET_OK) { printf("ROS ERROR\n"); }

static MPU6050::Device Dev = {
    .i2cPort = 0,
    .i2cAddress = MPU6050::I2C_ADDRESS_MPU6050_AD0_L
};

static void quaternionToEuler(float qx, float qy, float qz, float qw, 
                               float &roll, float &pitch, float &yaw) {
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0f) {
        pitch = std::copysign(M_PI / 2.0f, sinp);
    } else {
        pitch = std::asin(sinp);
    }
    
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void ImuNode::init(rclc_support_t *support, rcl_allocator_t *allocator)
{
    rcl_node_t node;

    rclc_node_init_default(&node, "imu_node", "", support);

    rclc_publisher_init_default(
        &pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"
    );

    memset(&msg, 0, sizeof(msg));

    msg.orientation_covariance[0] = -1;
    msg.angular_velocity_covariance[0] = -1;
    msg.linear_acceleration_covariance[0] = -1;

    if (mpu6050_hal_init(Dev.i2cPort) == Mpu6050_Error_t::MPU6050_ERR) {
        ESP_LOGE(TAG, "Failed to initialize I2C HAL");
        return;
    }

    mpu = new MPU6050::MPU6050_Driver(Dev);

    if (mpu->Mpu6050_Init(&MPU6050::DefaultConfig) != Mpu6050_Error_t::MPU6050_OK) {
        ESP_LOGE(TAG, "MPU6050 initialization failed!");
        return;
    }

    uint8_t dev_id = 0;
    if (mpu->Mpu6050_GetDevideId(dev_id) != Mpu6050_Error_t::MPU6050_OK ||
        dev_id != MPU6050::WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "Invalid MPU6050 device ID: 0x%x", dev_id);
        return;
    }

    calibrate();
}

void ImuNode::calibrate()
{
    ESP_LOGI(TAG, "Keep IMU completely still - calibrating gyro...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    gyroBias.setZero();
    const int samples = 1000;
    
    ESP_LOGI(TAG, "Collecting %d gyro samples...", samples);
    
    for (int i = 0; i < samples; i++) {
        MPU6050::Mpu6050_GyroData_t g;
        
        if (mpu->Mpu6050_GetGyroData(g) == Mpu6050_Error_t::MPU6050_OK) {
            gyroBias(0) += g.Gyro_X * M_PI / 180.0f;
            gyroBias(1) += g.Gyro_Y * M_PI / 180.0f;
            gyroBias(2) += g.Gyro_Z * M_PI / 180.0f;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    gyroBias /= samples;
    
    ESP_LOGI(TAG, "Gyro bias (rad/s): X=%.6f, Y=%.6f, Z=%.6f", 
             gyroBias(0), gyroBias(1), gyroBias(2));
    ESP_LOGI(TAG, "Gyro bias (deg/s): X=%.2f, Y=%.2f, Z=%.2f", 
             gyroBias(0) * 180.0f / M_PI, 
             gyroBias(1) * 180.0f / M_PI, 
             gyroBias(2) * 180.0f / M_PI);
}

void ImuNode::spin()
{
    auto prev = std::chrono::steady_clock::now();
    auto last_log_time = std::chrono::steady_clock::now();
    
    static Eigen::Vector3f accel_fused = Eigen::Vector3f::Zero();
    
    float dt_filtered = 0.01f;
    
    while (true) {
        
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        
        msg.header.stamp.sec = ts.tv_sec;
        msg.header.stamp.nanosec = ts.tv_nsec;
        
        MPU6050::Mpu6050_AccelData_t acc;
        MPU6050::Mpu6050_GyroData_t gyro;
        
        if (mpu->Mpu6050_GetAccelData(acc) != Mpu6050_Error_t::MPU6050_OK ||
            mpu->Mpu6050_GetGyroData(gyro) != Mpu6050_Error_t::MPU6050_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - prev).count();
        prev = now;
        
        dt_filtered = 0.9f * dt_filtered + 0.1f * dt;
        
        if (dt_filtered <= 0.0f || dt_filtered > 0.1f) {
            dt_filtered = 0.01f;
        }
        
        ekf.setDt(dt_filtered);
        
        Eigen::Vector3f g;
        g <<    (gyro.Gyro_X * M_PI / 180.0f),
                (gyro.Gyro_Y * M_PI / 180.0f),
                (gyro.Gyro_Z * M_PI / 180.0f);
        
        g -= gyroBias;
        
        Eigen::Vector3f a;
        a << acc.Accel_X,
             acc.Accel_Y,
             acc.Accel_Z;
        
        float norm = a.norm();
        bool accel_valid = (fabs(norm - 9.81f) < 0.5f);
        
        if (accel_valid) {
            Eigen::Vector3f a_norm = a.normalized();
            accel_fused = 0.7f * accel_fused + 0.3f * a_norm;
        }
        
        ekf.predict(g);
        
        if (accel_valid) {
            ekf.update(accel_fused);
        }
        
        const auto &q = ekf.getQuaternion();
        
        float roll_rad, pitch_rad, yaw_rad;
        quaternionToEuler(q(0), q(1), q(2), q(3), roll_rad, pitch_rad, yaw_rad);
        
        float roll_deg = roll_rad * 180.0f / M_PI;
        float pitch_deg = pitch_rad * 180.0f / M_PI;
        float yaw_deg = yaw_rad * 180.0f / M_PI;
        
        auto log_now = std::chrono::steady_clock::now();
        auto log_elapsed = std::chrono::duration<float>(log_now - last_log_time).count();
        
        if (log_elapsed >= 0.5f) {
            ESP_LOGI(TAG, "Raw Gyro (deg/s) | X: %+7.2f | Y: %+7.2f | Z: %+7.2f", gyro.Gyro_X, gyro.Gyro_Y, gyro.Gyro_Z);
            ESP_LOGI(TAG, "Raw Accel (m/s²) | X: %+7.2f | Y: %+7.2f | Z: %+7.2f | Norm: %.2f", acc.Accel_X, acc.Accel_Y, acc.Accel_Z, norm);
            ESP_LOGI(TAG, "Euler Angles (deg) | Roll: %+7.2f | Pitch: %+7.2f | Yaw: %+7.2f | dt=%.3fs",
                     roll_deg, pitch_deg, yaw_deg, dt_filtered);
            
            last_log_time = log_now;
        }
        
        msg.orientation.x = q(0);
        msg.orientation.y = q(1);
        msg.orientation.z = q(2);
        msg.orientation.w = q(3);
        
        msg.angular_velocity.x = g(0);
        msg.angular_velocity.y = g(1);
        msg.angular_velocity.z = g(2);
        
        msg.linear_acceleration.x = a(0);
        msg.linear_acceleration.y = a(1);
        msg.linear_acceleration.z = a(2);
        
        rcl_ret_t ret = rcl_publish(&pub, &msg, NULL);
        (void)ret;
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}