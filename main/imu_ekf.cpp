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
#include "imu_ekf.hpp"
#include <cmath>

ImuEKF::ImuEKF()
{
    x = Eigen::VectorXf::Zero(7);
    x(3) = 1.0f;

    P = Eigen::MatrixXf::Identity(7,7) * 0.01f;

    dt = 0.01f;
}

void ImuEKF::setDt(float d)
{
    dt = d;
}

Eigen::Vector4f ImuEKF::getQuaternion() const
{
    return x.segment<4>(0);
}

Eigen::Vector3f ImuEKF::gravityModel(const Eigen::Vector4f &q) const
{
    return {
        2.0f * (q(1)*q(3) - q(0)*q(2)),
        2.0f * (q(0)*q(1) + q(2)*q(3)),
        q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3)
    };
}

void ImuEKF::predict(const Eigen::Vector3f &gyro)
{
    Eigen::Vector4f q = x.segment<4>(0);
    Eigen::Vector3f bg = x.segment<3>(4);

    Eigen::Vector3f w = gyro - bg;

    Eigen::Quaternionf q_old(q(3), q(0), q(1), q(2));

    float theta = w.norm() * dt;
    Eigen::Quaternionf dq;
    if (theta < 1e-6f) {
        dq = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
    } else {
        Eigen::Vector3f axis = w / w.norm();
        float half_theta = theta * 0.5f;
        dq = Eigen::Quaternionf(cos(half_theta),
                                axis(0) * sin(half_theta),
                                axis(1) * sin(half_theta),
                                axis(2) * sin(half_theta));
    }

    Eigen::Quaternionf q_new = q_old * dq;
    q_new.normalize();

    q << q_new.x(), q_new.y(), q_new.z(), q_new.w();

    x.segment<4>(0) = q;
    x.segment<3>(4) = bg;

    Eigen::Matrix<float,7,7> F = Eigen::Matrix<float,7,7>::Identity();

    float q0 = q(0);
    float q1 = q(1);
    float q2 = q(2);
    float q3 = q(3);

    Eigen::Matrix<float,4,3> G;
    G << 
        -q1, -q2, -q3,
         q0, -q3,  q2,
         q3,  q0, -q1,
        -q2,  q1,  q0;

    F.block<4,3>(0,4) = -0.5f * G * dt;

    Eigen::Matrix<float,7,7> Q = Eigen::Matrix<float,7,7>::Zero();
    Q.block<3,3>(0,0) = Eigen::Matrix3f::Identity() * 1e-6f;
    Q.block<3,3>(4,4) = Eigen::Matrix3f::Identity() * 1e-6f;

    P = F * P * F.transpose() + Q;
}

void ImuEKF::update(const Eigen::Vector3f &accel)
{
    float norm = accel.norm();
    if (norm < 5.0f || norm > 15.0f) return;

    Eigen::Vector3f z = accel.normalized();

    Eigen::Vector4f q = x.segment<4>(0);

    Eigen::Vector3f h = gravityModel(q);
    h.normalize();

    Eigen::Vector3f y = z - h;

    Eigen::Matrix<float,3,7> H;
    H.setZero();

    float q0 = q(0);
    float q1 = q(1);
    float q2 = q(2);
    float q3 = q(3);

    H(0,0) = 2.0f * q2;
    H(0,1) = -2.0f * q3;
    H(0,2) = 2.0f * q0;
    H(0,3) = -2.0f * q1;

    H(1,0) = -2.0f * q1;
    H(1,1) = -2.0f * q0;
    H(1,2) = -2.0f * q3;
    H(1,3) = -2.0f * q2;

    H(2,0) = -2.0f * q0;
    H(2,1) = 2.0f * q1;
    H(2,2) = 2.0f * q2;
    H(2,3) = -2.0f * q3;

    for (int i = 4; i < 7; i++) {
        H.col(i).setZero();
    }

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity() * 0.02f;

    Eigen::Matrix3f S = H * P * H.transpose() + R;
    Eigen::Matrix<float,7,3> K = P * H.transpose() * S.inverse();

    x = x + K * y;

    Eigen::Matrix<float,7,7> I = Eigen::Matrix<float,7,7>::Identity();
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

    Eigen::Vector4f q_new = x.segment<4>(0);
    q_new.normalize();
    x.segment<4>(0) = q_new;
}