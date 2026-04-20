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
    x(3) = 1.0f; // quaternion w

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
        2.0f * (q(0)*q(2) - q(3)*q(1)),
        2.0f * (q(3)*q(0) + q(1)*q(2)),
        q(3)*q(3) - q(0)*q(0) - q(1)*q(1) + q(2)*q(2)
    };
}

void ImuEKF::predict(const Eigen::Vector3f &gyro)
{
    Eigen::Vector4f q = x.segment<4>(0);
    Eigen::Vector3f bg = x.segment<3>(4);

    Eigen::Vector3f w = gyro - bg;

    Eigen::Vector4f qdot;
    qdot << 
        0.5f * ( w(0)*q(3) + w(1)*q(2) - w(2)*q(1)),
        0.5f * (-w(0)*q(2) + w(1)*q(3) + w(2)*q(0)),
        0.5f * ( w(0)*q(1) - w(1)*q(0) + w(2)*q(3)),
        0.5f * (-w(0)*q(0) - w(1)*q(1) - w(2)*q(2));

    q += qdot * dt;
    q.normalize();

    x.segment<4>(0) = q;

    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(7,7);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(7,7) * 0.001f;

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

    float eps = 1e-5f;

    for (int i = 0; i < 4; i++) {
        Eigen::Vector4f q_eps = q;
        q_eps(i) += eps;
        q_eps.normalize();

        Eigen::Vector3f h_eps = gravityModel(q_eps);
        h_eps.normalize();

        H.col(i) = (h_eps - h) / eps;
    }

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity() * 0.05f;

    Eigen::Matrix3f S = H * P * H.transpose() + R;
    Eigen::Matrix<float,7,3> K = P * H.transpose() * S.inverse();

    x = x + K * y;

    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(7,7);
    P = (I - K * H) * P;

    Eigen::Vector4f q_new = x.segment<4>(0);
    q_new.normalize();
    x.segment<4>(0) = q_new;
}