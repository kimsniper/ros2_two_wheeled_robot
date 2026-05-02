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
#include "robot_estimation/imu_ekf.hpp"
#include <cmath>

ImuEKF::ImuEKF()
{
    x = {0,0,0,1, 0,0,0}; // quaternion + gyro bias

    P.fill(0.0f);
    for (int i = 0; i < 7; i++)
        P[i*7 + i] = 0.01f;

    dt = 0.01f;
}

void ImuEKF::setDt(float d)
{
    dt = d;
}

std::array<float,4> ImuEKF::getQuaternion() const
{
    return {x[0], x[1], x[2], x[3]};
}

std::array<float,3> ImuEKF::gravityModel(const std::array<float,4>& q) const
{
    return {
        2.0f * (q[1]*q[3] - q[0]*q[2]),
        2.0f * (q[0]*q[1] + q[2]*q[3]),
        q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]
    };
}

void ImuEKF::predict(const std::array<float,3>& gyro)
{
    float qx = x[0], qy = x[1], qz = x[2], qw = x[3];
    float bx = x[4], by = x[5], bz = x[6];

    float wx = gyro[0] - bx;
    float wy = gyro[1] - by;
    float wz = gyro[2] - bz;

    float dq0 = 0.5f * (-qx*wx - qy*wy - qz*wz);
    float dq1 = 0.5f * ( qw*wx + qy*wz - qz*wy);
    float dq2 = 0.5f * ( qw*wy - qx*wz + qz*wx);
    float dq3 = 0.5f * ( qw*wz + qx*wy - qy*wx);

    qx += dq1 * dt;
    qy += dq2 * dt;
    qz += dq3 * dt;
    qw += dq0 * dt;

    float norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    qx /= norm; qy /= norm; qz /= norm; qw /= norm;

    x[0] = qx;
    x[1] = qy;
    x[2] = qz;
    x[3] = qw;
}

void ImuEKF::update(const std::array<float,3>& accel)
{
    float ax = accel[0];
    float ay = accel[1];
    float az = accel[2];

    float norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;

    ax /= norm;
    ay /= norm;
    az /= norm;

    std::array<float,4> q = {x[0], x[1], x[2], x[3]};
    std::array<float,3> g = gravityModel(q);

    float ex = (ay * g[2] - az * g[1]);
    float ey = (az * g[0] - ax * g[2]);
    float ez = (ax * g[1] - ay * g[0]);

    float k = 0.1f;

    float bx = x[4], by = x[5], bz = x[6];

    bx += k * ex * dt;
    by += k * ey * dt;
    bz += k * ez * dt;

    x[4] = bx;
    x[5] = by;
    x[6] = bz;
}