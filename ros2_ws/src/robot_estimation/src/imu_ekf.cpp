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
    x = {0,0,0,1, 0,0,0};

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
        2.0f * (q[0]*q[2] - q[3]*q[1]),
        2.0f * (q[3]*q[0] + q[1]*q[2]),
        q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2]
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

    float F[49] = {0};
    for(int i=0;i<7;i++) F[i*7+i]=1.0f;

    std::array<float,49> Pnew = {0};
    for(int i=0;i<7;i++)
        for(int j=0;j<7;j++)
            for(int k=0;k<7;k++)
                Pnew[i*7+j]+=F[i*7+k]*P[k*7+j];

    for(int i=0;i<49;i++) P[i]=Pnew[i];
}

void ImuEKF::update(const std::array<float,3>& accel)
{
    float ax = accel[0];
    float ay = accel[1];
    float az = accel[2];

    float accel_mag = std::sqrt(ax*ax + ay*ay + az*az);
    if (accel_mag < 1e-6f) return;
    if (std::fabs(accel_mag - 9.81f) > 2.0f) return;

    ax /= accel_mag;
    ay /= accel_mag;
    az /= accel_mag;

    std::array<float,4> q = {x[0], x[1], x[2], x[3]};
    std::array<float,3> g = gravityModel(q);

    float ex = ax - g[0];
    float ey = ay - g[1];
    float ez = az - g[2];

    ex *= 0.05f;
    ey *= 0.05f;
    ez *= 0.05f;

    if (ex > 0.5f) ex = 0.5f;
    if (ex < -0.5f) ex = -0.5f;
    if (ey > 0.5f) ey = 0.5f;
    if (ey < -0.5f) ey = -0.5f;
    if (ez > 0.5f) ez = 0.5f;
    if (ez < -0.5f) ez = -0.5f;

    float Sx = P[0] + 0.05f;
    float Sy = P[8] + 0.05f;
    float Sz = P[16] + 0.05f;

    float Kx = P[0] / Sx;
    float Ky = P[8] / Sy;
    float Kz = P[16] / Sz;

    x[0] += Kx * ex * dt;
    x[1] += Ky * ey * dt;
    x[2] += Kz * ez * dt;

    float n = std::sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]+x[3]*x[3]);
    x[0]/=n; x[1]/=n; x[2]/=n; x[3]/=n;

    float bx = x[4], by = x[5], bz = x[6];

    bx += 0.0001f * ex * dt;
    by += 0.0001f * ey * dt;
    bz += 0.0001f * ez * dt;

    x[4] = bx;
    x[5] = by;
    x[6] = bz;
}