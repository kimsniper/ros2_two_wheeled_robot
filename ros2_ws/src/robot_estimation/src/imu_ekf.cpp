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

    float dw = 0.5f * (-qx*wx - qy*wy - qz*wz);
    float dx = 0.5f * ( qw*wx + qy*wz - qz*wy);
    float dy = 0.5f * ( qw*wy - qx*wz + qz*wx);
    float dz = 0.5f * ( qw*wz + qx*wy - qy*wx);

    qx += dx * dt;
    qy += dy * dt;
    qz += dz * dt;
    qw += dw * dt;

    float norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;

    x[0] = qx;
    x[1] = qy;
    x[2] = qz;
    x[3] = qw;

    float F[49] = {
        1,0,0,0,-0.5f*dt*qw, 0.5f*dt*qz,-0.5f*dt*qy,
        0,1,0,0,-0.5f*dt*qz,-0.5f*dt*qw, 0.5f*dt*qx,
        0,0,1,0, 0.5f*dt*qy,-0.5f*dt*qx,-0.5f*dt*qw,
        0,0,0,1, 0.5f*dt*qx, 0.5f*dt*qy, 0.5f*dt*qz,
        0,0,0,0,1,0,0,
        0,0,0,0,0,1,0,
        0,0,0,0,0,0,1
    };

    float Q[49] = {0};

    for(int i=0;i<7;i++)
        Q[i*7+i]=0.0001f;

    std::array<float,49> FP = {0};

    for(int i=0;i<7;i++)
        for(int j=0;j<7;j++)
            for(int k=0;k<7;k++)
                FP[i*7+j]+=F[i*7+k]*P[k*7+j];

    std::array<float,49> FPFt = {0};

    for(int i=0;i<7;i++)
        for(int j=0;j<7;j++)
            for(int k=0;k<7;k++)
                FPFt[i*7+j]+=FP[i*7+k]*F[j*7+k];

    for(int i=0;i<49;i++)
        P[i]=FPFt[i]+Q[i];
}

void ImuEKF::update(const std::array<float,3>& accel)
{
    float ax = accel[0];
    float ay = accel[1];
    float az = accel[2];

    float accel_mag = std::sqrt(ax*ax + ay*ay + az*az);

    if (accel_mag < 1e-6f)
        return;

    if (std::fabs(accel_mag - 9.81f) > 2.0f)
        return;

    ax /= accel_mag;
    ay /= accel_mag;
    az /= accel_mag;

    std::array<float,4> q = {x[0], x[1], x[2], x[3]};
    std::array<float,3> g = gravityModel(q);

    float y[3] = {
        ax - g[0],
        ay - g[1],
        az - g[2]
    };

    float H[21] = {
         2.0f*q[2],-2.0f*q[3], 2.0f*q[0],-2.0f*q[1],0,0,0,
         2.0f*q[3], 2.0f*q[2], 2.0f*q[1], 2.0f*q[0],0,0,0,
        -2.0f*q[0],-2.0f*q[1], 2.0f*q[2], 2.0f*q[3],0,0,0
    };

    float R[9] = {
        0.05f,0,0,
        0,0.05f,0,
        0,0,0.05f
    };

    float HP[21] = {0};

    for(int i=0;i<3;i++)
        for(int j=0;j<7;j++)
            for(int k=0;k<7;k++)
                HP[i*7+j]+=H[i*7+k]*P[k*7+j];

    float S[9] = {0};

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            for(int k=0;k<7;k++)
                S[i*3+j]+=HP[i*7+k]*H[j*7+k];

    for(int i=0;i<9;i++)
        S[i]+=R[i];

    float det =
        S[0]*(S[4]*S[8]-S[5]*S[7])-
        S[1]*(S[3]*S[8]-S[5]*S[6])+
        S[2]*(S[3]*S[7]-S[4]*S[6]);

    if(std::fabs(det)<1e-9f)
        return;

    float invS[9];

    invS[0]=(S[4]*S[8]-S[5]*S[7])/det;
    invS[1]=(S[2]*S[7]-S[1]*S[8])/det;
    invS[2]=(S[1]*S[5]-S[2]*S[4])/det;

    invS[3]=(S[5]*S[6]-S[3]*S[8])/det;
    invS[4]=(S[0]*S[8]-S[2]*S[6])/det;
    invS[5]=(S[2]*S[3]-S[0]*S[5])/det;

    invS[6]=(S[3]*S[7]-S[4]*S[6])/det;
    invS[7]=(S[1]*S[6]-S[0]*S[7])/det;
    invS[8]=(S[0]*S[4]-S[1]*S[3])/det;

    float PHt[21] = {0};

    for(int i=0;i<7;i++)
        for(int j=0;j<3;j++)
            for(int k=0;k<7;k++)
                PHt[i*3+j]+=P[i*7+k]*H[j*7+k];

    float K[21] = {0};

    for(int i=0;i<7;i++)
        for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)
                K[i*3+j]+=PHt[i*3+k]*invS[k*3+j];

    for(int i=0;i<7;i++)
        for(int j=0;j<3;j++)
            x[i]+=K[i*3+j]*y[j];

    float n = std::sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]+x[3]*x[3]);

    x[0]/=n;
    x[1]/=n;
    x[2]/=n;
    x[3]/=n;

    float KH[49] = {0};

    for(int i=0;i<7;i++)
        for(int j=0;j<7;j++)
            for(int k=0;k<3;k++)
                KH[i*7+j]+=K[i*3+k]*H[k*7+j];

    float I_KH[49] = {0};

    for(int i=0;i<7;i++)
        I_KH[i*7+i]=1.0f;

    for(int i=0;i<49;i++)
        I_KH[i]-=KH[i];

    std::array<float,49> Pnew = {0};

    for(int i=0;i<7;i++)
        for(int j=0;j<7;j++)
            for(int k=0;k<7;k++)
                Pnew[i*7+j]+=I_KH[i*7+k]*P[k*7+j];

    for(int i=0;i<49;i++)
        P[i]=Pnew[i];
}