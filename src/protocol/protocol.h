/*******************************************************
 * @file protocol.h
 * @author SongYang (ysong@aceinna.com)
 * @brief 
 * @date 2021-05-01
 * 
 * @copyright Copyright (c) 2021
 * 
*******************************************************/

#pragma once
#include <iostream>
using namespace std;

#pragma pack(1)

struct S_a1
{
    uint32_t itow;
    double   dblItow;
    float    roll;
    float    pitch;
    float    corrRates[3];
    float    accels[3];
    uint8_t  ekfOpMode;
    uint8_t  accelLinSwitch;
    uint8_t  turnSwitch;
};

struct S_s1
{
    uint32_t tstmp;
    double   dbTstmp;
    float    accel_g[3];
    float    rate_dps[3];
    float    mag_G[3];
    float    temp_C;
};

struct S_e1
{
    uint32_t tstmp;
    double   dbTstmp;
    float    roll;
    float    pitch;
    float    yaw;
    float    accels_g[3];
    float    rates_dps[3];
    float    rateBias[3];
    float    mags[3];
    uint8_t  opMode;
    uint8_t  accelLinSwitch;
    uint8_t  turnSwitch;
};

struct S_e2
{
    uint32_t tstmp;
    double   dbTstmp;
    float    roll;
    float    pitch;
    float    yaw;
    float    accels_g[3];
    float    accelBias[3];
    float    rates_dps[3];
    float    rateBias[3];
    float    velocity[3];
    float    mags[3];
    double   pos[3];
    uint8_t  opMode;
    uint8_t  accelLinSwitch;
    uint8_t  turnSwitch;
};

struct S_a2
{
    uint32_t itow;
    double   dblItow;
    float    roll;
    float    pitch;
    float    yaw;
    float    corrRates[3];
    float    accels[3];
};

#pragma pack ()

