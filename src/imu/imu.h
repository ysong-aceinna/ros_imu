/*******************************************************
 * @file imu.h
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


class CIMU
{
public:
    string m_sn;
    string m_pn;
    string m_packetType;
    string m_arc;
    string m_port;
    int32_t m_baud;
    int m_odr;

public:
    CIMU();
    virtual ~CIMU();

};
