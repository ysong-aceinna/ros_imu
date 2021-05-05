/*******************************************************
 * @file driver.h
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
#include <queue>
#include <mutex>
#include <thread>
#include <signal.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include "serial/serial.h"
#include "imu.h"
#include "macro.h"
#include "protocol.h"

class CDriver
{
public:
    CDriver(ros::NodeHandle nh);
    virtual ~CDriver();

    void Start();
    void Stop();
    bool Spin();

    static void SigintHandler(int sig);
    void ThreadGetData();

    void ParseFrame(uint8_t* frame, uint16_t len);
    void Handle_a2(uint8_t* frame, uint16_t len);

    uint16_t calcCRC(uint8_t *ptr, uint32_t num);
    string Bytestohexstring(uint8_t* bytes,int bytelength);

private:
    ros::NodeHandle m_nh;
    ros::Publisher m_imu_pub;
    CIMU m_imu;
    string m_topic;
    serial::Serial* m_pserial;

    std::mutex m_mt_buf;
    queue<uint8_t> m_buf;

    bool m_bexit;
    std::mutex m_mx_exit;
    std::thread m_get_data_thread;
};

