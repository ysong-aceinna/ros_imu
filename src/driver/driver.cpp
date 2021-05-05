/*******************************************************
 * @file driver.cpp
 * @author SongYang (ysong@aceinna.com)
 * @brief 
 * @date 2021-05-01
 * 
 * @copyright Copyright (c) 2021
 * 
*******************************************************/
#include "driver.h"
const float r2d = 180/3.14159265;

CDriver::CDriver(ros::NodeHandle nh)
    : m_nh(nh)
{
    cout << "CDriver:CDriver()" << endl;
    m_pserial = nullptr;

    m_nh.param("device", m_imu.m_port, std::string("/dev/ttyUSB0"));
    m_nh.param("baud", m_imu.m_baud, 115200);

    ROS_INFO("device: %s", m_imu.m_port.c_str());
    ROS_INFO("baud: %d", m_imu.m_baud);

    m_imu_pub = m_nh.advertise<sensor_msgs::Imu>("topic_imu", 100);
}

CDriver::~CDriver()
{
    cout << "CDriver:~CDriver()" << endl;
}

void CDriver::Start()
{
    // Set and open serial port.
    m_pserial = new serial::Serial(m_imu.m_port, m_imu.m_baud, serial::Timeout::simpleTimeout(10));

    while (!m_pserial->isOpen())
    {
        ROS_WARN("Keep trying to open the device in 0.1 second period...");
        usleep(100000); // 100 ms
        m_pserial->open();
    }
    ROS_INFO("Device is opened successfully. [%s:%d]", m_imu.m_port.c_str(), m_imu.m_baud);
        
    m_mx_exit.lock();
    m_bexit = false;
    m_mx_exit.unlock();
    m_get_data_thread = std::thread(&CDriver::ThreadGetData, this);

    signal(SIGINT, SigintHandler);
    Spin();
}

void CDriver::Stop()
{
    cout << "CDriver::Stop()" << endl;
    if(m_pserial)
    {
        if(m_pserial->isOpen())
        {
            m_pserial->close();
        }
        SAFEDELETE(m_pserial);
    }

    m_mx_exit.lock();
    m_bexit = true;
    m_mx_exit.unlock();
    usleep(1000);
}

void CDriver::SigintHandler(int sig)
{
    // Capture Ctrl+C.
    ROS_INFO("shutting down!");
	ros::shutdown();
}

bool CDriver::Spin()
{
    double rate_ = 200; // Hz
    ros::Rate loop_rate(rate_);

    size_t max_sz = 200;
    uint8_t *buf = new uint8_t[max_sz];
    while (ros::ok())
    {
        /*
         * ros::ok() return false once:
         *  1. ros::shutdown() has been called and is finished
         *  2. SIGINT (Ctrl-C)
         *  3. ros::NodeHandles were destroyed.
         *  4. Node with same name appears in ROS network.
	     */

        size_t sz = m_pserial->read(buf, max_sz);

        m_mt_buf.lock();
        for (size_t i = 0; i < sz; i++)
        {
            m_buf.push(buf[i]);
        }
        m_mt_buf.unlock();

        ros::spinOnce();
        loop_rate.sleep();
    }

    Stop();

    return true;
};

void CDriver::ThreadGetData()
{
    cout << "CDriver::ThreadGetData()" << endl;

    const uint8_t HEADER[] = {0X55, 0X55};
    const uint8_t PACKAGE_TYPE_IDX = 2;
    const uint8_t PAYLOAD_LEN_IDX = 4;
    const int MAX_FRAME_LIMIT = 256;  // assume max len of frame is smaller than MAX_FRAME_LIMIT.

    deque<uint8_t> sync_pattern (2, 0);
	uint8_t* buf = new uint8_t[MAX_FRAME_LIMIT];
    bool find_header = false;
    uint16_t payload_len = 0;
    uint8_t d = 0;
	int idx = 0;

    while (1)
    {
        m_mx_exit.lock();
        if(m_bexit) 
        {
            m_mx_exit.unlock();
            break;
        }
        m_mx_exit.unlock();

        m_mt_buf.lock();
        if (m_buf.empty()) 
        {
            m_mt_buf.unlock();
            usleep(1000); // 1ms
            continue;
        }
        d = m_buf.front();
        m_buf.pop();
        m_mt_buf.unlock();
        
        if(find_header)
        {
            buf[++idx] = d;

            if (PAYLOAD_LEN_IDX == idx)
            {
                payload_len = buf[PAYLOAD_LEN_IDX];
            }
            else if ( 2 + 2 + 1 + payload_len + 2 == idx + 1)  // 2: len of header 'UU'; 2: package type 'a1'; 1: payload len; 2:len of checksum.
            {    
                find_header = false;
                // checksum
                uint16_t packet_crc = 256 * buf[idx-1] + buf[idx];
                if (packet_crc == calcCRC(&buf[PACKAGE_TYPE_IDX], idx-3)) // 4: len of header 'UU', and len of checksum.
                {
                    // find a whole frame
                    ParseFrame(buf, idx+1);
                }
                else
                {
                    cout << "CRC error! : " << Bytestohexstring(buf, idx + 1) << endl;
                }
            }

            if (payload_len > MAX_FRAME_LIMIT || idx > MAX_FRAME_LIMIT)
            {
                find_header = false;
                payload_len = 0;
    			memset(buf, MAX_FRAME_LIMIT, 0);
            }
        }
        else  // if hasn't found header 'UU'
        {
            sync_pattern.emplace_front(d);
            if (sync_pattern[0] == HEADER[0] && sync_pattern[1] == HEADER[1])
            {
                idx = 1;
    			memcpy(buf, HEADER, 2);
                find_header = true;
                sync_pattern[0] = 0;
            }
            sync_pattern.resize(2, 0);
        }
    }
}

void CDriver::ParseFrame(uint8_t* frame, uint16_t len)
{
    const uint8_t PACKAGE_TYPE_IDX = 2;

    string packetType;
    packetType.push_back(frame[PACKAGE_TYPE_IDX]);
    packetType.push_back(frame[PACKAGE_TYPE_IDX + 1]);

    if(m_imu.m_packetType.compare(packetType) != 0)
    {
        m_imu.m_packetType = packetType;
        ROS_INFO("PACKET TYPE:%s", packetType.c_str());
    }

    if(m_imu.m_packetType.compare("a1") == 0)
    {
    }
    else if(m_imu.m_packetType.compare("a2") == 0)
    {
        Handle_a2(frame, len);
    }
    else
    {
        ROS_WARN("Unknown Packet Type!");
    }
}

void CDriver::Handle_a2(uint8_t* frame, uint16_t len)
{
    // Check size of frame.
    if (len - 7 != sizeof(S_a2)) // 7: UU (2), packet type(2), playload length(1), crc(2).
    {
        ROS_WARN("Frame size error!");
        return;
    }

    S_a2 *p = (S_a2*)(&frame[5]);

    // cout << " itow:" << p->itow << " dblItow:" << p->dblItow 
    // << " rpy:[" << p->roll << ", " << p->pitch  << ", " << p->yaw << " deg] "
    // << " gyro:[" << p->corrRates[0] << ", " << p->corrRates[1] << ", " << p->corrRates[2] << " dps] "
    // << " accel:[" << p->accels[0] << ", " << p->accels[1] << ", " << p->accels[2] << " m/s^2] " << endl;

    sensor_msgs::Imu data;
    data.header.frame_id = "base_link";
    data.header.stamp = ros::Time::now();

    // Linear acceleration [m/s^2]
    data.linear_acceleration.x = p->accels[0];
    data.linear_acceleration.y = p->accels[1];
    data.linear_acceleration.z = p->accels[2];

    // Angular velocity [rad/s]
    data.angular_velocity.x = p->corrRates[0]/r2d;
    data.angular_velocity.y = p->corrRates[1]/r2d;
    data.angular_velocity.z = p->corrRates[2]/r2d;

    // Orientation (not provided)
    // data.orientation.x = 0;
    // data.orientation.y = 0;
    // data.orientation.z = 0;
    // data.orientation.w = 1;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(p->roll/r2d, p->pitch/r2d, p->yaw/r2d);
    data.orientation = odom_quat;

    m_imu_pub.publish(data);

    // Publish TF
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setW(odom_quat.w);
    q.setX(odom_quat.x);
    q.setY(odom_quat.y);
    q.setZ(odom_quat.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, data.header.stamp, "base_link", "body"));
}

/*******************************************************************************
* FUNCTION: calcCRC calculates a 2-byte CRC on serial data using 
*           CRC-CCITT 16-bit standard maintained by the ITU 
*           (International Telecommunications Union). 
* ARGUMENTS: ptr is pointer to queue holding area to be CRCed
*            num is offset into buffer where to stop CRC calculation
* RETURNS: 2-byte CRC
*******************************************************************************/
uint16_t CDriver::calcCRC(uint8_t *ptr, uint32_t num)
{
    uint16_t crc = 0x1D0F; //non-augmented initial value equivalent to augmented initial value 0xFFFF
    for (uint32_t i = 0; i < num; i++)
    {
        crc ^= ptr[i] << 8;
        for(uint32_t j = 0; j < 8; j++) 
        {
            if(crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc = crc << 1;
            }
        } 
    }
    return crc;
}

string CDriver::Bytestohexstring(uint8_t* bytes,int bytelength)  
{  
  string str("");  
  string str2("0123456789ABCDEF");   
  for (int i = 0; i < bytelength; i++)
  {  
    int b = 0x0f & (bytes[i] >> 4);  
    str.append(1, str2.at(b));            
    b = 0x0f & bytes[i];  
    str.append(1, str2.at(b));  
    str += " ";
  }
  return str;  
}  
