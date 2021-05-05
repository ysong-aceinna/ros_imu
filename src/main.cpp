/*******************************************************
 * @file main.cpp
 * @author SongYang (ysong@aceinna.com)
 * @brief 
 * @date 2021-05-01
 * 
 * @copyright Copyright (c) 2021
 * 
*******************************************************/
#include "driver.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aceinna_imu_node");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); //Info  Debug   Warn, Error,Fatal,
    
    CDriver* pdriver = new CDriver(nh);
    pdriver->Start();
    // ros::spin();
    SAFEDELETE(pdriver);
    return 0;
}
