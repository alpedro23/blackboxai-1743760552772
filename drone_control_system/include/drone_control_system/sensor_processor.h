#ifndef SENSOR_PROCESSOR_H
#define SENSOR_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <mavros_msgs/State.h>
#include <drone_control_system/SensorFusion.h>

class SensorProcessor {
public:
    SensorProcessor();
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps);
    void stateCallback(const mavros_msgs::State::ConstPtr& state);
    void publishFusedData();

private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber state_sub;
    ros::Publisher fusion_pub;
    
    drone_control_system::SensorFusion fused_data;
};

#endif // SENSOR_PROCESSOR_H