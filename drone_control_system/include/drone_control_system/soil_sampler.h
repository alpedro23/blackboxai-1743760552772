#ifndef SOIL_SAMPLER_H
#define SOIL_SAMPLER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "servo_control.h"

class SoilSampler {
public:
    SoilSampler();
    void samplingCallback(const std_msgs::Bool::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sampling_sub_;
    ServoControl drill_servo_;
    ServoControl collector_servo_;
    
    void performSampling();
    void retractSampler();
};

#endif // SOIL_SAMPLER_H