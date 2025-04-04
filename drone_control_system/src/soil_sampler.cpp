#include "drone_control_system/soil_sampler.h"

SoilSampler::SoilSampler() : 
    nh_("~"),
    drill_servo_(18),  // GPIO pin for drill servo
    collector_servo_(19) {  // GPIO pin for collector servo
    
    sampling_sub_ = nh_.subscribe("/sampling/command", 1, 
                                &SoilSampler::samplingCallback, this);
}

void SoilSampler::samplingCallback(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data) {
        performSampling();
    } else {
        retractSampler();
    }
}

void SoilSampler::performSampling() {
    // Extend drill
    drill_servo_.setServoPosition(0.9);  // 90% extension
    ros::Duration(2.0).sleep();
    
    // Activate collector
    collector_servo_.setServoPosition(0.8);  // 80% open
    ros::Duration(1.0).sleep();
    
    // Retract drill while collecting
    drill_servo_.setServoPosition(0.1);  // 10% retracted
    ros::Duration(2.0).sleep();
    
    // Close collector
    collector_servo_.setServoPosition(0.0);  // Fully closed
}

void SoilSampler::retractSampler() {
    drill_servo_.setServoPosition(0.0);  // Fully retracted
    collector_servo_.setServoPosition(0.0);  // Fully closed
}