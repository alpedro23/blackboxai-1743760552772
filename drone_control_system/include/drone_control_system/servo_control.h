#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <servo_msgs/ServoCommand.h>

class ServoControl {
public:
    ServoControl();
    void servoCallback(const servo_msgs::ServoCommand::ConstPtr& cmd);
    void setServoPosition(float position);

private:
    ros::NodeHandle nh;
    ros::Subscriber servo_sub;
    int gpio_pin;
    float current_position;
    bool initialized;
    
    void initializeGPIO();
    void cleanupGPIO();
};

#endif // SERVO_CONTROL_H