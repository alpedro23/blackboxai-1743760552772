#include "drone_control_system/servo_control.h"
#include <wiringPi.h>
#include <softPwm.h>

ServoControl::ServoControl() : gpio_pin(18), current_position(0), initialized(false) {
    servo_sub = nh.subscribe("/servo_commands", 10, &ServoControl::servoCallback, this);
    initializeGPIO();
}

void ServoControl::initializeGPIO() {
    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Failed to initialize WiringPi");
        return;
    }
    softPwmCreate(gpio_pin, 0, 200);
    initialized = true;
    ROS_INFO("Servo control initialized on GPIO %d", gpio_pin);
}

void ServoControl::servoCallback(const servo_msgs::ServoCommand::ConstPtr& cmd) {
    if (!initialized) {
        ROS_WARN("Servo control not initialized");
        return;
    }
    setServoPosition(cmd->position);
}

void ServoControl::setServoPosition(float position) {
    if (position < 0.0 || position > 1.0) {
        ROS_WARN("Invalid servo position: %f (must be 0.0-1.0)", position);
        return;
    }
    
    int pwm_value = 5 + (position * 20); // Convert to 5-25 (50-250Hz)
    softPwmWrite(gpio_pin, pwm_value);
    current_position = position;
    ROS_DEBUG("Set servo to position: %f (PWM: %d)", position, pwm_value);
}

void ServoControl::cleanupGPIO() {
    if (initialized) {
        softPwmWrite(gpio_pin, 0);
        initialized = false;
    }
}

ServoControl::~ServoControl() {
    cleanupGPIO();
}