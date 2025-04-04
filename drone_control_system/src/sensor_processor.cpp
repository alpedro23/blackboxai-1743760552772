#include "drone_control_system/sensor_processor.h"

SensorProcessor::SensorProcessor() {
    imu_sub = nh.subscribe("/mavros/imu/data", 10, &SensorProcessor::imuCallback, this);
    gps_sub = nh.subscribe("/mavros/global_position/global", 10, &SensorProcessor::gpsCallback, this);
    state_sub = nh.subscribe("/mavros/state", 10, &SensorProcessor::stateCallback, this);
    fusion_pub = nh.advertise<drone_control_system::SensorFusion>("/sensor_fusion", 10);
}

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
        fused_data.orientation = imu->orientation;
        fused_data.angular_velocity = imu->angular_velocity;
        fused_data.linear_acceleration = imu->linear_acceleration;
        publishFusedData();
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps) {
        fused_data.latitude = gps->latitude;
        fused_data.longitude = gps->longitude;
        fused_data.altitude = gps->altitude;
        publishFusedData();
    }

    void stateCallback(const mavros_msgs::State::ConstPtr& state) {
        fused_data.armed = state->armed;
        fused_data.connected = state->connected;
        fused_data.mode = state->mode;
        publishFusedData();
    }

    void publishFusedData() {
        fused_data.header.stamp = ros::Time::now();
        fusion_pub.publish(fused_data);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_processor");
    SensorProcessor sensor_processor;
    ros::spin();
    return 0;
}