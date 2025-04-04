#ifndef WAYPOINT_PROCESSOR_H
#define WAYPOINT_PROCESSOR_H

#include <ros/ros.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/Waypoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class WaypointProcessor {
public:
    WaypointProcessor();
    void waypointCallback(const mavros_msgs::WaypointList::ConstPtr& msg);
    void scanCallback(const sensor_msgs/LaserScan::ConstPtr& scan);

private:
    ros::NodeHandle nh;
    ros::Subscriber waypoint_sub;
    ros::Subscriber scan_sub;
    ros::Publisher setpoint_pub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    
    mavros_msgs::WaypointList current_waypoints;
    bool has_waypoints = false;
    
    void adjustForObstacles(const sensor_msgs::LaserScan::ConstPtr& scan);
    void publishAdjustedWaypoint();
};

#endif // WAYPOINT_PROCESSOR_H