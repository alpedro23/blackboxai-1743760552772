#ifndef OBJECT_TRACKER_H
#define OBJECT_TRACKER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ObjectTracker {
public:
    ObjectTracker();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher target_pub_;
    
    cv::Ptr<cv::Tracker> tracker_;
    bool tracking_initialized_;
    
    void initTracker(const cv::Mat& frame);
};

#endif // OBJECT_TRACKER_H