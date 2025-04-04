#include "drone_control_system/object_tracker.h"
#include <opencv2/tracking.hpp>

ObjectTracker::ObjectTracker() : 
    nh_("~"),
    tracking_initialized_(false) {
    image_sub_ = nh_.subscribe("/camera/image_raw", 1, 
                             &ObjectTracker::imageCallback, this);
    target_pub_ = nh_.advertise<geometry_msgs::Point>("/tracking/target", 1);
}

void ObjectTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        if(!tracking_initialized_) {
            initTracker(frame);
            return;
        }

        cv::Rect bbox;
        if(tracker_->update(frame, bbox)) {
            geometry_msgs::Point target;
            target.x = bbox.x + bbox.width/2;
            target.y = bbox.y + bbox.height/2;
            target_pub_.publish(target);
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV bridge error: %s", e.what());
    }
}

void ObjectTracker::initTracker(const cv::Mat& frame) {
    // Initialize with default ROI (can be made configurable)
    cv::Rect2d roi(frame.cols/4, frame.rows/4, frame.cols/2, frame.rows/2);
    tracker_ = cv::TrackerKCF::create();
    tracker_->init(frame, roi);
    tracking_initialized_ = true;
}