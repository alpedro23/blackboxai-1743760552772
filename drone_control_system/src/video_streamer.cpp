#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <mavros_msgs/Mavlink.h>

class VideoStreamer {
public:
    VideoStreamer() : nh_("~") {
        // Initialize camera
        cap_.open(0); // Use default camera
        if(!cap_.isOpened()) {
            ROS_ERROR("Failed to open video device");
            return;
        }

        // Setup publisher
        image_transport::ImageTransport it(nh_);
        pub_ = it.advertise("camera/image", 1);

        // Setup MAVLink publisher for QGC
        mavlink_pub_ = nh_.advertise<mavros_msgs::Mavlink>("mavlink/to", 10);
    }

    void stream() {
        cv::Mat frame;
        while(ros::ok()) {
            cap_ >> frame;
            if(frame.empty()) {
                ROS_WARN("Empty frame received");
                continue;
            }

            // Convert to ROS message
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
                std_msgs::Header(), "bgr8", frame).toImageMsg();

            // Publish image
            pub_.publish(msg);

            // Convert to MAVLink and send to QGC
            sendMavlinkVideoFrame(frame);

            ros::spinOnce();
        }
    }

private:
    void sendMavlinkVideoFrame(const cv::Mat& frame) {
        // Implementation for MAVLink video streaming
        // Would need to convert frame to MAVLink VIDEO_STREAM_INFORMATION message
        // This is a simplified placeholder
        mavros_msgs::Mavlink msg;
        // ... fill MAVLink message ...
        mavlink_pub_.publish(msg);
    }

    ros::NodeHandle nh_;
    image_transport::Publisher pub_;
    ros::Publisher mavlink_pub_;
    cv::VideoCapture cap_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_streamer");
    VideoStreamer streamer;
    streamer.stream();
    return 0;
}