#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp> // Includes dictionary and detection types
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h> 

class ArucoPoseDetector : public rclcpp::Node
{
public:
    ArucoPoseDetector();

private:
    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr rgb_msg);
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr depth_msg);

    // ROS 2 components
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; 

    // OpenCV/ArUco components 
    cv::Mat latest_depth_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    // Parameters
    std::string camera_frame_;
    std::string base_frame_;
    int marker_id_;
    double marker_length_;
    double offset_roll_;
    double offset_pitch_;
    double offset_yaw_;
    double offset_z_; // <-- NEW PARAMETER

    // Camera Intrinsics 
    const double fx = 640.5098;
    const double fy = 640.5098;
    const double c_x = 640.0;
    const double c_y = 360.0;
};