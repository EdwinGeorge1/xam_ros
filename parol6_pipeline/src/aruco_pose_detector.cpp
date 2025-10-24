#include "parol6_pipeline/aruco_pose_detector.hpp" 
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp> // Needed for TransformStamped message

ArucoPoseDetector::ArucoPoseDetector()
: Node("aruco_pose_detector")
{
    // --- Parameter Declaration and Retrieval ---
    this->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
    this->declare_parameter<std::string>("base_frame", "link_base");
    this->declare_parameter<int>("marker_id", 0); // Default set to 0
    this->declare_parameter<double>("marker_length", 0.05);
    this->declare_parameter<double>("offset_roll", 0.0);
    this->declare_parameter<double>("offset_pitch", 0.0);
    this->declare_parameter<double>("offset_yaw", 0.0);
    this->declare_parameter<double>("offset_z", 0.0); // <-- DECLARE NEW PARAMETER

    camera_frame_ = this->get_parameter("camera_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    marker_id_ = this->get_parameter("marker_id").as_int();
    marker_length_ = this->get_parameter("marker_length").as_double();
    offset_roll_ = this->get_parameter("offset_roll").as_double();
    offset_pitch_ = this->get_parameter("offset_pitch").as_double();
    offset_yaw_ = this->get_parameter("offset_yaw").as_double();
    offset_z_ = this->get_parameter("offset_z").as_double(); // <-- RETRIEVE NEW PARAMETER
    
    RCLCPP_INFO(this->get_logger(), "Configured to look for Marker ID: %d with Z-Offset: %.3f m", marker_id_, offset_z_);

    // --- TF Listener and Buffer Setup ---
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // **NEW:** Initialize the TF Broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // --- ArUco and ROS 2 Componenets Setup ---
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/color/image_raw", 10, std::bind(&ArucoPoseDetector::rgbCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/aligned_depth_to_color/image_raw", 10, std::bind(&ArucoPoseDetector::depthCallback, this, std::placeholders::_1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detected_box_pose", 10);

    cv::namedWindow("Aruco Detection", cv::WINDOW_NORMAL); 

    RCLCPP_INFO(this->get_logger(), "Aruco Pose Detector initialized with depth and visualization.");
}

void ArucoPoseDetector::depthCallback(const sensor_msgs::msg::Image::SharedPtr depth_msg)
{
    try {
        latest_depth_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        RCLCPP_DEBUG(this->get_logger(), "Depth image received: %dx%d", latest_depth_.cols, latest_depth_.rows);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Depth cv_bridge exception: %s", e.what());
    }
}

void ArucoPoseDetector::rgbCallback(const sensor_msgs::msg::Image::SharedPtr rgb_msg)
{
    if (latest_depth_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for depth image on /aligned_depth_to_color/image_raw...");
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "RGB cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    RCLCPP_DEBUG(this->get_logger(), "RGB image size: %dx%d", cv_ptr->image.cols, cv_ptr->image.rows);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids); 

    RCLCPP_DEBUG(this->get_logger(), "Detected %zu markers in total.", ids.size());

    if (ids.empty()) {
        cv::putText(cv_ptr->image, "No ArUco marker detected", cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,255), 2);
        cv::imshow("Aruco Detection", cv_ptr->image);
        cv::waitKey(1);
        return;
    }

    for (size_t i = 0; i < ids.size(); ++i)
    {
        RCLCPP_INFO(this->get_logger(), "Detected marker ID: %d", ids[i]);
        if (ids[i] != marker_id_) {
            RCLCPP_DEBUG(this->get_logger(), "Skipping marker ID %d (looking for %d).", ids[i], marker_id_);
            continue;
        }

        RCLCPP_DEBUG(this->get_logger(), "Marker ID %d matches target ID %d. Proceeding with pose calculation.", ids[i], marker_id_);

        // --- Visuals ---
        cv::aruco::drawDetectedMarkers(cv_ptr->image, std::vector<std::vector<cv::Point2f>>{corners[i]}, std::vector<int>{ids[i]});

        // --- Calculate Center and Depth Checks ---
        cv::Point2f center(0,0);
        for (auto &pt : corners[i]) center += pt;
        center /= 4.0;

        int cx = int(center.x), cy = int(center.y);
        RCLCPP_DEBUG(this->get_logger(), "Marker center pixel coordinates: (%d, %d)", cx, cy);

        // 1. Check bounds
        if (cx < 0 || cx >= latest_depth_.cols || cy < 0 || cy >= latest_depth_.rows) {
            RCLCPP_WARN(this->get_logger(), "FAIL: Marker center outside depth image bounds: (%d, %d). Check image alignment or move marker.", cx, cy);
            continue;
        }
        RCLCPP_DEBUG(this->get_logger(), "Center pixel is within depth image bounds.");


        // 2. Check depth validity
        uint16_t depth_mm = latest_depth_.at<uint16_t>(cy, cx);
        double z = depth_mm / 1000.0;
        
        if (z <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "FAIL: Invalid depth reading at (%d, %d). Depth (mm): %u, Converted Z (m): %.4f. Marker may be too close/far, or surface is unsuitable.", cx, cy, depth_mm, z);
            continue;
        }

        RCLCPP_DEBUG(this->get_logger(), "SUCCESS: Valid depth Z=%.3f m obtained.", z);

        // --- Calculate X, Y in Camera Frame ---
        double x = (center.x - c_x) * z / fx;
        double y = (center.y - c_y) * z / fy;

        RCLCPP_DEBUG(this->get_logger(), "Marker position in %s: x=%.3f, y=%.3f, z=%.3f", camera_frame_.c_str(), x, y, z);

        // --- Create Pose in Camera Frame (with rotation offsets) ---
        tf2::Quaternion q_offset;
        q_offset.setRPY(offset_roll_ * M_PI / 180.0,
                        offset_pitch_ * M_PI / 180.0,
                        offset_yaw_ * M_PI / 180.0);

        geometry_msgs::msg::PoseStamped pose_camera;
        pose_camera.header.frame_id = camera_frame_;
        pose_camera.header.stamp = rgb_msg->header.stamp;
        pose_camera.pose.position.x = x;
        pose_camera.pose.position.y = y;
        pose_camera.pose.position.z = z;
        pose_camera.pose.orientation = tf2::toMsg(q_offset);

        // --- TF TRANSFORM CHECK (Camera Frame -> Base Frame) ---
        geometry_msgs::msg::PoseStamped pose_base;
        try {
            RCLCPP_DEBUG(this->get_logger(), "Attempting TF lookup: target frame '%s', source frame '%s'", base_frame_.c_str(), camera_frame_.c_str());

            auto transform = tf_buffer_->lookupTransform(base_frame_, camera_frame_, rclcpp::Time(0),
                                                         rclcpp::Duration::from_seconds(0.5));
            
            RCLCPP_DEBUG(this->get_logger(), "TF lookup succeeded. Applying transform.");
            tf2::doTransform(pose_camera, pose_base, transform);
            RCLCPP_DEBUG(this->get_logger(), "Pose successfully transformed to %s.", base_frame_.c_str());

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "FAIL: TF lookup failed from %s to %s: %s", camera_frame_.c_str(), base_frame_.c_str(), ex.what());
            continue; 
        }

        // --- APPLY Z-OFFSET in Base Frame ---
        // This moves the pose's Z coordinate by the configured offset_z_ (e.g., marker thickness).
        pose_base.pose.position.z += offset_z_; // <-- APPLY Z-AXIS OFFSET

        // --- Publish PoseStamped Message ---
        pose_pub_->publish(pose_base);


        // --- TF BROADCASTING (for RViz visualization) ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = pose_base.header.stamp; // Use the same timestamp as the pose message
        t.header.frame_id = base_frame_; // Parent frame
        t.child_frame_id = "detected_aruco_marker"; // The new child frame name in RViz

        // Copy position and rotation data
        t.transform.translation.x = pose_base.pose.position.x;
        t.transform.translation.y = pose_base.pose.position.y;
        t.transform.translation.z = pose_base.pose.position.z;
        t.transform.rotation = pose_base.pose.orientation;

        // Send the transform
        tf_broadcaster_->sendTransform(t);
        RCLCPP_DEBUG(this->get_logger(), "Broadcasted TF frame 'detected_aruco_marker'.");


        // --- Log Final Pose ---
        tf2::Quaternion q_base;
        tf2::fromMsg(pose_base.pose.orientation, q_base);
        tf2::Matrix3x3 m(q_base);
        double roll_rad, pitch_rad, yaw_rad;
        m.getRPY(roll_rad, pitch_rad, yaw_rad); 

        double roll_deg = roll_rad * 180.0 / M_PI;
        double pitch_deg = pitch_rad * 180.0 / M_PI;
        double yaw_deg = yaw_rad * 180.0 / M_PI;

        RCLCPP_INFO(this->get_logger(), 
                    "Published Marker %d Pose in %s: X=%.3f Y=%.3f Z=%.3f | R=%.2f P=%.2f Y=%.2f (deg)", 
                    marker_id_, base_frame_.c_str(), 
                    pose_base.pose.position.x, pose_base.pose.position.y, pose_base.pose.position.z,
                    roll_deg, pitch_deg, yaw_deg);

        break; // Only process first matching marker
    }

    cv::imshow("Aruco Detection", cv_ptr->image);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoPoseDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}