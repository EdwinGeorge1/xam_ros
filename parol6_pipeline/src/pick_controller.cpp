#include "parol6_pipeline/pick_controller.hpp"
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <memory>
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace parol6_pipeline
{

PickController::PickController(const rclcpp::NodeOptions & options)
: Node("pick_controller", options),
  logger_(get_logger()),
  received_pose_(false),
  move_group_initialized_(false)
{
  RCLCPP_INFO(logger_, "PickController started. Waiting for /detected_box_pose and /start_picking trigger.");

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/detected_box_pose", 10,
    std::bind(&PickController::pose_callback, this, std::placeholders::_1));

  trigger_srv_ = create_service<std_srvs::srv::Trigger>(
    "start_picking",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      if (!received_pose_) {
        res->success = false;
        res->message = "No box pose received yet!";
        RCLCPP_WARN(logger_, "%s", res->message.c_str());
        return;
      }

      if (!move_group_initialized_)
        initialize_move_group();

      if (!move_group_) {
        res->success = false;
        res->message = "Failed to initialize MoveGroupInterface!";
        RCLCPP_ERROR(logger_, "%s", res->message.c_str());
        return;
      }

      // Set speed before execution
      double default_speed = 0.5;
      move_group_->setMaxVelocityScalingFactor(default_speed);
      move_group_->setMaxAccelerationScalingFactor(default_speed);
      
      if (execute_pick_sequence(latest_pose_)) {
        res->success = true;
        res->message = "Pick sequence completed!";
      } else {
        res->success = false;
        res->message = "Pick sequence failed! Check terminal logs for planning error.";
      }
    });
}

void PickController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_pose_ = *msg;
  received_pose_ = true;
  RCLCPP_INFO_THROTTLE(logger_, *this->get_clock(), 2000,
    "Detected box pose received from %s: (%.3f, %.3f, %.3f)",
    msg->header.frame_id.c_str(),
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void PickController::initialize_move_group()
{
  try {
    // Note: "lite6" is the name of the planning group in your SRDF
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "lite6");

    move_group_->setPlanningTime(10.0);
    // Increased attempts for the challenging raw pose target
    move_group_->setNumPlanningAttempts(15); 
    // Reference frame (usually the robot base)
    move_group_->setPoseReferenceFrame("link_base"); 

    move_group_initialized_ = true;
    RCLCPP_INFO(logger_, "MoveGroupInterface initialized for 'lite6'.");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "MoveGroupInterface initialization failed: %s", e.what());
    move_group_.reset();
  }
}

bool PickController::move_to_pose(const geometry_msgs::msg::PoseStamped& pose_msg)
{
  // Safety: check if MoveGroup is ready
  if (!move_group_) {
    RCLCPP_ERROR(logger_, "MoveGroupInterface is not initialized.");
    return false;
  }
  
  // *** RETAINED FIX: Explicitly specify the end-effector link to move ***
  const std::string target_link = "link_eef"; 
  
  // MoveIt will use the full pose (position + orientation) provided in pose_msg.pose
  move_group_->setPoseTarget(pose_msg.pose, target_link); 

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!success) {
    RCLCPP_ERROR(logger_, "Planning failed! Could not find path to target pose.");
    return false;
  }

  RCLCPP_INFO(logger_, "Planning success, executing...");
  auto result = move_group_->execute(plan);
  return (result == moveit::core::MoveItErrorCode::SUCCESS);
}

bool PickController::execute_pick_sequence(const geometry_msgs::msg::PoseStamped& box_pose)
{
  // pick_pose now contains the RAW coordinates (position and orientation) from the ArUco detector.
  geometry_msgs::msg::PoseStamped pick_pose = box_pose;

  // *** ORIENTATION MANIPULATION REMOVED: Robot uses the raw quaternion from box_pose. ***
  
  // *** Z-OFFSET REMOVED: Robot uses the raw Z position from box_pose. ***

  // 1. MOVE DIRECTLY TO RAW TARGET POSE 
  RCLCPP_INFO(logger_, "1. Moving directly to raw target box pose: (%.3f, %.3f, %.3f)",
      pick_pose.pose.position.x, pick_pose.pose.position.y, pick_pose.pose.position.z);
  
  if (!move_to_pose(pick_pose)) return false;

  RCLCPP_INFO(logger_, "Target position reached. Simulating gripper close (2s)...");
  // Simulate gripper closing
  rclcpp::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(logger_, "Pick sequence completed successfully!");
  return true;
}

}  // namespace parol6_pipeline

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Using MultiThreadedExecutor is CRITICAL for MoveIt!
  rclcpp::executors::MultiThreadedExecutor executor; 
  auto node = std::make_shared<parol6_pipeline::PickController>();
  executor.add_node(node);
  executor.spin(); 
  
  rclcpp::shutdown();
  return 0;
}