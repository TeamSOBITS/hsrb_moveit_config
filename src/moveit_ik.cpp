#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

#include "hsrb_moveit_interfaces/srv/move_hand_to_target_tf.hpp"
#include "hsrb_moveit_interfaces/srv/move_hand_to_target_coord.hpp"
#include <memory>
#include <string>
#include <chrono>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <vector>

class MoveHandService : public rclcpp::Node
{
public:
  MoveHandService() : Node("move_hand_service")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    move_hand_to_target_tf_srv_ = this->create_service<hsrb_moveit_interfaces::srv::MoveHandToTargetTf>(
      "move_hand_to_target_tf",
      std::bind(&MoveHandService::moveHandToTargetTfCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    move_hand_to_target_coord_srv_ = this->create_service<hsrb_moveit_interfaces::srv::MoveHandToTargetCoord>(
      "move_hand_to_target_coord",
      std::bind(&MoveHandService::moveHandToTargetCoordCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "MoveHandServiceが起動しました");
  }

  void init(std::shared_ptr<rclcpp::Node> node)
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node, "active_hand", tf_buffer_,
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(0.1)))
    );
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);

    loadJointConstraints(node);
  }

private:
  void loadJointConstraints(std::shared_ptr<rclcpp::Node> node)
  {
    std::vector<std::string> joint_names = {
        "arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
        "wrist_flex_joint", "wrist_roll_joint"};

    joint_constraints_.reset(new moveit_msgs::msg::Constraints());

    for (const auto &joint_name : joint_names)
    {
      std::string param_prefix = "joint_constraints." + joint_name;

      node->declare_parameter(param_prefix + ".position", 0.0);
      node->declare_parameter(param_prefix + ".tolerance_above", 0.1);
      node->declare_parameter(param_prefix + ".tolerance_below", 0.1);
      node->declare_parameter(param_prefix + ".weight", 0.0);

      double position = node->get_parameter(param_prefix + ".position").as_double();
      double tolerance_above = node->get_parameter(param_prefix + ".tolerance_above").as_double();
      double tolerance_below = node->get_parameter(param_prefix + ".tolerance_below").as_double();
      double weight = node->get_parameter(param_prefix + ".weight").as_double();

      moveit_msgs::msg::JointConstraint constraint;
      constraint.joint_name = joint_name;
      constraint.position = position;
      constraint.tolerance_above = tolerance_above;
      constraint.tolerance_below = tolerance_below;
      constraint.weight = weight;

      joint_constraints_->joint_constraints.push_back(constraint);
    }
    move_group_->setPathConstraints(*joint_constraints_);
    RCLCPP_INFO(this->get_logger(), "Joint constraints loaded.");
  }

  geometry_msgs::msg::Pose modifyPoseForOrientation(const geometry_msgs::msg::Pose &target_pose)
  {
    double yaw = std::atan2(target_pose.position.y, target_pose.position.x) + M_PI;
    tf2::Quaternion q;
    q.setRPY(0.0, -M_PI / 2.0, yaw);
    geometry_msgs::msg::Pose modified_pose = target_pose;
    modified_pose.orientation.x = q.x();
    modified_pose.orientation.y = q.y();
    modified_pose.orientation.z = q.z();
    modified_pose.orientation.w = q.w();
    return modified_pose;
  }

  void moveHandToTargetTfCallback(
      const std::shared_ptr<hsrb_moveit_interfaces::srv::MoveHandToTargetTf::Request> request,
      std::shared_ptr<hsrb_moveit_interfaces::srv::MoveHandToTargetTf::Response> response)
  {
    std::string object_name = request->object_name;
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      transformStamped = tf_buffer_->lookupTransform("base_footprint", object_name, rclcpp::Time(0), tf2::durationFromSec(1.0));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "TFの取得に失敗: %s", ex.what());
      response->success = false;
      return;
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = transformStamped.transform.translation.x;
    target_pose.position.y = transformStamped.transform.translation.y;
    target_pose.position.z = transformStamped.transform.translation.z;
    target_pose.orientation = transformStamped.transform.rotation;

    geometry_msgs::msg::Pose modified_pose = modifyPoseForOrientation(target_pose);

    move_group_->setPoseTarget(modified_pose);
    auto move_result = move_group_->move();
    if (move_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      response->success = true;
    }
    else
    {
      response->success = false;
    }
  }

  void moveHandToTargetCoordCallback(
      const std::shared_ptr<hsrb_moveit_interfaces::srv::MoveHandToTargetCoord::Request> request,
      std::shared_ptr<hsrb_moveit_interfaces::srv::MoveHandToTargetCoord::Response> response)
  {
    geometry_msgs::msg::Pose target_pose = request->pose;

    geometry_msgs::msg::Pose modified_pose = modifyPoseForOrientation(target_pose);

    move_group_->setPoseTarget(modified_pose);
    auto move_result = move_group_->move();
    if (move_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      response->success = true;
    }
    else
    {
      response->success = false;
    }
  }

  rclcpp::Service<hsrb_moveit_interfaces::srv::MoveHandToTargetTf>::SharedPtr move_hand_to_target_tf_srv_;
  rclcpp::Service<hsrb_moveit_interfaces::srv::MoveHandToTargetCoord>::SharedPtr move_hand_to_target_coord_srv_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<moveit_msgs::msg::Constraints> joint_constraints_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveHandService>();
  node->init(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}