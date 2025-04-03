#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include "sobits_interfaces/srv/move_hand_to_target_tf.hpp"
#include "sobits_interfaces/srv/move_hand_to_target_coord.hpp"
#include <memory>
#include <string>
#include <chrono>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <vector>
#include <thread> // std::this_thread::sleep_for用

class MoveHandService : public rclcpp::Node
{
public:
  MoveHandService() : Node("move_hand_service")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    move_hand_to_target_tf_srv_ = this->create_service<sobits_interfaces::srv::MoveHandToTargetTF>(
      "move_hand_to_target_tf",
      std::bind(&MoveHandService::moveHandToTargetTFCallback, this, std::placeholders::_1, std::placeholders::_2));

    move_hand_to_target_coord_srv_ = this->create_service<sobits_interfaces::srv::MoveHandToTargetCoord>(
      "move_hand_to_target_coord",
      std::bind(&MoveHandService::moveHandToTargetCoordCallback, this, std::placeholders::_1, std::placeholders::_2));

    this->declare_parameter("velocity_scaling_factor", 0.5);
    this->declare_parameter("acceleration_scaling_factor", 0.5);
    this->declare_parameter("orientation_yaw_offset", M_PI);
    this->declare_parameter("orientation_pitch", -M_PI / 2.0);
    this->declare_parameter("retry_attempts", 3);
    this->declare_parameter("retry_delay", 1.0);

    RCLCPP_INFO(this->get_logger(), "MoveHandServiceが起動しました");
  }

  void init(std::shared_ptr<rclcpp::Node> node)
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node, "active_hand", tf_buffer_,
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(0.1))));

    move_group_->setMaxVelocityScalingFactor(this->get_parameter("velocity_scaling_factor").as_double());
    move_group_->setMaxAccelerationScalingFactor(this->get_parameter("acceleration_scaling_factor").as_double());

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
    double yaw_offset = this->get_parameter("orientation_yaw_offset").as_double();
    double pitch = this->get_parameter("orientation_pitch").as_double();
    double yaw = std::atan2(target_pose.position.y, target_pose.position.x) + yaw_offset;
    tf2::Quaternion q;
    q.setRPY(0.0, pitch, yaw);
    geometry_msgs::msg::Pose modified_pose = target_pose;
    modified_pose.orientation.x = q.x();
    modified_pose.orientation.y = q.y();
    modified_pose.orientation.z = q.z();
    modified_pose.orientation.w = q.w();
    return modified_pose;
  }

  void moveHandToTargetTFCallback(
      const std::shared_ptr<sobits_interfaces::srv::MoveHandToTargetTF::Request> request,
      std::shared_ptr<sobits_interfaces::srv::MoveHandToTargetTF::Response> response)
  {
    // 1. TF変換の処理 (request->tf_differential から target_pose を計算)
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = request->tf_differential.transform.translation.x;
    target_pose.position.y = request->tf_differential.transform.translation.y;
    target_pose.position.z = request->tf_differential.transform.translation.z;
    target_pose.orientation = request->tf_differential.transform.rotation;

    target_pose = modifyPoseForOrientation(target_pose);

    int retry_attempts = this->get_parameter("retry_attempts").as_int();
    double retry_delay = this->get_parameter("retry_delay").as_double();

    // 2. MoveItによる計画と実行 (最大retry_attempts回まで再試行)
    for (int i = 0; i < retry_attempts; ++i)
    {
      move_group_->setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto move_result = move_group_->plan(plan);

      if (move_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        std::vector<double> joint_values = plan.trajectory_.joint_trajectory.points.back().positions;
        response->target_joint_rad = joint_values;
        response->target_joint_names = move_group_->getJointNames();

        auto move_exec_result = move_group_->move();

        if (move_exec_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          response->success = true;
          response->message = "MoveHandToTargetTF succeeded.";
          response->move_pose = move_group_->getCurrentPose().pose;
          return; // 成功したらループを抜ける
        }
        else
        {
          response->success = false;
          response->message = "MoveHandToTargetTF execution failed.";
          RCLCPP_WARN(this->get_logger(), "MoveHandToTargetTF execution failed. Retrying...");
        }
      }
      else
      {
        response->success = false;
        response->message = "MoveHandToTargetTF planning failed.";
        RCLCPP_WARN(this->get_logger(), "MoveHandToTargetTF planning failed. Retrying...");
      }
      // レスポンスを送信
      response->success = false;
      response->message = "Retry attempt: " + std::to_string(i + 1);
      RCLCPP_INFO(this->get_logger(), response->message.c_str()); // c_str()を追加
      std::this_thread::sleep_for(std::chrono::duration<double>(retry_delay));
    }
    RCLCPP_ERROR(this->get_logger(), ("MoveHandToTargetTF failed after " + std::to_string(retry_attempts) + " attempts.").c_str()); // c_str()を追加
    response->success = false;
    response->message = "MoveHandToTargetTF failed after " + std::to_string(retry_attempts) + " attempts.";
  }

  void moveHandToTargetCoordCallback(
      const std::shared_ptr<sobits_interfaces::srv::MoveHandToTargetCoord::Request> request,
      std::shared_ptr<sobits_interfaces::srv::MoveHandToTargetCoord::Response> response)
  {
    // 1. target_coord から target_pose を作成
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = request->target_coord.transform.translation.x;
    target_pose.position.y = request->target_coord.transform.translation.y;
    target_pose.position.z = request->target_coord.transform.translation.z;
    target_pose.orientation = request->target_coord.transform.rotation;

    target_pose = modifyPoseForOrientation(target_pose);

    int retry_attempts = this->get_parameter("retry_attempts").as_int();
    double retry_delay = this->get_parameter("retry_delay").as_double();

    // 2. MoveItによる計画と実行 (最大retry_attempts回まで再試行)
    for (int i = 0; i < retry_attempts; ++i)
    {
      move_group_->setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto move_result = move_group_->plan(plan);

      if (move_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        std::vector<double> joint_values = plan.trajectory_.joint_trajectory.points.back().positions;
        response->target_joint_rad = joint_values;
        response->target_joint_names = move_group_->getJointNames();

        auto move_exec_result = move_group_->move();

        if (move_exec_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          response->success = true;
          response->message = "MoveHandToTargetCoord succeeded.";
          response->move_pose = move_group_->getCurrentPose().pose;
          return; // 成功したらループを抜ける
        }
        else
        {
          response->success = false;
          response->message = "MoveHandToTargetCoord execution failed.";
          RCLCPP_WARN(this->get_logger(), "MoveHandToTargetCoord execution failed. Retrying...");
        }
      }
      else
      {
        response->success = false;
        response->message = "MoveHandToTargetCoord planning failed.";
        RCLCPP_WARN(this->get_logger(), "MoveHandToTargetCoord planning failed. Retrying...");
      }
      // レスポンスを送信
      response->success = false;
      response->message = "Retry attempt: " + std::to_string(i + 1);
      RCLCPP_INFO(this->get_logger(), response->message.c_str()); // c_str()を追加
      std::this_thread::sleep_for(std::chrono::duration<double>(retry_delay));
    }
    RCLCPP_ERROR(this->get_logger(), ("MoveHandToTargetCoord failed after " + std::to_string(retry_attempts) + " attempts.").c_str()); // c_str()を追加
    response->success = false;
    response->message = "MoveHandToTargetCoord failed after " + std::to_string(retry_attempts) + " attempts.";
  }

  rclcpp::Service<sobits_interfaces::srv::MoveHandToTargetTF>::SharedPtr move_hand_to_target_tf_srv_;
  rclcpp::Service<sobits_interfaces::srv::MoveHandToTargetCoord>::SharedPtr move_hand_to_target_coord_srv_;
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