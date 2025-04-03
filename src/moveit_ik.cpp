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
#include <thread>

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
    this->declare_parameter("retry_attempts", 5);
    this->declare_parameter("retry_delay", 2.0);
    this->declare_parameter("base_frame", "base_footprint");

    velocity_scaling_factor_ = this->get_parameter("velocity_scaling_factor").as_double();
    acceleration_scaling_factor_ = this->get_parameter("acceleration_scaling_factor").as_double();
    orientation_yaw_offset_ = this->get_parameter("orientation_yaw_offset").as_double();
    orientation_pitch_ = this->get_parameter("orientation_pitch").as_double();
    retry_attempts_ = this->get_parameter("retry_attempts").as_int();
    retry_delay_ = this->get_parameter("retry_delay").as_double();
    base_frame_ = this->get_parameter("base_frame").as_string();

    RCLCPP_INFO(this->get_logger(), "MoveHandServiceが起動しました");
  }

  void init(std::shared_ptr<rclcpp::Node> node)
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node, "active_hand", tf_buffer_,
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(0.1))));

    move_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);

    loadJointConstraints(node);
  }

private:
  double velocity_scaling_factor_;
  double acceleration_scaling_factor_;
  double orientation_yaw_offset_;
  double orientation_pitch_;
  int retry_attempts_;
  double retry_delay_;
  std::string base_frame_;

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
    double yaw = std::atan2(target_pose.position.y, target_pose.position.x) + orientation_yaw_offset_;
    tf2::Quaternion q;
    q.setRPY(0.0, orientation_pitch_, yaw);
    geometry_msgs::msg::Pose modified_pose = target_pose;
    modified_pose.orientation.x = q.x();
    modified_pose.orientation.y = q.y();
    modified_pose.orientation.z = q.z();
    modified_pose.orientation.w = q.w();
    return modified_pose;
  }

  template <typename ResponseType>
  bool executeMove(const geometry_msgs::msg::Pose &target_pose, ResponseType &response)
  {
    for (int i = 0; i < retry_attempts_; ++i)
    {
      move_group_->setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto move_result = move_group_->plan(plan);

      if (move_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        std::vector<double> joint_values = plan.trajectory_.joint_trajectory.points.back().positions;
        response.target_joint_rad = joint_values;
        response.target_joint_names = move_group_->getJointNames();

        auto move_exec_result = move_group_->move();

        if (move_exec_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          response.success = true;
          response.move_pose = move_group_->getCurrentPose().pose;
          return true;
        }
        else
        {
          response.success = false;
          response.message = "Move execution failed.";
          RCLCPP_WARN(this->get_logger(), response.message.c_str());
        }
      }
      else
      {
        response.success = false;
        response.message = "Move planning failed.";
        RCLCPP_WARN(this->get_logger(), response.message.c_str());
      }
      response.success = false;
      response.message = "Retry attempt: " + std::to_string(i + 1);
      RCLCPP_INFO(this->get_logger(), response.message.c_str());
      std::this_thread::sleep_for(std::chrono::duration<double>(retry_delay_));
    }
    response.success = false;
    response.message = "Move failed after " + std::to_string(retry_attempts_) + " attempts.";
    RCLCPP_ERROR(this->get_logger(), response.message.c_str());
    return false;
  }

  void moveHandToTargetTFCallback(
      const std::shared_ptr<sobits_interfaces::srv::MoveHandToTargetTF::Request> request,
      std::shared_ptr<sobits_interfaces::srv::MoveHandToTargetTF::Response> response)
  {
    std::string target_frame = request->target_frame;
    geometry_msgs::msg::TransformStamped tf_differential = request->tf_differential;
    geometry_msgs::msg::Pose target_pose_base_frame;

    try
    {
      geometry_msgs::msg::TransformStamped target_transform_stamped =
        tf_buffer_->lookupTransform(base_frame_, target_frame, tf2::TimePointZero);

      tf2::Transform target_transform;
      tf2::fromMsg(target_transform_stamped.transform, target_transform);

      tf2::Transform differential_transform;
      tf2::fromMsg(tf_differential.transform, differential_transform);

      tf2::Transform final_transform = target_transform * differential_transform;

      tf2::toMsg(final_transform, target_pose_base_frame);

      RCLCPP_INFO(this->get_logger(), "ターゲットフレーム '%s' の座標 (変換適用後、base_footprint基準):", target_frame.c_str());
      RCLCPP_INFO(this->get_logger(), "  位置: x=%.3f, y=%.3f, z=%.3f",
                   target_pose_base_frame.position.x, target_pose_base_frame.position.y, target_pose_base_frame.position.z);
      RCLCPP_INFO(this->get_logger(), "  回転: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                   target_pose_base_frame.orientation.x, target_pose_base_frame.orientation.y,
                   target_pose_base_frame.orientation.z, target_pose_base_frame.orientation.w);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "TF lookup failed (base_footprint -> target): %s", ex.what());
      response->success = false;
      response->message = "TF lookup failed (base_footprint -> target): " + std::string(ex.what());
      return;
    }

    geometry_msgs::msg::Pose target_pose_odom_compensated = target_pose_base_frame;

    try
    {
      geometry_msgs::msg::TransformStamped base_footprint_to_odom_transform_stamped =
        tf_buffer_->lookupTransform("odom", base_frame_, tf2::TimePointZero);

      target_pose_odom_compensated.position.x += base_footprint_to_odom_transform_stamped.transform.translation.x;
      target_pose_odom_compensated.position.y += base_footprint_to_odom_transform_stamped.transform.translation.y;

      RCLCPP_INFO(this->get_logger(), "ターゲット座標 (odomのオフセットを加算後):");
      RCLCPP_INFO(this->get_logger(), "  位置: x=%.3f, y=%.3f, z=%.3f",
                   target_pose_odom_compensated.position.x, target_pose_odom_compensated.position.y, target_pose_odom_compensated.position.z);
      RCLCPP_INFO(this->get_logger(), "  回転: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                   target_pose_odom_compensated.orientation.x, target_pose_odom_compensated.orientation.y,
                   target_pose_odom_compensated.orientation.z, target_pose_odom_compensated.orientation.w);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed (odom -> base_footprint), odom offset not applied: %s", ex.what());
    }

    geometry_msgs::msg::Pose modified_pose = modifyPoseForOrientation(target_pose_odom_compensated);

    if (executeMove(modified_pose, *response))
    {
      response->message = "MoveHandToTargetTF succeeded.";
    }
    else
    {
      response->message = "MoveHandToTargetTF failed.";
    }
  }

  void moveHandToTargetCoordCallback(
      const std::shared_ptr<sobits_interfaces::srv::MoveHandToTargetCoord::Request> request,
      std::shared_ptr<sobits_interfaces::srv::MoveHandToTargetCoord::Response> response)
  {
    geometry_msgs::msg::Pose target_pose_base_frame;
    target_pose_base_frame.position.x = request->target_coord.transform.translation.x;
    target_pose_base_frame.position.y = request->target_coord.transform.translation.y;
    target_pose_base_frame.position.z = request->target_coord.transform.translation.z;
    target_pose_base_frame.orientation = request->target_coord.transform.rotation;

    geometry_msgs::msg::Pose target_pose_odom_compensated = target_pose_base_frame;

    try
    {
      geometry_msgs::msg::TransformStamped base_footprint_to_odom_transform_stamped =
        tf_buffer_->lookupTransform("odom", base_frame_, tf2::TimePointZero);

      target_pose_odom_compensated.position.x += base_footprint_to_odom_transform_stamped.transform.translation.x;
      target_pose_odom_compensated.position.y += base_footprint_to_odom_transform_stamped.transform.translation.y;

      RCLCPP_INFO(this->get_logger(), "目標座標 (odomのオフセットを加算後):");
      RCLCPP_INFO(this->get_logger(), "  位置: x=%.3f, y=%.3f, z=%.3f",
                   target_pose_odom_compensated.position.x, target_pose_odom_compensated.position.y, target_pose_odom_compensated.position.z);
      RCLCPP_INFO(this->get_logger(), "  回転: x=%.3f, y=%.3f, z=%.3f",
                   target_pose_odom_compensated.orientation.x, target_pose_odom_compensated.orientation.y,
                   target_pose_odom_compensated.orientation.z, target_pose_odom_compensated.orientation.w);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed (odom -> base_footprint), odom offset not applied: %s", ex.what());
    }

    geometry_msgs::msg::Pose modified_pose = modifyPoseForOrientation(target_pose_odom_compensated);

    if (executeMove(modified_pose, *response))
    {
      response->message = "MoveHandToTargetCoord succeeded.";
    }
    else
    {
      response->message = "MoveHandToTargetCoord failed.";
    }
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