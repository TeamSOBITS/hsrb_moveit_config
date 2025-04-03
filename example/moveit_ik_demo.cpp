#include "interfaces.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>

void move_to_target(moveit::planning_interface::MoveGroupInterface &whole_body_group,
                    const geometry_msgs::msg::Pose &target_pose)
{
    double yaw = std::atan2(target_pose.position.y, target_pose.position.x) + M_PI;
    tf2::Quaternion q;
    q.setRPY(0.0, -M_PI / 2.0, yaw);
    geometry_msgs::msg::Pose modified_pose = target_pose;
    modified_pose.orientation.x = q.x();
    modified_pose.orientation.y = q.y();
    modified_pose.orientation.z = q.z();
    modified_pose.orientation.w = q.w();

    whole_body_group.setPoseTarget(modified_pose);
    whole_body_group.move();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto move_group_node = rclcpp::Node::make_shared("moveit_ik_demo");
    auto logger = move_group_node->get_logger();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    RCLCPP_INFO(logger, "step1: move_to_neutral");
    auto interfaces = hsrb_moveit_config::Interfaces(move_group_node);
    if (!interfaces.MoveToNeutral())
    {
        RCLCPP_ERROR(logger, "MoveToNeutral failure");
        return EXIT_FAILURE;
    }

    RCLCPP_INFO(logger, "step2: move hand forward");

    moveit::planning_interface::MoveGroupInterface whole_body_group(move_group_node, "active_hand");
    whole_body_group.setMaxVelocityScalingFactor(0.5);
    whole_body_group.setMaxAccelerationScalingFactor(0.5);

    std::vector<std::string> joint_names = {
        "arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
        "wrist_flex_joint", "wrist_roll_joint"};

    moveit_msgs::msg::Constraints joint_constraints;

    for (const auto &joint_name : joint_names)
    {
        // パラメータの宣言
        std::string param_prefix = "joint_constraints." + joint_name;

        move_group_node->declare_parameter(param_prefix + ".position", 0.0);
        move_group_node->declare_parameter(param_prefix + ".tolerance_above", 0.1);
        move_group_node->declare_parameter(param_prefix + ".tolerance_below", 0.1);
        move_group_node->declare_parameter(param_prefix + ".weight", 0.0);

        // パラメータの取得
        double position = move_group_node->get_parameter(param_prefix + ".position").as_double();
        double tolerance_above = move_group_node->get_parameter(param_prefix + ".tolerance_above").as_double();
        double tolerance_below = move_group_node->get_parameter(param_prefix + ".tolerance_below").as_double();
        double weight = move_group_node->get_parameter(param_prefix + ".weight").as_double();

        // 各関節に制約を追加
        moveit_msgs::msg::JointConstraint constraint;
        constraint.joint_name = joint_name;
        constraint.position = position;
        constraint.tolerance_above = tolerance_above;
        constraint.tolerance_below = tolerance_below;
        constraint.weight = weight;

        joint_constraints.joint_constraints.push_back(constraint);
    }

    whole_body_group.setPathConstraints(joint_constraints);

    // 3つの target_pose を設定
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = 2.0;
    target_pose1.position.y = -1.0;
    target_pose1.position.z = 0.05;

    geometry_msgs::msg::Pose target_pose2;
    target_pose2.position.x = -2.0;
    target_pose2.position.y = 0.0;
    target_pose2.position.z = 0.5;

    geometry_msgs::msg::Pose target_pose3;
    target_pose3.position.x = 2.0;
    target_pose3.position.y = 1.0;
    target_pose3.position.z = 1.0;

    // 各 target_pose に対して move_to_target を呼び出し
    move_to_target(whole_body_group, target_pose1);
    move_to_target(whole_body_group, target_pose2);
    move_to_target(whole_body_group, target_pose3);

    // Move to neutral position again after the moves
    if (!interfaces.MoveToNeutral())
    {
        RCLCPP_ERROR(logger, "MoveToNeutral failure");
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}