//
// Created by zeid on 2/27/20.
//

#ifndef SRC_ROBOT_CONTROLLER_H
#define SRC_ROBOT_CONTROLLER_H


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <stdarg.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <initializer_list>

#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

class RobotController {
public:
    RobotController(std::string arm_id);
    ~RobotController();
    bool Planner();
    void Execute();
    void GoToTarget(std::initializer_list<geometry_msgs::Pose> list);
    void GoToTarget(const geometry_msgs::Pose& pose);
    void PrepareRobot(std::string task);
    bool DropPart(geometry_msgs::Pose pose);
    void GripperToggle(const bool& state);
    void GripperCallback(const osrf_gear::VacuumGripperState::ConstPtr& grip);
    void GripperStateCheck(geometry_msgs::Pose pose);
    bool PickPart(geometry_msgs::Pose& part_pose);

private:
    ros::NodeHandle robot_controller_nh_;
    moveit::planning_interface::MoveGroupInterface::Options robot_controller_options;
    ros::ServiceClient gripper_client_;
    ros::NodeHandle gripper_nh_;
    ros::Subscriber gripper_subscriber_;

    tf::TransformListener robot_tf_listener_;
    tf::StampedTransform robot_tf_transform_;
    tf::TransformListener agv_tf_listener_;
    tf::StampedTransform agv_tf_transform_;

    geometry_msgs::Pose target_pose_;

    moveit::planning_interface::MoveGroupInterface robot_move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;

    osrf_gear::VacuumGripperControl gripper_service_;
    osrf_gear::VacuumGripperState gripper_status_;

    std::string object;
    bool plan_success_;

    std::vector<double> binJointPose1_;
    std::vector<double> binJointPose2_;
    std::vector<double> binJointPose3_;
    std::vector<double> binJointPose4_;
    std::vector<double> binJointPose5_;
    std::vector<double> binJointPose6_;
    std::vector<double> beltJointPose_;
    std::vector<double> dropJointPose_;
    std::vector<double> transJointPose_;
    std::vector<double> endJointPose_;

    geometry_msgs::Pose home_cart_pose_;
    geometry_msgs::Quaternion fixed_orientation_;
    geometry_msgs::Pose agv_position_;

    double offset_;
    double roll_def_,pitch_def_,yaw_def_;
    tf::Quaternion q;
    int counter_;
    bool gripper_state_, drop_flag_;
};
#endif //SRC_ROBOT_CONTROLLER_H
