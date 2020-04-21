#pragma once

#include <list>
#include <map>
#include <string>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "sensor.h"
#include "robot_controller.h"

class AriacOrderManager {
public:
    AriacOrderManager();
    ~AriacOrderManager();
    void OrderCallback(const osrf_gear::Order::ConstPtr& order_msg);
    void PickFromBelt();
    void PickFromBin();
    std::string GetProductFrame(std::string product_type);
    std::map<std::string, std::list<std::pair<std::string,geometry_msgs::Pose>>> GetOrder();
    bool PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id, int whichArm, bool transition, int placed_index);
    void SubmitAGV(int num);
    void offsetPose(std::string type_, geometry_msgs::Pose& pose_);

    void PrintOrder();
    void UpdateOrder(int orderIdx, int shipmentIdx, int productIdx);
    void PlanStrategy();
    void CheckUpdate();

    std::vector<osrf_gear::Order> received_orders_;
    std::vector<osrf_gear::Order> order_in_progress_;
    RobotController arm1_;
    RobotController arm2_;

    geometry_msgs::Pose trans_pose; // Position of transition parts

    std::map<int, geometry_msgs::Pose> placed_order; // Orders being placed
    std::vector<std::string> received_orders_type;
    std::vector<geometry_msgs::Pose> received_orders_pose;
    int current_order;
    int place_time;
    int agv_id;


private:
    ros::NodeHandle order_manager_nh_;
    ros::Subscriber order_subscriber_;
    AriacSensorManager camera_;
    tf::TransformListener part_tf_listener_;
    std::pair<std::string,geometry_msgs::Pose> product_type_pose_;
    std::string object;
    std::map<std::string, std::vector<std::string>> product_frame_list_;
    osrf_gear::Order order_;

    std::string state_;

    int agv_id_ = 0;
};

