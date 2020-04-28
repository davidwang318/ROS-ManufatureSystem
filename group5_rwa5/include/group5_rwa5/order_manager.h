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
    bool PickFromBelt();
    bool PickFromBin();
    std::string GetProductFrame(std::string product_type, int bin_num);
    std::map<std::string, std::list<std::pair<std::string,geometry_msgs::Pose>>> GetOrder();
    bool PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id, int whichArm, bool transition, int placed_index, int bin_num);
    void PickAndPlaceDrop(int whichArm, std::string product_type, std::string product_frame, geometry_msgs::Pose end_pose);
    bool PickAndPlaceBelt(std::string whichBin, bool transition);
    void SubmitAGV(int num);
    void offsetPose(std::string type_, geometry_msgs::Pose& pose_);

    void PrintOrder();
    void PrintShipment();
    void UpdateOrder(int productIdx);
    void PlanBeltStrategy();
    void PlanStrategyShipment();
    bool CheckUpdate();
    void FindEmptySpace();

    RobotController arm1_;
    RobotController arm2_;

    geometry_msgs::Pose trans_pose; // Position of transition parts

    // Order attributes:
    std::vector<osrf_gear::Order> received_orders_;
    osrf_gear::Shipment original_shipment;
    osrf_gear::Shipment shipment_in_progress;
    
    std::string order_id;
    int shipment_num;
    int agv_id;

    std::map<int, geometry_msgs::Pose> placed_order; // Orders being placed
    std::vector<std::string> received_shipment_type;
    std::vector<geometry_msgs::Pose> received_shipment_pose;

    bool bin_end = false;

    // Belt attributes:
    std::map<std::string, int> belt_part_num; // number of each type needed to pick from belt
    std::map<int, std::vector<geometry_msgs::Pose>> empty_place; // place where to part should be put
    bool belt_end = false; // Force quit the belt strategy
    int empty_count = 0;



private:
    ros::NodeHandle order_manager_nh_;
    ros::Subscriber order_subscriber_;
    AriacSensorManager camera_;
    tf::TransformListener part_tf_listener_;
    std::pair<std::string, geometry_msgs::Pose> product_type_pose_;
    std::string object;
    std::map<std::string, std::vector<std::string>> product_frame_list_;
    osrf_gear::Order order_;
    std::string state_;

    int agv_id_ = 0;
};

