//
// Created by zeid on 2/27/20.
//

#pragma once


#include <list>
#include <map>
#include <string>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Proximity.h>
#include "ariac_part_manager.h"

class AriacSensorManager {
public:
    AriacSensorManager();
    ~AriacSensorManager();
    void LogicalCamera0Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera2Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera3Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera5Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera6Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera7Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void QualityControl1Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void QualityControl2Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void BreakBeamCallback(const osrf_gear::Proximity::ConstPtr&);

    bool faultyTray1;
    bool faultyTray2;

    bool beltFlag;
    std::map<std::string, int> beltMap;

    geometry_msgs::Pose GetPartPose(const std::string& src_frame,
                                    const std::string& target_frame);

    std::map<std::string, std::vector<std::string>> get_product_frame_list(int i) { return product_frame_list_[i]; }
    std::vector<std::map<std::string, std::vector<std::string>>> product_frame_list_;
    
    void BuildProductFrames(int);

    bool CheckForQuality(int i); // Checks for the quality of the parts present in the kit tray

private:
    ros::NodeHandle sensor_nh_;
    ros::Subscriber break_beam_subscriber_;
    ros::Subscriber camera_0_subscriber_;
    ros::Subscriber camera_1_subscriber_;
    ros::Subscriber camera_2_subscriber_;
    ros::Subscriber camera_3_subscriber_;
    ros::Subscriber camera_4_subscriber_;
    ros::Subscriber camera_5_subscriber_;
    ros::Subscriber camera_6_subscriber_;
    ros::Subscriber camera_7_subscriber_;
    ros::Subscriber quality_control_1_subscriber_; //-- For subscribing to the quality control sensor topic
    ros::Subscriber quality_control_2_subscriber_;


    tf::TransformListener camera_tf_listener_;
    tf::StampedTransform camera_tf_transform_;

    osrf_gear::LogicalCameraImage current_parts_0_;
    osrf_gear::LogicalCameraImage current_parts_1_;
    osrf_gear::LogicalCameraImage current_parts_2_;
    osrf_gear::LogicalCameraImage current_parts_3_;
    osrf_gear::LogicalCameraImage current_parts_4_;
    osrf_gear::LogicalCameraImage current_parts_5_;
    osrf_gear::LogicalCameraImage current_parts_6_;

    // std::vector<AriacPartManager> camera1_part_list,camera2_part_list,camera3_part_list;

    //std::map<std::string, std::list<std::string>> parts_list_;

    bool init_, cam_0_, cam_1_, cam_2_,cam_3_, cam_4_, cam_5_, cam_6_;
    int camera0_frame_counter_, camera1_frame_counter_, camera2_frame_counter_, camera3_frame_counter_, camera4_frame_counter_, camera5_frame_counter_, camera6_frame_counter_;
};

