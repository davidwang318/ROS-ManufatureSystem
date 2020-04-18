//
// Created by zeid on 2/27/20.
//
#include "sensor.h"


AriacSensorManager::AriacSensorManager(){
    ROS_INFO_STREAM(">>>>> Subscribing to logical sensors and Quality Control sensors");
    camera_0_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_0", 10,
                                                &AriacSensorManager::LogicalCamera0Callback, this);
    camera_1_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_1", 10,
                                                &AriacSensorManager::LogicalCamera1Callback, this);
    camera_2_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_2", 10,
                                                &AriacSensorManager::LogicalCamera2Callback, this);
    camera_3_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_3", 10,
                                                &AriacSensorManager::LogicalCamera3Callback, this);
    camera_4_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_4", 10,
                                                &AriacSensorManager::LogicalCamera4Callback, this);
    camera_5_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_5", 10,
                                                &AriacSensorManager::LogicalCamera5Callback, this);
    camera_6_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_6", 10,
                                                &AriacSensorManager::LogicalCamera6Callback, this);

    quality_control_1_subscriber_ = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 10,
                                                &AriacSensorManager::QualityControl1Callback, this);
    quality_control_2_subscriber_ = sensor_nh_.subscribe("/ariac/quality_control_sensor_2", 10,
                                                &AriacSensorManager::QualityControl2Callback, this);


    camera0_frame_counter_ = 0;
    camera1_frame_counter_ = 1;
    camera2_frame_counter_ = 1;
    camera3_frame_counter_ = 1;
    camera4_frame_counter_ = 1;
    camera5_frame_counter_ = 1;
    camera6_frame_counter_ = 1;
    faultyTray1 = false;
    faultyTray2 = false;

    init_ = false;
    cam_0_ = false;
    cam_1_ = false;
    cam_2_ = false;
    cam_3_ = false;
    cam_4_ = false;
    cam_5_ = false;
    cam_6_ = false;

    std::map<std::string, std::vector<std::string>> tmp;
    for(int i = 0; i < 7; i++){
        product_frame_list_.push_back(tmp);
    }
}

AriacSensorManager::~AriacSensorManager() {}



void AriacSensorManager::LogicalCamera0Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (init_) return;
    if (image_msg->models.size() == 0) {
        // ROS_INFO_STREAM("Logical Camera 1 does not see anything");
    }
    else{
        ROS_INFO_STREAM_THROTTLE(10, "Logical camera 1: '" << image_msg->models.size() << "' objects.");   
        current_parts_1_ = *image_msg;
        this->BuildProductFrames(0);
    }
    return;
}

void AriacSensorManager::LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    // if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 1: '" << image_msg->models.size() << "' objects.");
    // if (image_msg->models.size() == 0)
    //     ROS_ERROR_STREAM("Logical Camera 1 does not see anything");
    current_parts_1_ = *image_msg;
    this->BuildProductFrames(1);
}

void AriacSensorManager::LogicalCamera2Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    // if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 2: '" << image_msg->models.size() << "' objects.");
    current_parts_2_ = *image_msg;
    this->BuildProductFrames(2);
}

void AriacSensorManager::LogicalCamera3Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    // if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 3: '" << image_msg->models.size() << "' objects.");
    current_parts_3_ = *image_msg;
    this->BuildProductFrames(3);
}

void AriacSensorManager::LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    // if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 4: '" << image_msg->models.size() << "' objects.");
    current_parts_4_ = *image_msg;
    this->BuildProductFrames(4);
}

void AriacSensorManager::LogicalCamera5Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    // if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 5: '" << image_msg->models.size() << "' objects.");
    current_parts_5_ = *image_msg;
    this->BuildProductFrames(5);
}

void AriacSensorManager::LogicalCamera6Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    // if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 6: '" << image_msg->models.size() << "' objects.");
    current_parts_6_ = *image_msg;
    this->BuildProductFrames(6);
}

void AriacSensorManager::QualityControl1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    // ROS_INFO_STREAM(">>>>>>> Quality Check Sensor 1: Checking Quality");
    auto msg = image_msg;
    if (image_msg->models.size() == 0){
        faultyTray1 = false;
        // ROS_INFO_STREAM("All Model in Kit Tray 1 are good!");
    }
    else{
        faultyTray1 = true;
        // ROS_INFO_STREAM("QC1 detects faulty parts, message: " << *msg);
    }
}

void AriacSensorManager::QualityControl2Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    // ROS_INFO_STREAM(">>>>>>> Quality Check Sensor 1: Checking Quality");
    auto msg = image_msg;
    if (image_msg->models.size() == 0){
        faultyTray2 = false;
        // ROS_INFO_STREAM("All Model in Kit Tray 1 are good!");
    }
    else{
        faultyTray2 = true;
        // ROS_INFO_STREAM("QC1 detects faulty parts, message: " << *msg);
    }
}

void AriacSensorManager::BuildProductFrames(int camera_id){
    if (camera_id == 0) {
        camera0_frame_counter_ = 0;
        for (auto& msg : current_parts_0_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_0_" + msg.type + "_" +
                                        std::to_string(camera0_frame_counter_) + "_frame";

            product_frame_list_[0][msg.type].emplace_back(product_frame);
            camera0_frame_counter_++;
        }
        cam_0_ = true;
    }
    // ------------------------------------------------------------------------------------------
    // variable init is for preventing very large vector inside the product_frame_list_[i] (a map)
    else if (camera_id == 1) {
        bool init = false;
        camera1_frame_counter_ = 1;
        for (auto& msg : current_parts_1_.models) {
            //--build the frame for each product
            if(!init) {
                product_frame_list_[1][msg.type].clear();
                init = true;
            }
            std::string product_frame = "logical_camera_1_" + msg.type + "_" +
                                        std::to_string(camera1_frame_counter_) + "_frame";

            product_frame_list_[1][msg.type].emplace_back(product_frame);
            camera1_frame_counter_++;
        }
        cam_1_ = true;
    }
    else if (camera_id == 2) {
        bool init = false;
        camera2_frame_counter_ = 1;
        for (auto& msg : current_parts_2_.models) {
            //--build the frame for each product
            if(!init) {
                product_frame_list_[2][msg.type].clear();
                init = true;
            }
            std::string product_frame = "logical_camera_2_" + msg.type + "_" +
                                        std::to_string(camera2_frame_counter_) + "_frame";

            product_frame_list_[2][msg.type].emplace_back(product_frame);
            camera2_frame_counter_++;
        }
        cam_2_ = true;
    }
    else if (camera_id == 3) {
        if (cam_3_) return;
        bool init = false;
        camera3_frame_counter_ = 1;
        for (auto& msg : current_parts_3_.models) {
            //--build the frame for each product
            if(!init) {
                product_frame_list_[3][msg.type].clear();
                init = true;
            }
            std::string product_frame = "logical_camera_3_" + msg.type + "_" +
                                        std::to_string(camera3_frame_counter_) + "_frame";

            product_frame_list_[3][msg.type].emplace_back(product_frame);
            camera3_frame_counter_++;
        }
        cam_3_ = true;
    }
    else if (camera_id == 4) {
        if (cam_4_) return;
        bool init = false;
        camera4_frame_counter_ = 1;
        for (auto& msg : current_parts_4_.models) {
            //--build the frame for each product
            if(!init) {
                product_frame_list_[4][msg.type].clear();
                init = true;
            }
            std::string product_frame = "logical_camera_4_" + msg.type + "_" +
                                        std::to_string(camera4_frame_counter_) + "_frame";

            product_frame_list_[4][msg.type].emplace_back(product_frame);
            camera4_frame_counter_++;
        }
        cam_4_ = true;
    }
    else if (camera_id == 5) {
        bool init = false;
        camera5_frame_counter_ = 1;
        for (auto& msg : current_parts_5_.models) {
            //--build the frame for each product
            if(!init) {
                product_frame_list_[5][msg.type].clear();
                init = true;
            }
            std::string product_frame = "logical_camera_5_" + msg.type + "_" +
                                        std::to_string(camera5_frame_counter_) + "_frame";

            product_frame_list_[5][msg.type].emplace_back(product_frame);
            camera5_frame_counter_++;
        }
        cam_5_ = true;
    }
    else if (camera_id == 6) {
        bool init = false;
        camera6_frame_counter_ = 1;
        for (auto& msg : current_parts_6_.models) {
            //--build the frame for each product
            if(!init) {
                product_frame_list_[6][msg.type].clear();
                init = true;
            }
            std::string product_frame = "logical_camera_6_" + msg.type + "_" +
                                        std::to_string(camera6_frame_counter_) + "_frame";

            product_frame_list_[6][msg.type].emplace_back(product_frame);
            camera6_frame_counter_++;
        }
        cam_6_ = true;
    }
    else {
        ROS_ERROR_STREAM("In BuildProductFrames, no such camera_id!");
    }

}


geometry_msgs::Pose AriacSensorManager::GetPartPose(const std::string& src_frame,
                                        const std::string& target_frame) {
    geometry_msgs::Pose part_pose;
    ROS_INFO_STREAM("Getting part pose...");
    init_ = true;   

    if (init_) {
        camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                             ros::Duration(0));
        camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                            camera_tf_transform_);

        part_pose.position.x = camera_tf_transform_.getOrigin().x();
        part_pose.position.y = camera_tf_transform_.getOrigin().y();
        part_pose.position.z = camera_tf_transform_.getOrigin().z();

    } else {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(1);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(2);

        part_pose = this->GetPartPose(src_frame, target_frame);
    }

    return part_pose;
}

bool AriacSensorManager::CheckForQuality(int i) {
    ROS_WARN(">>>>>>Checking quality of the parts placed in the kit%d...", i);
    bool faulty = (i==1)? faultyTray1 : faultyTray2;
    if(faulty) ROS_INFO("This part is a faulty part !!!");
    return faulty;
}

