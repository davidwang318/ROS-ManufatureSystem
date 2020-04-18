//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>



//AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);
    current_order = 0;
    place_time = 0;
    agv_id = 0;

}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN(">>>>> OrderCallback");
    received_orders_.push_back(*order_msg);
}


/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type) {
    //--Grab the last one from the list then remove it
    if (!product_frame_list_.empty()) {
        std::string frame = product_frame_list_[product_type].back();
        ROS_INFO_STREAM("Frame >>>> " << frame);
        product_frame_list_[product_type].pop_back();
        return frame;
    } else {
        ROS_ERROR_STREAM("No product frame found for " << product_type);
        ros::shutdown();
    }
}

bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id, int whichArm, bool transition, int placed_index) {
    std::string product_type = product_type_pose.first;
    ROS_WARN_STREAM("Product type >>>> " << product_type);
    std::string product_frame = this->GetProductFrame(product_type);
    ROS_WARN_STREAM("Product frame >>>> " << product_frame);
    
    auto part_pose = camera_.GetPartPose("/world",product_frame);
    this -> offsetPose(product_type, part_pose);
    // std::cout << "///////////////////////" <<part_pose.position.z << std::endl;
    // Pick the part
    bool failed_pick = false;
    if (whichArm == 1) failed_pick = arm1_.PickPart(part_pose);
    else failed_pick = arm2_.PickPart(part_pose);
    ROS_WARN_STREAM("Picking up state " << failed_pick);
    while(!failed_pick){
        part_pose.position.z -= 0.03;
        // std::cout << "///////////////////////" <<part_pose.position.z << std::endl;
        if (whichArm == 1) failed_pick = arm1_.PickPart(part_pose);
        else failed_pick = arm2_.PickPart(part_pose);
    }

    // Is it transition?
    if (transition){
        if (whichArm == 1){
            arm1_.PrepareRobot("bin4");
            arm1_.GripperToggle(false);
            arm1_.PrepareRobot("bin2");
            arm2_.PrepareRobot("bin4");

            product_frame[15] = '4';
            whichArm = 2;
            part_pose = camera_.GetPartPose("/world",product_frame);
            this -> offsetPose(product_type, part_pose);

            failed_pick = arm2_.PickPart(part_pose);
            while(!failed_pick){
                part_pose.position.z -= 0.03;
                failed_pick = arm2_.PickPart(part_pose);
            }
        }
        else {
            arm2_.PrepareRobot("bin3");
            arm2_.GripperToggle(false);
            arm2_.PrepareRobot("bin5");
            arm1_.PrepareRobot("bin3");

            product_frame[15] = '3';
            whichArm = 1;
            part_pose = camera_.GetPartPose("/world",product_frame);
            this -> offsetPose(product_type, part_pose);

            failed_pick = arm1_.PickPart(part_pose);
            while(!failed_pick){
                part_pose.position.z -= 0.03;
                failed_pick = arm1_.PickPart(part_pose);
            }
        }
    }

    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;

    if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        // StampedPose_out.pose.position.z += 0.1;
        // StampedPose_out.pose.position.y -= 0.2;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

    }
    else{
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        // StampedPose_out.pose.position.z += 0.1;
        // StampedPose_out.pose.position.y += 0.2;
        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
    }

    if (whichArm == 1) {
        auto result = arm1_.DropPart(StampedPose_out.pose);
        if(camera_.CheckForQuality(1)){
            auto tmp = arm1_.PickPart(StampedPose_out.pose);
            arm1_.PrepareRobot("drop");
            arm1_.GripperToggle(false);
            return false;
        }
    }
    else {
        auto result = arm2_.DropPart(StampedPose_out.pose);
        if(camera_.CheckForQuality(2)){
            auto tmp = arm2_.PickPart(StampedPose_out.pose);
            arm2_.PrepareRobot("drop");
            arm2_.GripperToggle(false);
            return false;
        }
    }
    std::cout << "---------" << placed_index << std::endl;
    placed_order[placed_index] = StampedPose_out.pose;
    return true;
}

// Function to pick a part from the belt
void AriacOrderManager::PickFromBelt() {
    state_ = "belt";
    arm1_.PrepareRobot("belt");
    ROS_WARN(">>>>>> Waiting parts...");
    bool hold = true;
    int orderIdx_ = 0, shipmentIdx_ = 0, productIdx_ = 0;
    while(hold){
        ros::spinOnce();
        product_frame_list_ = camera_.get_product_frame_list(0);

        if(!product_frame_list_.empty()){
            orderIdx_ = 0;
            shipmentIdx_ = 0;
            productIdx_ = 0;

            for (const auto &order : received_orders_){
                auto order_id = order.order_id;
                auto shipments = order.shipments;
                ROS_WARN(">>>>>> Start scanning Order ID: %s", order_id.c_str());

                for (const auto &shipment : shipments){
                    auto shipment_type = shipment.shipment_type;
                    auto agv = shipment.agv_id.back();//--this returns a char
                    agv_id_ = (shipment.agv_id == "any") ? 1 : agv - '0';
                    auto products = shipment.products;
                    ROS_INFO_STREAM("Order ID: " << order_id);
                    ROS_INFO_STREAM("Shipment Type: " << shipment_type);
                    ROS_INFO_STREAM("AGV ID: " << agv_id_);

                    for (const auto &product: products){
                        if (product_frame_list_.count(product.type) == 0) {
                            productIdx_++; continue;
                        }
                        ROS_WARN(">>>>>> FOUND !!");
                        product_type_pose_.first = product.type;
                        ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
                        product_type_pose_.second = product.pose;
                        hold = false;
                        break;    
                    }
                    if(!hold) break;
                    else shipmentIdx_++;
                }
                if(!hold) break;
                else orderIdx_++;
            }
        }
    }

    ROS_WARN(">>>>>> Executing order...");
    bool pick_n_place_success{false};
    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;
    pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id_, 1, true, productIdx_);

    if (pick_n_place_success){
        UpdateOrder(orderIdx_, shipmentIdx_, productIdx_);
    }
    else{
        PickFromBelt();
    }    

    return;
}

// Function to pick a part of the order from the Bin
void AriacOrderManager::PickFromBin() {
    ros::spinOnce();
    ROS_WARN(">>>>>> Searching bins...");
    state_ = "bin";
    int shipmentIdx_ = 0, productIdx_ = 0;
    bool find = false;
    std::string whichBin;
    for(int i = 1; i < 7; i++){
        product_frame_list_ = camera_.get_product_frame_list(i);

        if(!product_frame_list_.empty()){
            shipmentIdx_ = 0;
            productIdx_ = 0;
            auto order = order_in_progress_[current_order];  // Using a copy of recieved_orders_ 
            auto order_id = order.order_id;
            auto shipments = order.shipments;
            ROS_WARN("[PickFromBin]: Start scanning Order ID: %s", order_id.c_str());

            for (const auto &shipment : shipments){
                auto shipment_type = shipment.shipment_type;
                auto agv = shipment.agv_id.back();//--this returns a char
                agv_id_ = (shipment.agv_id == "any") ? 1 : agv - '0';
                auto products = shipment.products;
                ROS_INFO_STREAM("[PickFromBin]: Order ID: " << order_id);
                ROS_INFO_STREAM("[PickFromBin]: Shipment Type: " << shipment_type);
                ROS_INFO_STREAM("[PickFromBin]: AGV ID: " << agv_id_);

                for (const auto &product: products){
                    if (product_frame_list_.count(product.type) == 0) {
                        productIdx_++; continue;
                    }
                    ROS_WARN("[PickFromBin]: FOUND !!");
                    product_type_pose_.first = product.type;
                    ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
                    product_type_pose_.second = product.pose;
                    find = true;
                    whichBin = "bin" + std::to_string(i);
                    break;    
                }
                if(find) break;
                else shipmentIdx_++;
            }
            if(find) break;
        }
        if(find) break;
        else ROS_INFO("Not found in Bin %d", i);
    }

    if(!find) return;


    int trueIdx = 0;
    for(int i = 0; i < received_orders_[current_order].shipments[0].products.size(); i++){
        auto org_product = received_orders_[current_order].shipments[0].products[i];
        if(product_type_pose_.first == org_product.type && product_type_pose_.second.position.x == org_product.pose.position.x && 
            product_type_pose_.second.position.y == org_product.pose.position.y && product_type_pose_.second.position.z == org_product.pose.position.z){
            trueIdx = i;
        }
    }

    std::cout << "==============" << std::endl;
    std::cout << "Product Index: " << trueIdx << std::endl;
    std::cout << whichBin << std::endl;
    bool pick_n_place_success{false};
    if (agv_id_ == 1){
        if(whichBin == "bin5" || whichBin == "bin6"){
            arm2_.PrepareRobot(whichBin);
            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id_, 2, true, trueIdx);
        }
        else{
            arm1_.PrepareRobot(whichBin);
            pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id_, 1, false, trueIdx);
        }
    }
    else{
        if(whichBin == "bin1" || whichBin == "bin2"){
            arm1_.PrepareRobot(whichBin);
            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id_, 1, true, trueIdx);
        }
        else{
            arm2_.PrepareRobot(whichBin);
            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id_, 2, false, trueIdx);
        }
    }


    if (pick_n_place_success) {
        UpdateOrder(current_order, shipmentIdx_, productIdx_);
    }
    else PickFromBin();
    return;
}


void AriacOrderManager::SubmitAGV(int num) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    // srv.request.kit_type = "order_0_kit_0";
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else
        ROS_INFO("Service succeeded.");
}

/* 
    Utility Functions

1. offsetPose: Offset the pickup place
2. PrintOrder: print the order
3. UpdateOrder: update the order
4. CheckFaulty: cheack whether there is a faulty part

*/
void AriacOrderManager::offsetPose(std::string type_, geometry_msgs::Pose& pose_){
    if (state_ == "belt"){
        if(type_ == "gasket_part"){       
            pose_.position.x += 0.01;
            pose_.position.y -= 0.365;
            pose_.position.z += 0.0115;
        }
        else if(type_ == "piston_rod_part"){
            pose_.position.x += 0.005;
            pose_.position.y -= 0.325;
            pose_.position.z -= 0.008;
        }
        else if(type_ == "gear_part"){
            pose_.position.y -= 0.35;
            pose_.position.z += 0.0025;
        }
        else{
            ROS_WARN(">>>>>> Undefined type. Don't know how to offset");
        }
    }
    else{
        if(type_ == "gasket_part"){       
            pose_.position.z += 0.012;
        }
        else if(type_ == "piston_rod_part"){
            pose_.position.z -= 0.005;
        }
        else if(type_ == "gear_part"){
            // pose_.position.x += 0.05;
            pose_.position.z -= 0.001;
        }
        else if(type_ == "disk_part"){
            // pose_.position.x += 0.05;
            pose_.position.z += 0.01;
        }
        else if(type_ == "pulley_part"){
            // pose_.position.x += 0.05;
            pose_.position.z += 0.04;
        }
        else{
            ROS_WARN(">>>>>> Undefined type. Don't know how to offset");
        }
    }

    return;
}

void AriacOrderManager::PrintOrder(){
    for (const auto &order : received_orders_){
                auto order_id = order.order_id;
                auto shipments = order.shipments;
                ROS_INFO("\n\n==========PrintOrder==========");
        for (const auto &shipment : shipments){
            auto shipment_type = shipment.shipment_type;
            auto agv = shipment.agv_id.back();
            agv_id_ = (shipment.agv_id == "any") ? 1 : agv - '0';
            auto products = shipment.products;
            ROS_INFO("Order ID: %s",  order_id.c_str());
            ROS_INFO("Shipment Type: %s", shipment_type.c_str());
            ROS_INFO("AGV ID: %d", agv_id_);
            for (const auto &product: products){
                ROS_INFO_STREAM("Product Type: " << product.type);
            }
            ROS_INFO("---------------------------");
        }
    }
    return;
}

void AriacOrderManager::UpdateOrder(int orderIdx, int shipmentIdx, int productIdx){
    // Erase the specific product in the order
    order_in_progress_[orderIdx].shipments[shipmentIdx].products.erase(
        order_in_progress_[orderIdx].shipments[shipmentIdx].products.begin()+productIdx
        );

    PrintOrder();
    return;
}

void AriacOrderManager::PlanStrategy(){
    ROS_INFO_STREAM("[AriacOrderManager]:[PlanStrategy]: Called");
    ros::spinOnce();
    received_orders_type.clear();
    received_orders_pose.clear();
    int fromBelt = 0, fromBin = 0;

    int pulley = 0, gasket = 0, piston = 0, gear = 0, disk = 0;
    for (int i = 1; i < 7; i++){
        product_frame_list_ = camera_.get_product_frame_list(i);
        pulley += product_frame_list_["pulley_part"].size();
        gasket += product_frame_list_["gasket_part"].size();
        piston += product_frame_list_["piston_rod_part"].size();
        gear += product_frame_list_["gear_part"].size();
        disk += product_frame_list_["disk_part"].size();
    }
    ROS_WARN("[AriacOrderManager]:[PlanStrategy]: Getting Order Details");
    int order_pulley = 0, order_gasket = 0, order_piston = 0, order_gear = 0, order_disk = 0;

    auto order = received_orders_[current_order];
    auto shipments = order.shipments;
    for (const auto &shipment : shipments){
        auto products = shipment.products;
        for (const auto &product: products){
            received_orders_type.push_back(product.type);
            received_orders_pose.push_back(product.pose);
            std::cout << product.type << std::endl;
            std::cout << product.pose.position.x << " " << product.pose.position.y << " " << product.pose.position.z << std::endl;
            if(product.type == "pulley_part") {order_pulley++; continue;}
            else if(product.type == "gasket_part") {order_gasket++; continue;}
            else if(product.type == "piston_rod_part") {order_piston++; continue;}
            else if(product.type == "gear_part") {order_gear++; continue;}
            else if(product.type == "disk_part") {order_disk++; continue;}
            else ROS_WARN("!!! ERROR, Missing part type !!!");
        }
    }

    fromBin = std::min(pulley, order_pulley) + std::min(gasket, order_gasket)
            + std::min(piston, order_piston) + std::min(gear, order_gear) + std::min(disk, order_disk);
    fromBelt = order_pulley + order_gasket + order_piston + order_gear + order_disk - fromBin;

    ROS_INFO("\n\n==========Print Parts On Bins==========");
    ROS_INFO("pulley_part = %d", pulley);
    ROS_INFO("gasket_part = %d", gasket);
    ROS_INFO("piston_rod_part = %d", piston);
    ROS_INFO("gear_part = %d", gear);
    ROS_INFO("disk_part = %d", disk);

    ROS_INFO("\n\n==========Print Parts In Orders==========");
    ROS_INFO("pulley_part = %d", order_pulley);
    ROS_INFO("gasket_part = %d", order_gasket);
    ROS_INFO("piston_rod_part = %d", order_piston);
    ROS_INFO("gear_part = %d", order_gear);
    ROS_INFO("disk_part = %d", order_disk);

    ROS_INFO("\n\n==========Print Strategy==========");
    ROS_INFO("Pick %d Parts From Belt", fromBelt);
    ROS_INFO("Pick %d Parts From Bins", fromBin);

    order_in_progress_ = received_orders_; // Let PickFromBin() and PickFromBelt() work on copy of recieved_orders_;
    while(fromBelt-- > 0) PickFromBelt();
    while(fromBin-- > 0) PickFromBin();

    // todo: Now think about the updated order what we have to change - remove / move / add
    ROS_INFO_STREAM("[AriacOrderManager::PlanStrategy] Strategy One executed");
    if(received_orders_.size()-1 > current_order){ CheckUpdate();}

    return;
}

void AriacOrderManager::CheckUpdate(){
    std::vector<std::vector<int>> eraseOrderIdx;
    std::vector<int> correctProductIdx;
    current_order ++;

    auto order = received_orders_[current_order];
    int agv_id_ = 0;
    auto shipments = order.shipments;

    for (int shipmentIdx_ = 0; shipmentIdx_ < shipments.size(); shipmentIdx_++){

        auto shipment = shipments[shipmentIdx_];
        agv_id_ = (shipment.agv_id == "any") ? 1 : shipment.agv_id.back() - '0';
        auto products = shipment.products;

        for (int productIdx_ = 0; productIdx_ < products.size(); productIdx_++){

            auto product = products[productIdx_];
            auto type = product.type;
            auto pose = product.pose;

            for (int copyIdx = 0; copyIdx < received_orders_type.size(); copyIdx++){
                auto copy_type = received_orders_type[copyIdx];
                auto copy_pose = received_orders_pose[copyIdx];
                if(type == copy_type && pose.position.x == copy_pose.position.x && pose.position.y == copy_pose.position.y && pose.position.z == copy_pose.position.z){
                    std::cout << productIdx_ << ' ' << copyIdx << std::endl;
                    std::cout << type << " " << copy_type << std::endl;
                    std::cout << pose.position.x << ' ' << pose.position.y << ' ' << pose.position.z << std::endl;
                    std::cout << copy_pose.position.x << ' ' << copy_pose.position.y << ' ' << copy_pose.position.z << std::endl;
                    eraseOrderIdx.push_back({current_order, shipmentIdx_, productIdx_});
                    correctProductIdx.push_back(copyIdx);
                    break;
                }
            }
        }
    }

    for (int i = 0; i < placed_order.size(); i++){
        if (std::find(correctProductIdx.begin(), correctProductIdx.end(), i) != correctProductIdx.end()){
            continue;
        }
        if (agv_id_ == 1){
            auto tmp = arm1_.PickPart(placed_order[i]);
            arm1_.PrepareRobot("drop");
            arm1_.GripperToggle(false);
            arm1_.PrepareRobot("end");
        }
        else {
            auto tmp = arm2_.PickPart(placed_order[i]);
            arm2_.PrepareRobot("drop");
            arm2_.GripperToggle(false);
            arm2_.PrepareRobot("end");
        }
    }

    int tmpCount = 0; // offset the index
    for (const auto &eraseIdx : eraseOrderIdx){
        received_orders_[eraseIdx[0]].shipments[eraseIdx[1]].products.erase(
            received_orders_[eraseIdx[0]].shipments[eraseIdx[1]].products.begin()+eraseIdx[2]-tmpCount);
        tmpCount++;
    }

    std::cout << "-------------------\n";
    for (auto tmp : received_orders_[current_order].shipments[0].products){
        std::cout << tmp.type << std::endl;
        std::cout << tmp.pose.position.x << " " << tmp.pose.position.y << " " << tmp.pose.position.z << std::endl;
    }

    agv_id = agv_id_;

    PlanStrategy();
}



