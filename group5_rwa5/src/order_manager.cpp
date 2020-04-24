//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>


AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);
    agv_id = 0;
    trans_pose.position.x = 0.3;
    trans_pose.position.y = 0;
    trans_pose.position.z = 1;
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
    // Select the primary arm and the secondary arm
    auto pri_arm = &arm1_, sec_arm = &arm2_;
    if (whichArm == 2) pri_arm = &arm2_, sec_arm = &arm1_;

    // Print the part type and frame
    std::string product_type = product_type_pose.first;
    std::string product_frame = this->GetProductFrame(product_type);
    ROS_WARN_STREAM("Product type >>>> " << product_type);
    ROS_WARN_STREAM("Product frame >>>> " << product_frame);

    // Get and offset the part pose    
    auto part_pose = camera_.GetPartPose("/world",product_frame);
    this -> offsetPose(product_type, part_pose);

    // Pick the part 
    bool failed_pick = pri_arm -> PickPart(part_pose);
    while(!failed_pick){
        part_pose.position.z -= 0.03;
        failed_pick = pri_arm -> PickPart(part_pose);
    }

    // Transmitted to other side by case
    if (transition){
        pri_arm -> PrepareRobot("trans");
        auto tmp = trans_pose;
        auto trans_pose_up = trans_pose;
        trans_pose_up.position.z += 0.3;
        this -> offsetPose(product_type, trans_pose);
        pri_arm -> GoToTarget(trans_pose);
        pri_arm -> GripperToggle(false);
        pri_arm -> GoToTarget(trans_pose_up);
        if (whichArm == 1) pri_arm -> PrepareRobot("bin2");
        else pri_arm -> PrepareRobot("bin5");
        sec_arm -> PrepareRobot("trans");
        trans_pose = tmp;

        if (whichArm == 1) whichArm = 2;
        else whichArm = 1; 
        std::swap(pri_arm, sec_arm);

        product_frame[15] = '7';
        part_pose = camera_.GetPartPose("/world",product_frame);
        this -> offsetPose(product_type, part_pose);

        failed_pick = pri_arm -> PickPart(part_pose);
        while(!failed_pick){
            part_pose.position.z -= 0.03;
            failed_pick = pri_arm -> PickPart(part_pose);      
        }
    }

    // get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;
    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
    StampedPose_in.header.frame_id = "/kit_tray_" + std::to_string(agv_id);
    StampedPose_in.pose = drop_pose;
    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);

    // Offset the pose for safety
    if(product_type == "pulley_part") StampedPose_out.pose.position.z += 0.07;
    else if(product_type == "disk_part") StampedPose_out.pose.position.z += 0.02;

    // Drop the part on the tray and check its quality
    auto result = pri_arm -> DropPart(StampedPose_out.pose);
    if(camera_.CheckForQuality(whichArm)){
        auto tmp = pri_arm -> PickPart(StampedPose_out.pose);
        pri_arm -> PrepareRobot("drop");
        pri_arm -> GripperToggle(false);
        return false;
    }

    // Save the sequence and position of the part for update check 
    placed_order[placed_index] = StampedPose_out.pose;

    return true;
}

// Function to pick a part from the belt
void AriacOrderManager::PickFromBelt() {

    // Initialize some paremeters
    arm1_.PrepareRobot("belt");
    camera_.product_frame_list_[0].clear(); // To have correct offset
    int productIdx_ = 0;
    std::string whichBin;
    state_ = "belt";

    // Start waiting the target parts
    ROS_WARN(">>>>>> Waiting parts...");
    bool hold = true;
    while(hold){
        ros::spinOnce();
        product_frame_list_ = camera_.get_product_frame_list(0);

        if(!product_frame_list_.empty()){
            std::cout << "entering finding stage" << std::endl;
            productIdx_ = 0;
            auto products = shipment_in_progress.products;

            for (const auto &product: products){

                if (product_frame_list_.count(product.type) == 0) {
                    productIdx_++; continue;
                }

                ROS_WARN(">>>>>> FOUND !!");
                ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
                product_type_pose_.first = product.type;
                product_type_pose_.second = product.pose;
                hold = false;
                break;    
            }
        }
    }

    // Find the order of the part ready to pick in the current shipment
    int trueIdx = 0;
    for(int i = 0; i < original_shipment.products.size(); i++){
        auto org_product = original_shipment.products[i];
        if(product_type_pose_.first == org_product.type && product_type_pose_.second.position.x == org_product.pose.position.x && 
            product_type_pose_.second.position.y == org_product.pose.position.y && product_type_pose_.second.position.z == org_product.pose.position.z){
            trueIdx = i;
        }
    }
    std::cout << "==============" << std::endl;
    std::cout << "Product Index: " << trueIdx << std::endl;

    // Call the PickAndPlace function to pick parts
    bool pick_n_place_success{false};
    if (agv_id == 1) pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id, 1, false, productIdx_);
    else pick_n_place_success = PickAndPlace(product_type_pose_, agv_id, 1, true, productIdx_);

    // Faulty Part Condition
    if (pick_n_place_success){
        UpdateOrder(productIdx_);
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

    // Initialize some paremeters
    state_ = "bin";
    int productIdx_ = 0;
    bool find = false;
    std::string whichBin;

    // Start searching for the part from bin1 to bin 6
    for(int i = 1; i < 7; i++){
        product_frame_list_ = camera_.get_product_frame_list(i);

        if(!product_frame_list_.empty()){
            productIdx_ = 0;
            auto products = shipment_in_progress.products;
            ROS_INFO_STREAM("[PickFromBin]: Order ID: " << order_id);
            // ROS_INFO_STREAM("[PickFromBin]: Shipment Type: " << shipment_in_progress.type);
            ROS_INFO_STREAM("[PickFromBin]: AGV ID: " << agv_id);

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
        }
        // Break th searching process to pick    
        if(find) break;
        else ROS_INFO("Not found in Bin %d", i);
    }
    if(!find) return;

    // Find the order of the part ready to pick in the current shipment
    int trueIdx = 0;
    for(int i = 0; i < original_shipment.products.size(); i++){
        auto org_product = original_shipment.products[i];
        if(product_type_pose_.first == org_product.type && product_type_pose_.second.position.x == org_product.pose.position.x && 
            product_type_pose_.second.position.y == org_product.pose.position.y && product_type_pose_.second.position.z == org_product.pose.position.z){
            trueIdx = i;
        }
    }
    std::cout << "==============" << std::endl;
    std::cout << "Product Index: " << trueIdx << std::endl;
    std::cout << "WhichBin: " << whichBin << std::endl;

    // Call the pick function
    bool pick_n_place_success{false};
    if (agv_id == 1){
        if(whichBin == "bin6"){
            arm2_.PrepareRobot(whichBin);
            std::cout << "case1" << std::endl;
            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id, 2, true, trueIdx);
        }
        else{
            if(whichBin == "bin5") arm2_.PrepareRobot("bin6");
            arm1_.PrepareRobot(whichBin);
            std::cout << "case2" << std::endl;
            pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id, 1, false, trueIdx);
        }
    }
    else{
        if(whichBin == "bin1"){
            arm1_.PrepareRobot(whichBin);
            std::cout << "case3" << std::endl;
            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id, 1, true, trueIdx);
        }
        else{
            if(whichBin == "bin2") arm1_.PrepareRobot("bin1");
            arm2_.PrepareRobot(whichBin);
            std::cout << "case4" << std::endl;
            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id, 2, false, trueIdx);
        }
    }


    if (pick_n_place_success) {
        UpdateOrder(productIdx_);
    }
    else PickFromBin(); // Faulty parts

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
        else if(type_ == "disk_part"){
        	pose_.position.x += 0.01;
            pose_.position.y -= 0.35;
            pose_.position.z += 0.008;
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
            pose_.position.z += 0.05;
        }
        else{
            ROS_WARN(">>>>>> Undefined type. Don't know how to offset");
        }
    }

    return;
}

void AriacOrderManager::PrintOrder(){

    int orders_size = received_orders_.size();
    int orders_count = 0;

    ROS_INFO("\n\n==========PrintOrders==========");
    ROS_INFO("Order total count: %d", orders_size);

    for (const auto &order : received_orders_){
        
        auto order_id = order.order_id;
        auto shipments = order.shipments;
        
        int shipments_size = shipments.size();
        int shipments_count = 0;

        ROS_INFO("\n\n==========Print %d Order Detail==========", orders_count++);
        ROS_INFO("-Order ID: %s",  order_id.c_str());
        ROS_INFO("-Shipment total count: %d", shipments_size);
        
        for (const auto &shipment : shipments){
            auto shipment_type = shipment.shipment_type;
            auto agv = shipment.agv_id.back(); agv_id_ = (shipment.agv_id == "any") ? 1 : agv - '0';
            auto products = shipment.products;

            ROS_INFO("--Shipment count: %d", shipments_count++);
            ROS_INFO("--Shipment Type: %s", shipment_type.c_str());
            ROS_INFO("--AGV ID: %d", agv_id_);

            for (const auto &product: products){
                ROS_INFO_STREAM("---Product Type: " << product.type);
            }
            ROS_INFO("---------------------------");
        }
    }
    ROS_INFO("\n\n==========PrintOrdersEnd==========");
    return;
}

void AriacOrderManager::PrintShipment(){
    ROS_INFO("\n\n==========PrintCurrentShipment==========");
    ROS_INFO("-Order ID: %s",  order_id.c_str());
    for (const auto &product: shipment_in_progress.products){
        ROS_INFO_STREAM("--Product Type: " << product.type);
    }
    return;
}

void AriacOrderManager::UpdateOrder(int productIdx){
    // Erase the specific product in the order
    shipment_in_progress.products.erase(shipment_in_progress.products.begin()+productIdx);
    PrintShipment();
    return;
}

void AriacOrderManager::PlanStrategyOrder(){
    ROS_INFO_STREAM("[AriacOrderManager]:[PlanStrategy]: Called");
    ros::spinOnce();
    // Print all the order
    this -> PrintOrder();

    // Get the order_id
    for (int i = 0; i < received_orders_.size(); i++){
        auto order_id_tmp = received_orders_[i].order_id;
        // Make sure its not the updated order
        if (order_id_tmp.size() == 7) PlanStrategyShipment(received_orders_[i]);
    }
    return;
}

void AriacOrderManager::PlanStrategyShipment(osrf_gear::Order order){

    // Initialize the order id
    order_id = order.order_id.c_str();

    for (int i = 0; i < order.shipments.size(); i++){
        // Clear the helper
        placed_order.clear();
        
        // Initialize the shipment parameters
        original_shipment = order.shipments[i];
        shipment_in_progress = original_shipment;
        shipment_num = i;
        agv_id = (original_shipment.agv_id == "any") ? 1 : original_shipment.agv_id.back() - '0';
        int fromBin = 0, fromBelt = 0;

        do{
            // Check update
            ros::spinOnce();
            auto tmp = CheckUpdate();
            std::cout << "test" <<std::endl;

            // Initialize the counter of parts on bins
            int pulley = 0, gasket = 0, piston = 0, gear = 0, disk = 0;
            for (int i = 1; i < 7; i++){
                product_frame_list_ = camera_.get_product_frame_list(i);
                pulley += product_frame_list_["pulley_part"].size();
                gasket += product_frame_list_["gasket_part"].size();
                piston += product_frame_list_["piston_rod_part"].size();
                gear += product_frame_list_["gear_part"].size();
                disk += product_frame_list_["disk_part"].size();
            }

            // Initialize the counter of parts in the shipment
            int order_pulley = 0, order_gasket = 0, order_piston = 0, order_gear = 0, order_disk = 0;
            received_shipment_type.clear();
            received_shipment_pose.clear();
            for(const auto &product : shipment_in_progress.products){
                // Memorize the type and pose for checkUpdate
                received_shipment_type.push_back(product.type);
                received_shipment_pose.push_back(product.pose);
                // Add on the counter
                if(product.type == "pulley_part") {order_pulley++; continue;}
                else if(product.type == "gasket_part") {order_gasket++; continue;}
                else if(product.type == "piston_rod_part") {order_piston++; continue;}
                else if(product.type == "gear_part") {order_gear++; continue;}
                else if(product.type == "disk_part") {order_disk++; continue;}
                else ROS_WARN("!!! ERROR, Missing part type !!!");
            }

            // Calculate the parts needed to be picked from bins and belts
            fromBin = std::min(pulley, order_pulley) + std::min(gasket, order_gasket)
                    + std::min(piston, order_piston) + std::min(gear, order_gear) + std::min(disk, order_disk);
            fromBelt = order_pulley + order_gasket + order_piston + order_gear + order_disk - fromBin;

            // Printing
            ROS_INFO("\n\n==========Print Order Information==========");
            ROS_INFO("Order id = %s", order_id.c_str());
            ROS_INFO("Shipment number = %d", shipment_num);
            ROS_INFO("Agv_id = %d", agv_id);

            // ROS_INFO("\n\n==========Print Parts On Bins==========");
            // ROS_INFO("pulley_part = %d", pulley);
            // ROS_INFO("gasket_part = %d", gasket);
            // ROS_INFO("piston_rod_part = %d", piston);
            // ROS_INFO("gear_part = %d", gear);
            // ROS_INFO("disk_part = %d", disk);

            // ROS_INFO("\n\n==========Print Parts In Orders==========");
            // ROS_INFO("pulley_part = %d", order_pulley);
            // ROS_INFO("gasket_part = %d", order_gasket);
            // ROS_INFO("piston_rod_part = %d", order_piston);
            // ROS_INFO("gear_part = %d", order_gear);
            // ROS_INFO("disk_part = %d", order_disk);

            ROS_INFO("\n\n==========Print Strategy==========");
            ROS_INFO("Pick %d Parts From Belt", fromBelt);
            ROS_INFO("Pick %d Parts From Bins", fromBin);

            // Execute the strategy
            if(fromBelt > 0) {PickFromBelt(); fromBelt--;}
            else if(fromBin > 0) {PickFromBin(); fromBin--;}
            std::cout << fromBin << fromBelt << order_id.size() << std::endl; 

            // Final Check
            if(order_id.size() == 7 && fromBelt == 0 && fromBin == 0){
                ros::Duration(5.5).sleep();
                ros::spinOnce();
                if(CheckUpdate()) fromBin = 1; // Force to run the loop
            }
        }
        while(fromBelt > 0 || fromBin > 0);

        // Submit the agv
        SubmitAGV(agv_id);
    } 

    return;
}

bool AriacOrderManager::CheckUpdate(){
    std::cout << "~~~~~~~~~Checking Update~~~~~~~~~~" << std::endl;
    // Do nothing if its an updated order
    if (order_id.size() > 7) return false;

    // Initialize the parameters
    int update_order_idx = -1;
    osrf_gear::Shipment update_shipment;

    std::cout << "test2" << std::endl;

    // Search the order
    std::string compare_id = order_id + "_update_0";
    for (int i = 0; i < received_orders_.size(); i++){
        std::string target_id = received_orders_[i].order_id.c_str();
        if(target_id == compare_id){
            update_order_idx = i;
            break;
        }
    }

    // Store the updated order if there is one
    if(update_order_idx == -1) return false;
    else{
        update_shipment = received_orders_[update_order_idx].shipments[shipment_num];
    }
    std::cout << "Find new order" << std::endl;

    // Compare the update shipment with the current shipment
    std::map<int, int> correctIdxMap;
    std::vector<int> correctProductIdx;
    for (int updateIdx = 0; updateIdx < update_shipment.products.size(); updateIdx++){
        // Get the part and pose in the update shipment
        auto update_product = update_shipment.products[updateIdx];
        auto update_type = update_product.type;
        auto update_pose = update_product.pose;

        for (int originalIdx = 0; originalIdx < received_shipment_type.size(); originalIdx++){
            // Search the orignal shipment
            auto original_type = received_shipment_type[originalIdx];
            auto original_pose = received_shipment_pose[originalIdx];
            // Find the same pruduct in both shipments
            if(update_type == original_type && update_pose.position.x == original_pose.position.x && 
               update_pose.position.y == original_pose.position.y && update_pose.position.z == original_pose.position.z){
                // Save the index for further process
                correctProductIdx.push_back(originalIdx);
                correctIdxMap[originalIdx] = updateIdx;
                break;
            }
        }
    }

    // Find the wrong products being placed and removed it
    std::vector<int> eraseShipmentIdx;
    for (auto m : placed_order){
        // If its placed and at its correct postion
        auto it = std::find(correctProductIdx.begin(), correctProductIdx.end(), m.first);
        if (it != correctProductIdx.end()){
            eraseShipmentIdx.push_back(correctIdxMap[*it]);
            continue;
        }
        // If its place but not at its correct position
        else{
            if (agv_id == 1){
                auto tmp = arm1_.PickPart(m.second);
                arm1_.PrepareRobot("drop");
                arm1_.GripperToggle(false);
                arm1_.PrepareRobot("end");
            }
            else {
                auto tmp = arm2_.PickPart(m.second);
                arm2_.PrepareRobot("drop");
                arm2_.GripperToggle(false);
                arm2_.PrepareRobot("end");
            }
        }
    }

    // Erase the product already placed in the updated order
    std::sort(eraseShipmentIdx.begin(), eraseShipmentIdx.end());
    int tmpCount = 0; // offset the index
    for (const auto &eraseIdx : eraseShipmentIdx){
        update_shipment.products.erase(update_shipment.products.begin()+eraseIdx-tmpCount);
        tmpCount++;
    }

    // Update the shipment information
    original_shipment = update_shipment; // doesn't matter for only 1 update
    shipment_in_progress = update_shipment;
    order_id = compare_id;
    agv_id = (update_shipment.agv_id == "any") ? 1 : update_shipment.agv_id.back() - '0';

    ROS_WARN(">>>>>> Update Order Comes in <<<<<<");
    ROS_INFO("==========Print Shipment Information==========");
    ROS_INFO("New order_id = %s", order_id.c_str());
    ROS_INFO("New agv_id = %d", agv_id);
    ROS_INFO("Update Complete");
    return true;
}



