//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <string> 
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>


AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);
    agv_id = 0;
    trans_pose.position.x = 0.3;
    trans_pose.position.y = 0;
    trans_pose.position.z = 1;

    arm1_.PrepareRobot("bin2");
    arm2_.PrepareRobot("bin6");
}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN(">>>>> OrderCallback");
    received_orders_.push_back(*order_msg);
}

std::string AriacOrderManager::GetProductFrame(std::string product_type, int num) {
    //--Grab the last one from the list then remove it
    if (!product_frame_list_.empty()) {
        auto frame = product_frame_list_[product_type].back();
        camera_.product_frame_list_[num][product_type].pop_back();
        return frame;
    } else {
        ROS_ERROR_STREAM("No product frame found for " << product_type);
        ros::shutdown();
    }
}

// Stage 1: belt
//////////////////////////////////////////////////////////////////////////////////////////

void AriacOrderManager::PlanBeltStrategy(){
    ROS_INFO_STREAM("[AriacOrderManager]:[PlanStrategy]: Called");
    ros::spinOnce();
    // Print all the order
    this -> PrintOrder();

    // Find the empty space
    FindEmptySpace();

    // Initialize the counter of parts in the bins
    int bin_pulley = 0, bin_gasket = 0, bin_piston = 0, bin_gear = 0, bin_disk = 0;
    for (int i = 1; i < 7; i++){
        product_frame_list_ = camera_.get_product_frame_list(i);
        bin_pulley += product_frame_list_["pulley_part"].size();
        bin_gasket += product_frame_list_["gasket_part"].size();
        bin_piston += product_frame_list_["piston_rod_part"].size();
        bin_gear += product_frame_list_["gear_part"].size();
        bin_disk += product_frame_list_["disk_part"].size();
    }

    // Initialize the counter of parts in the order
    int order_pulley = 0, order_gasket = 0, order_piston = 0, order_gear = 0, order_disk = 0;
    for (const auto &order : received_orders_){
        auto shipments = order.shipments;
        for (const auto& shipment : shipments){
        	for(const auto &product : shipment.products){
		        if(product.type == "pulley_part") {order_pulley++; continue;}
		        else if(product.type == "gasket_part") {order_gasket++; continue;}
		        else if(product.type == "piston_rod_part") {order_piston++; continue;}
		        else if(product.type == "gear_part") {order_gear++; continue;}
		        else if(product.type == "disk_part") {order_disk++; continue;}
		        else ROS_WARN("!!! ERROR, Missing part type !!!");
	    	}
        }
    }

    // Calculate the strategy for belt
    int belt_pulley = 0, belt_gasket = 0, belt_piston = 0, belt_gear = 0, belt_disk = 0;
    belt_pulley = std::max(0, order_pulley - bin_pulley);
    belt_gasket = std::max(0, order_gasket - bin_gasket);
    belt_piston = std::max(0, order_piston - bin_piston);
    belt_gear   = std::max(0, order_gear   - bin_gear);
    belt_disk   = std::max(0, order_disk   - bin_disk);
    int belt_num = belt_pulley + belt_gasket + belt_piston + belt_gear + belt_disk; // total number of parts picked from belt
    belt_num = std::min(belt_num, empty_count);
    
    // Construct the type and number
    if (belt_num == 0) return;
    belt_part_num["pulley_part"] = belt_pulley;
    belt_part_num["gasket_part"] = belt_gasket;
    belt_part_num["piston_part"] = belt_piston;
    belt_part_num["gear_part"] = belt_gear;
    belt_part_num["disk_part"] = belt_disk;

    // // Check
    // std::cout << "==== Belt Strategy Check ====" << std::endl;
    // for (auto tmp : belt_part_num){
    // 	std::cout << tmp.first << "=" << tmp.second << std::endl;
    // }

    // Pick part from the belt
    while(belt_num > 0 && !belt_end){
    	if(PickFromBelt()) belt_num--;
    }

    return;
}

void AriacOrderManager::FindEmptySpace(){
	std::cout << "start finding empty space" << std::endl;
	// t: top, b: buttom, l: left, r: right, cen: center
	std::pair<double, double> tl{0, 2.216}, tr{0, 1.616}, bl{-0.6, 2.216}, br{-0.6, 1.616}, cen{-0.3, 1.916};

	// i: bin number
	for (int i = 1; i < 7; i++){
		std::pair<double, double> bin_tl {tl.first, tl.second - (i-1)*0.766},
		                          bin_tr {tr.first, tr.second - (i-1)*0.766},
		                          bin_bl {bl.first, bl.second - (i-1)*0.766},
		                          bin_br {br.first, br.second - (i-1)*0.766},
		                          bin_cen {cen.first, cen.second - (i-1)*0.766};

		bool tl_flag = true, tr_flag = true, bl_flag = true, br_flag = true; 
		auto frame_list = camera_.get_product_frame_list(i);

		// Check every product we have on each bins
		for (auto product_type_frame : frame_list){
			for(auto product_frame : product_type_frame.second){
				auto part_pose = camera_.GetPartPose("/world", product_frame);
				auto part_x = part_pose.position.x;
				auto part_y = part_pose.position.y;
				// tl corner:
				if(tl_flag && part_x < bin_tl.first  && part_x > bin_cen.first
					       && part_y < bin_tl.second && part_y > bin_cen.second){
					tl_flag = false;
				}
				// tr corner:
				if(tr_flag && part_x < bin_tr.first  && part_x > bin_cen.first
					       && part_y > bin_tr.second && part_y < bin_cen.second){
					tr_flag = false;
				}
				// bl corner:
				if(bl_flag && part_x > bin_bl.first  && part_x < bin_cen.first
					       && part_y < bin_bl.second && part_y > bin_cen.second){
					bl_flag = false;
				}
				// br corner:
			    if(br_flag && part_x > bin_br.first  && part_x < bin_cen.first
					       && part_y > bin_br.second && part_y < bin_cen.second){
					br_flag = false;
				}
			}
		}

		// Create empty space:
		geometry_msgs::Pose empty_pose;
    	empty_pose.position.z = 0.82;
		if(tl_flag){
			empty_pose.position.x = (bin_tl.first+bin_cen.first)/2.0;
			empty_pose.position.y = (bin_tl.second+bin_cen.second)/2.0;
			empty_place[i].push_back(empty_pose);
			empty_count++;
		}
		if(tr_flag){
			empty_pose.position.x = (bin_tr.first+bin_cen.first)/2.0;
			empty_pose.position.y = (bin_tr.second+bin_cen.second)/2.0;
			empty_place[i].push_back(empty_pose);
			empty_count++;
		}
		if(bl_flag){
			empty_pose.position.x = (bin_bl.first+bin_cen.first)/2.0;
			empty_pose.position.y = (bin_bl.second+bin_cen.second)/2.0;
			empty_place[i].push_back(empty_pose);
			empty_count++;
		}
		if(br_flag){
			empty_pose.position.x = (bin_br.first+bin_cen.first)/2.0;
			empty_pose.position.y = (bin_br.second+bin_cen.second)/2.0;
			empty_place[i].push_back(empty_pose);
			empty_count++;
		}
	}
	// // Check
	// std::cout << "==== Find Empty Place Check ====" << std::endl;
	// for (auto m : empty_place){
	// 	std::cout << "--Bin" << m.first << std::endl;
	// 	for (auto p : m.second){
	// 		std:: cout << "x: " << p.position.x << "  y: " << p.position.y << std::endl;
	// 	}
	// }
	// std::cout << "Empty count: " << empty_count << std::endl;
}

// Function to pick a part from the belt
bool AriacOrderManager::PickFromBelt() {
    // Initialize some paremeters
    arm1_.PrepareRobot("belt");
    camera_.product_frame_list_[0].clear(); // To have correct offset
    state_ = "belt";
    std::string product_frame;
    int whichBin = 0;

    // Start waiting the target parts
    ROS_WARN(">>>>>> Waiting parts...");
    bool hold = true;
    bool pick_n_place_success{false};
    while(hold){
        ros::spinOnce();
        product_frame_list_ = camera_.get_product_frame_list(0);

        if(!product_frame_list_.empty()){
            std::cout << "entering finding stage" << std::endl;

            for (auto& m : belt_part_num){
            	std::cout << m.first << std::endl;
            	if(product_frame_list_.count(m.first)){
            		std::cout << "find part: " << m.first << std::endl;
            		product_type_pose_.first = m.first;
            		// empty_map.first: type, empty_map.second: pose
            		for(auto& empty_map : empty_place){
            			// if still have empty position
            			if(empty_map.second.size() > 0){
            				product_type_pose_.second = empty_map.second.back();
       						whichBin = empty_map.first;
       						// Call the PickAndPlace function to pick parts
    						if (whichBin == 6) pick_n_place_success = PickAndPlaceBelt("bin"+std::to_string(whichBin), true);
    						else pick_n_place_success = PickAndPlaceBelt("bin"+std::to_string(whichBin), false);
    						// Check is it working
    						if (pick_n_place_success) empty_map.second.pop_back();
            				break; 
            			}
            		}
            		m.second--;
            		hold = false;		
            		break;
            	}
            }
           	if(hold) camera_.product_frame_list_[0].clear(); // Prevent infinite loop;
        }
    }

	// // Check Frame construction:
	// for (int i = 1; i < 7; i++){
	// 	auto tmp = camera_.get_product_frame_list(i);
	// 	std::cout << "--Bin" << i << std::endl;
	// 	for(auto t : tmp){
	// 		std::cout << t.first << std::endl;
	// 		for(auto a : t.second){
	// 			std::cout << a << std::endl;
	// 		}
	// 	}
	// 	std::cout << "-----------------" << std::endl;
	// }

    return pick_n_place_success;
}

bool AriacOrderManager::PickAndPlaceBelt(std::string whichBin, bool transition){
	std::cout << "entering PickAndPlaceBelt" << std::endl;
    // Print the part type and frame
    std::string product_type = product_type_pose_.first;
    std::string product_frame = this->GetProductFrame(product_type, 0);

    // Get and offset the part pose    
    auto part_pose = camera_.GetPartPose("/world",product_frame);
    this -> offsetPose(product_type, part_pose);

    // Pick the part 
    bool failed_pick = arm1_.PickPart(part_pose);
    if (!failed_pick) return false;

    // Transfer the product if needed
    if (transition){
    	// arm1_.PrepareRobot("end");
    	arm1_.PrepareRobot("trans");
    	auto tmp = trans_pose;
        auto trans_pose_up = trans_pose;
        trans_pose_up.position.z += 0.3;
        this -> offsetPose(product_type, trans_pose);
        arm1_.GoToTarget(trans_pose);
        arm1_.GripperToggle(false);
        arm1_.GoToTarget(trans_pose_up);
        arm1_.PrepareRobot("belt");
        arm2_.PrepareRobot("trans");
        trans_pose = tmp;
        product_frame[15] = '7';
        part_pose = camera_.GetPartPose("/world",product_frame);
        this -> offsetPose(product_type, part_pose);
        failed_pick = arm2_.PickPart(part_pose);
        while(!failed_pick){
            part_pose.position.z -= 0.03;
            failed_pick = arm2_.PickPart(part_pose);      
        }
        arm2_.PrepareRobot(whichBin);
        arm2_.PrepareRobot(whichBin);
    	arm2_.GoToTarget(product_type_pose_.second);
    	arm2_.GripperToggle(false);
    	product_type_pose_.second.position.z += 0.3;
    	arm2_.GoToTarget(product_type_pose_.second);

        // Update the product_frame_list_
	    product_frame[15] = whichBin.back();
	    int bin_num = whichBin.back() - '0';
	    camera_.product_frame_list_[bin_num][product_type].push_back(product_frame);

        return true;
    }

    // place
    // arm1_.PrepareRobot("end"); // solve conflict
    arm1_.PrepareRobot(whichBin);
    if (product_type_pose_.first == "pulley_part"){
        auto tmp_pose = product_type_pose_.second;
        tmp_pose.position.z += 0.3;
        arm1_.GoToTarget(tmp_pose);
    }
    arm1_.GoToTarget(product_type_pose_.second);
    arm1_.GripperToggle(false);
    product_type_pose_.second.position.z += 0.3;
    arm1_.GoToTarget(product_type_pose_.second);

    // Update the product_frame_list_
    product_frame[15] = whichBin.back();
    int bin_num = whichBin.back() - '0';
    camera_.product_frame_list_[bin_num][product_type].push_back(product_frame);

    // // check
    // auto tmp = camera_.GetPartPose("/world",product_frame);
    // std::cout << "check whether frame is working" <<std:: endl;
    // std::cout << product_frame << std::endl;
    // std::cout << "x=" << tmp.position.x << " y=" << tmp.position.y << " z=" << tmp.position.z <<std::endl;
    return true;
}


// Stage 2: bin
//////////////////////////////////////////////////////////////////////////////////////////

void AriacOrderManager::PlanStrategyShipment(){

    // Get the order_id
    for (int i = 0; i < received_orders_.size(); i++){
    	auto order = received_orders_[i];
        auto order_id_ = order.order_id;
        // Make sure its not the updated order
        if (order_id_.size() == 7){
        	order_id = order_id_;
        	for (int i = 0; i < order.shipments.size(); i++){
		        // Clear the helper
		        placed_order.clear();
		        
		        // Initialize the shipment parameters
		        original_shipment = order.shipments[i];
		        shipment_in_progress = original_shipment;
		        shipment_num = i;
		        agv_id = (original_shipment.agv_id == "any") ? 1 : original_shipment.agv_id.back() - '0';
		        int fromBin = 0;

		        do{
		            // Check update
		            ros::spinOnce();
		            auto dummy = CheckUpdate();

		            // Initialize the counter of parts on bins
		            int bin_pulley = 0, bin_gasket = 0, bin_piston = 0, bin_gear = 0, bin_disk = 0;
		            for (int i = 1; i < 7; i++){
		                product_frame_list_ = camera_.get_product_frame_list(i);
		                bin_pulley += product_frame_list_["pulley_part"].size();
		                bin_gasket += product_frame_list_["gasket_part"].size();
		                bin_piston += product_frame_list_["piston_rod_part"].size();
		                bin_gear += product_frame_list_["gear_part"].size();
		                bin_disk += product_frame_list_["disk_part"].size();
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

		            // Calculate the parts needed to be picked from bins 
		            fromBin = std::min(bin_pulley, order_pulley) + std::min(bin_gasket, order_gasket)
		                    + std::min(bin_piston, order_piston) + std::min(bin_gear, order_gear) + std::min(bin_disk, order_disk);

		            // Printing
		            ROS_INFO("\n\n==========Print Order Information==========");
		            ROS_INFO("Order id = %s", order_id.c_str());
		            ROS_INFO("Shipment number = %d", shipment_num);
		            ROS_INFO("Agv_id = %d", agv_id);

		            ROS_INFO("==========Print Strategy==========");
		            ROS_INFO("Pick %d Parts From Bins", fromBin);

		            // Execute the strategy
		            if(fromBin > 0) {
		            	if(PickFromBin()) fromBin--;
		            }

		            // Final Check
		            if(order_id.size() == 7 && fromBin == 0){
		                ros::Duration(5.5).sleep();
		                ros::spinOnce();
		                if(CheckUpdate()) fromBin = 1; // Force to run the loop
		            }
		        }
		        while(fromBin > 0 && !bin_end);

                // TODO: Final Check...
                /////////////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////////////

		        // Submit the agv
		        SubmitAGV(agv_id);
		    } 
        }
    }
    return;
}

bool AriacOrderManager::PickFromBin() {
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
    if(!find){
    	bin_end = true;
    	return false;
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
    std::cout << "WhichBin: " << whichBin << std::endl;

    // Call the pick function
    bool pick_n_place_success{false};
    int bin_num = whichBin.back() - '0';
    if (agv_id == 1){
        if(whichBin == "bin6"){
            arm2_.PrepareRobot(whichBin);
            std::cout << "case1" << std::endl;
            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id, 2, true, trueIdx, bin_num);
        }
        else{
            if(whichBin == "bin5") arm2_.PrepareRobot("bin6");
            arm1_.PrepareRobot(whichBin);
            std::cout << "case2" << std::endl;
            pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id, 1, false, trueIdx, bin_num);
        }
    }
    else{
        if(whichBin == "bin1" || whichBin == "bin2"){
            arm1_.PrepareRobot(whichBin);
            std::cout << "case3" << std::endl;
            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id, 1, true, trueIdx, bin_num);
        }
        else{
            if(whichBin == "bin2") arm1_.PrepareRobot("bin1");
            arm2_.PrepareRobot(whichBin);
            std::cout << "case4" << std::endl;
            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id, 2, false, trueIdx, bin_num);
        }
    }


    if (pick_n_place_success) UpdateOrder(productIdx_);

    return pick_n_place_success;
}

bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id, int whichArm, bool transition, int placed_index, int bin_num) {
    auto pri_arm = &arm1_, sec_arm = &arm2_;
    if (whichArm == 2) pri_arm = &arm2_, sec_arm = &arm1_;

    // Print the part type and frame
    std::string product_type = product_type_pose.first;
    std::string product_frame = this->GetProductFrame(product_type, bin_num);

    // Get and offset the part pose    
    auto part_pose = camera_.GetPartPose("/world",product_frame);
    this -> offsetPose(product_type, part_pose);

    // Pick the part 
    bool failed_pick = pri_arm -> PickPart(part_pose);
    while(!failed_pick){
        part_pose.position.z -= 0.03;
        failed_pick = pri_arm -> PickPart(part_pose);
    }

    // Condition for flipping
    tf2::Quaternion quaternionOfPartPose;
    tf2::fromMsg(product_type_pose.second.orientation, quaternionOfPartPose);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternionOfPartPose).getRPY(roll, pitch, yaw); //  sometimes I am getting NAN values.
    bool isFlippingRequired = false;

    ROS_INFO_STREAM("[OM]:[PickAndPlace]: R-P-Y of Part: " << roll << "; " << pitch << "; " << yaw);
    if (abs(roll)> 3.12 && abs(roll) < 3.16) {
        ROS_WARN_STREAM("[OrderManager][PickAndPlace]: This parts needs to be flipped");
        isFlippingRequired = true;
    }

    // Flipped the part
    if(isFlippingRequired){
        // Prepare the robot
        pri_arm -> PrepareRobot("rail");
        sec_arm -> PrepareRobot("rail");
        pri_arm -> GoToTarget(pri_arm -> exchange_pose);
        sec_arm -> GoToTarget(sec_arm -> exchange_pose);

        // Exchanging the part
        sec_arm -> GripperToggle(true);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        while (!sec_arm -> gripper_state_) {
            ROS_INFO_STREAM(" Keep Actuating the gripper for exchanging the part...");
            sec_arm -> GripperToggle(true);
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
        pri_arm -> GripperToggle(false);

        // Seperate
        auto seperate_pose = pri_arm -> exchange_pose;
        if(whichArm == 1) seperate_pose.position.y += 0.2;
        else seperate_pose.position.y -= 0.2;
        pri_arm -> GoToTarget(seperate_pose);

        // Swap pri and sec: pri arm is the arm with the part
        if (whichArm == 1) whichArm = 2;
        else whichArm = 1; 
        std::swap(pri_arm, sec_arm);
        transition = !transition;
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
        ros::spinOnce(); // make sure its the newest frame 
        if (whichArm == 1) pri_arm -> PrepareRobot("bin2");
        else pri_arm -> PrepareRobot("bin5");
        sec_arm -> PrepareRobot("trans");
        trans_pose = tmp;

        if (whichArm == 1) whichArm = 2;
        else whichArm = 1; 
        std::swap(pri_arm, sec_arm);

        product_frame[15] = '7';
        part_pose = camera_.GetPartPose("/world",product_frame);
        if(isFlippingRequired) part_pose.position.z -= 0.03; // Need to be improved!
        else this -> offsetPose(product_type, part_pose);

        failed_pick = pri_arm -> PickPart(part_pose);
        while(!failed_pick){
            part_pose.position.z -= 0.03;
            failed_pick = pri_arm -> PickPart(part_pose);      
        }

        ros::Duration(0.5).sleep();
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
    bool dropCase = pri_arm -> DropPart(StampedPose_out.pose);
    if(dropCase) PickAndPlaceDrop(whichArm, product_type, product_frame, StampedPose_out.pose);

    if(camera_.CheckForQuality(whichArm)){
        auto tmp = pri_arm -> PickPart(StampedPose_out.pose);
        pri_arm -> PrepareRobot("drop");
        pri_arm -> GripperToggle(false);
        pri_arm -> PrepareRobot("end");
        return false;
    }

    // Save the sequence and position of the part for update check 
    placed_order[placed_index] = StampedPose_out.pose;

 //   	// Check Frame construction:
	// for (int i = 1; i < 7; i++){
	// 	auto tmp = camera_.get_product_frame_list(i);
	// 	std::cout << "--Bin" << i << std::endl;
	// 	for(auto t : tmp){
	// 		std::cout << t.first << std::endl;
	// 		for(auto a : t.second){
	// 			std::cout << a << std::endl;
	// 		}
	// 	}
	// 	std::cout << "-----------------" << std::endl;
	// }

    return true;
}

void AriacOrderManager::PickAndPlaceDrop(int whichArm, std::string product_type, std::string product_frame, geometry_msgs::Pose end_pose){
    // Get the current arm and frame
    auto arm = &arm1_;
    product_frame[15] = '8';
    if(whichArm == 2) {
        arm = &arm2_;
        product_frame[15] = '9';
    }

    // Find the pose
    auto part_pose = camera_.GetPartPose("/world",product_frame);
    this -> offsetPose(product_type, part_pose);

    // Pick the part
    bool failed_pick = arm -> PickPart(part_pose);
    while(!failed_pick){
        part_pose.position.z -= 0.03;
        failed_pick = arm -> PickPart(part_pose);
    }

    // Place it to the correct pose
    end_pose.position.z += 0.1;
    arm -> GoToTarget(end_pose);
    arm -> GripperToggle(false);       
    arm -> PrepareRobot("end");

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


void AriacOrderManager::offsetPose(std::string type_, geometry_msgs::Pose& pose_){
    if (state_ == "belt"){
        if(type_ == "gasket_part"){
            pose_.position.x -= 0.007;
            pose_.position.y -= 0.306;
            pose_.position.z += 0.0089;
        }
        else if(type_ == "piston_rod_part"){
            pose_.position.x += 0.005;
            pose_.position.y -= 0.2711;
            pose_.position.z -= 0.0079;
        }
        else if(type_ == "gear_part"){
            pose_.position.x += 0.005;
            pose_.position.y -= 0.349;
            pose_.position.z += 0.00285;
        }
        else if(type_ == "disk_part"){
            pose_.position.x += 0.0035;
            pose_.position.y -= 0.3117;
            pose_.position.z += 0.01379;
        }
        else if(type_ == "pulley_part"){
            pose_.position.x += 0.01;
            pose_.position.y -= 0.35;
            pose_.position.z += 0.06148;
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

bool AriacOrderManager::CheckUpdate(){
    std::cout << "~~~~~~~~~Checking Update~~~~~~~~~~" << std::endl;
    // Do nothing if its an updated order
    if (order_id.size() > 7) return false;

    // Initialize the parameters
    int update_order_idx = -1;
    osrf_gear::Shipment update_shipment;

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
    std::cout << "====== Find new order ======" << std::endl;

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



