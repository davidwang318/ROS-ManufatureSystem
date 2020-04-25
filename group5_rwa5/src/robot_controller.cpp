//
// Created by zeid on 2/27/20.
//
#include "robot_controller.h"

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotController::RobotController(std::string arm_id): robot_controller_nh_("/ariac/"+arm_id),
                                                      robot_controller_options("manipulator",
                                                      "/ariac/"+arm_id+"/robot_description", robot_controller_nh_),
                                                      robot_move_group_(robot_controller_options) {

    robot_move_group_.setPlanningTime(20);
    robot_move_group_.setNumPlanningAttempts(10);
    robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
    robot_move_group_.setMaxVelocityScalingFactor(0.9);
    robot_move_group_.setMaxAccelerationScalingFactor(0.9);
    robot_move_group_.allowReplanning(true);
    
    counter_ = 0;
    drop_flag_ = false;
    offset_ = 0.025;

    // Add an attribute to prevent occlusion in PrepareRobot function
    poseState = "";
    arm_id_ = arm_id;

    //--These are joint positions used for the home position
    if(arm_id == "arm1"){
        binJointPose1_ = {1.2, 3.14, -0.7, 2.2, 3.2, 4.7, 0};
        binJointPose2_ = {0.4, 3.14, -0.7, 2.2, 3.2, 4.7, 0};
        binJointPose3_ = {-0.4, 3.14, -0.7, 2.2, 3.2, 4.7, 0};
        binJointPose4_ = {-1.2, 3.14, -0.7, 2.2, 3.2, 4.7, 0};
        binJointPose5_ = binJointPose4_;
        beltJointPose_ = {1, 0, -1.1, 1.9, 3.9, 4.7, 0};
        dropJointPose_ = {2.0, 2.3, -0.5, 1.3, 3.9, 4.7, 0};
        transJointPose_ = {0, 4.21, -1.1, 1.9, 3.9, 4.7, 0};
        endJointPose_ = {2.0, 1.45, -1.1, 1.9, 3.9, 4.7, 0};

        // Add 2 poses for Pick up.
        railLeftPose_ = {0.26, -1.63, -1.63, 1.63, 3.14, -1.63, 0}; // -- Calibrated?
        railRightPose_ = {-0.195, -1.5, -1.63, 1.50, 0, 1.64, 0}; // -- Calibrated?

        // Add 1 special pose to prevent occlusion
        occluJointPose_ = {-1.2, 3.14, -1.1, 2.2, 3.2, 4.7, 0};

        //--topic used to get the status of the gripper
        gripper_subscriber_ = gripper_nh_.subscribe(
                "/ariac/arm1/gripper/state", 10, &RobotController::GripperCallback, this);

        agv_tf_listener_.waitForTransform("world", "kit_tray_1",
                                          ros::Time(0), ros::Duration(10));
        agv_tf_listener_.lookupTransform("/world", "/kit_tray_1",
                                         ros::Time(0), agv_tf_transform_);
        agv_position_.position.x = agv_tf_transform_.getOrigin().x();
        agv_position_.position.y = agv_tf_transform_.getOrigin().y();
        agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

        gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
                "/ariac/arm1/gripper/control");

    }
    else{
        binJointPose2_ = binJointPose3_;
        binJointPose3_ = {1.1, 2.6, -0.7, 2.37, 3, 4.7, 0};
        binJointPose4_ = {0.7, 3.14, -0.7, 2.2, 3.2, 4.7, 0};
        binJointPose5_ = {-0.1, 3.14, -0.7, 2.2, 3.2, 4.7, 0};
        binJointPose6_ = {-0.9, 3.14, -0.7, 2.2, 3.2, 4.7, 0};
        dropJointPose_ = {-1.5, 3.9, -0.5, 1.3, 3.9, 4.7, 0};
        transJointPose_ = {0, 1.57, -1.1, 1.9, 3.9, 4.7, 0};
        endJointPose_ = {-1.5, 4.5, -0.5, 1.3, 3.9, 4.7, 0};
        occluJointPose_ = {1.1, 2.6, -1.1, 2.37, 3, 4.7, 0};

        //--topic used to get the status of the gripper
        gripper_subscriber_ = gripper_nh_.subscribe(
                "/ariac/arm2/gripper/state", 10, &RobotController::GripperCallback, this);

        agv_tf_listener_.waitForTransform("world", "kit_tray_2",
                                          ros::Time(0), ros::Duration(10));
        agv_tf_listener_.lookupTransform("/world", "/kit_tray_2",
                                         ros::Time(0), agv_tf_transform_);
        agv_position_.position.x = agv_tf_transform_.getOrigin().x();
        agv_position_.position.y = agv_tf_transform_.getOrigin().y();
        agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

        gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
                "/ariac/arm2/gripper/control");
    }
}

RobotController::~RobotController() {}

bool RobotController::Planner() {
    ROS_INFO_STREAM("Planning started...");
    if (robot_move_group_.plan(robot_planner_) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        plan_success_ = true;
        ROS_INFO_STREAM("Planner succeeded!");
    } else {
        plan_success_ = false;
        ROS_WARN_STREAM("Planner failed!");
    }

    return plan_success_;
}


void RobotController::Execute() {
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.0).sleep();
    }
}

void RobotController::GoToTarget(const geometry_msgs::Pose& pose) {
    target_pose_.orientation = fixed_orientation_;
    target_pose_.position = pose.position;
    ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToTarget(std::initializer_list<geometry_msgs::Pose> list) {
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::vector<geometry_msgs::Pose> waypoints;
    for (auto i : list) {
        i.orientation.x = fixed_orientation_.x;
        i.orientation.y = fixed_orientation_.y;
        i.orientation.z = fixed_orientation_.z;
        i.orientation.w = fixed_orientation_.w;
        waypoints.emplace_back(i);
    }

    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    ROS_WARN_STREAM("Fraction: " << fraction * 100);
    // ros::Duration(5.0).sleep();

    robot_planner_.trajectory_ = traj;

    //if (fraction >= 0.3) {
        robot_move_group_.execute(robot_planner_);
        // ros::Duration(5.0).sleep();
//    } else {
//        ROS_ERROR_STREAM("Safe Trajectory not foundfrac!");
//    }
}

void RobotController::PrepareRobot(std::string task) {

    std::vector<double> jointPose;
    if (task == "bin1") jointPose = binJointPose1_;
    else if (task == "bin2") jointPose = binJointPose2_;    
    else if (task == "bin3") jointPose = binJointPose3_;
    else if (task == "bin4") jointPose = binJointPose4_;
    else if (task == "bin5") jointPose = binJointPose5_;
    else if (task == "bin6") jointPose = binJointPose6_;
    else if (task == "belt") jointPose = beltJointPose_;
    else if (task == "drop") jointPose = dropJointPose_;
    else if (task == "trans") jointPose = transJointPose_;
    else if (task == "occlusion") jointPose = occluJointPose_;
    else if (task == "railArm1") jointPose = railLeftPose_;
    else if (task == "railArm2") jointPose = railRightPose_;
    else jointPose = endJointPose_;

    // Handle possible occlusion
    if (arm_id_ == "arm1"){
        if(task == "end" && poseState == "bin5") PrepareRobot("occlusion");
    }
    else{
        if(task == "end" && poseState == "bin2") PrepareRobot("occlusion");
    }

    poseState = task;

    robot_move_group_.setJointValueTarget(jointPose);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        // ros::Duration(1.5).sleep();
    }
    ROS_INFO("Send Robot To %s Position Complete!", task.c_str());

    robot_tf_listener_.waitForTransform(arm_id_+"_linear_arm_actuator", arm_id_+"_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/"+arm_id_+"_linear_arm_actuator", "/"+arm_id_+"_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();

    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);


    robot_tf_listener_.waitForTransform("world", arm_id_+"_ee_link", ros::Time(0),
                                            ros::Duration(10));
    robot_tf_listener_.lookupTransform("/world", "/"+arm_id_+"_ee_link", ros::Time(0),
                                           robot_tf_transform_);

    home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
    home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
    home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
    home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
    home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
    home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
    home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();
    // ros::Duration(2.0).sleep();
}

void RobotController::GripperToggle(const bool& state) {
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    // ros::Duration(3.0).sleep();
    // if (gripper_client_.call(gripper_service_)) {
    if (gripper_service_.response.success) {
        ROS_INFO_STREAM("Gripper activated!");
    } else {
        ROS_WARN_STREAM("Gripper activation failed!");
    }
}

bool RobotController::DropPart(geometry_msgs::Pose part_pose) {
    // counter_++;

    drop_flag_ = true;

    ros::spinOnce();
    ROS_INFO_STREAM("Placing phase activated...");

    if (gripper_state_){//--while the part is still attached to the gripper
        //--move the robot to the end of the rail
         ROS_INFO_STREAM("Moving towards AGV...");

         PrepareRobot("end");
         ros::Duration(0.5).sleep();

         part_pose.position.z += 0.1;
         ROS_INFO_STREAM("Go to the correct position...");
         this->GoToTarget(part_pose);

         ROS_INFO_STREAM("Releasing the gripper...");
         this->GripperToggle(false);
         
         part_pose.position.z += 0.2;
         this->GoToTarget(part_pose);
    }

    drop_flag_ = false;
    return gripper_state_;
}

void RobotController::GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}


bool RobotController::PickPart(geometry_msgs::Pose& part_pose) {
    // gripper_state = false;
    // pick = true;
    //ROS_INFO_STREAM("fixed_orientation_" << part_pose.orros::Duration(3.0).sleep();ientation = fixed_orientation_);
    //ROS_WARN_STREAM("Picking the part...");
    int pickCount = 0;

    ROS_INFO_STREAM("Moving to part...");
    part_pose.position.z = part_pose.position.z + offset_;
    auto temp_pose_1 = part_pose;
    temp_pose_1.position.z += 0.3;

    this->GoToTarget({temp_pose_1, part_pose});
    // GoToTarget(part_pose);

    ROS_INFO_STREAM("Actuating the gripper..." << part_pose.position.z);
    this->GripperToggle(true);
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    while (!gripper_state_) {
        pickCount++;
        ROS_INFO_STREAM(" Keep Actuating the gripper...");
        this->GripperToggle(true);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        if(pickCount >= 10) return false;
    }

    ROS_INFO_STREAM("Going to waypoint...");
    this->GoToTarget(temp_pose_1);
    return gripper_state_;
}

/*
 * Adding Flipping Functionality if a part needs to be flipped.
 */
void RobotController::FlipPart(RobotController *arm) {
    ROS_WARN_STREAM("RobotController::FlipPart() > Function call received.");
    auto otherArm = arm;

    if (otherArm->arm_id_ == "arm2") std::cout << "Arm 1 takes help of Arm2" << std::endl;
    else std::cout << "Arm 2 takes help of Arm 1" << std::endl;

    // Send arms to appropriate home Positions
    if (otherArm->arm_id_ == "arm2"){
        this->PrepareRobot("bin3");
        otherArm->PrepareRobot("bin5");
        this->PrepareRobot("railArm1");
        otherArm->PrepareRobot("railArm2");
    }
    else{
        this->PrepareRobot("bin5");
        otherArm->PrepareRobot("bin3");
        this->PrepareRobot("railArm2");
        otherArm->PrepareRobot("railArm1");
    }

    // Move Exchange the parts.

    // Move the current Arm away

    // Drop the Part

    // Move the current Arm closer

    // CurrentArm grabs the part

    // Make sure each arms are in good home Position.


}
