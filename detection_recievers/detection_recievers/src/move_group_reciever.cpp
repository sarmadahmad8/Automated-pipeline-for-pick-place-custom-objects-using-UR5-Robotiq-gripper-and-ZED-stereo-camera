#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sstream>
#include <chrono>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class MoveGroupReceiver : public rclcpp::Node {
public:
    MoveGroupReceiver() : Node("move_group_reciever") {
        // Create the subscriber
        position_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/target_position", 10,
            [this](geometry_msgs::msg::Pose::SharedPtr msg) {
                target_pose1.position.x = msg->position.x;  
                target_pose1.position.y = msg->position.y;  
                target_pose1.position.z = msg->position.z;
                target_pose1.orientation.x = -0.707;
                target_pose1.orientation.y = -0.005;
                target_pose1.orientation.z = 0.006;
                target_pose1.orientation.w = 0.707;

                // Unsubscribe after processing the first message
                //position_subscriber_.reset(); // This stops the subscription
            }
        );
                // Subscriber for /fruit_width
        fruit_width_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/fruit_width", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                fruit_width_ = msg->data;
                //RCLCPP_INFO(LOGGER, "Received fruit width: %f", fruit_width_);
            }
        );

        // Subscriber for /distance_to_fruit
        distance_to_fruit_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/distance_to_fruit", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                distance_to_fruit_ = msg->data;
                //RCLCPP_INFO(LOGGER, "Received distance to fruit: %f", distance_to_fruit_);
            }
        );
    }

    geometry_msgs::msg::Pose getTargetPose() {
        return target_pose1;
    }
        // Getter functions to access the subscribed values
    float getFruitWidth() const { return fruit_width_; }
    float getDistanceToFruit() const { return distance_to_fruit_; }

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr position_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fruit_width_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_to_fruit_subscriber_;
    
    geometry_msgs::msg::Pose target_pose1;
    float fruit_width_ = 0.0;
    float distance_to_fruit_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_reciever = std::make_shared<MoveGroupReceiver>();

    // Single-threaded executor for the robot state monitor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_reciever);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Set the planning group for UR manipulator
    static const std::string PLANNING_GROUP = "ur_manipulator";

    // Initialize MoveGroupInterface with UR manipulator planning group
    moveit::planning_interface::MoveGroupInterface move_group(move_group_reciever, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization setup
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_reciever, "world", "/display_planned_path",
                                                        move_group.getRobotModel());

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.5;
    visual_tools.publishText(text_pose, "UR Manipulator Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Display basic information about the robot
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    // // Plan and visualize
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit_msgs::msg::CollisionObject object_to_attach;
    object_to_attach.id = "cylinder1";
    shape_msgs::msg::SolidPrimitive primitive;
    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.25;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.08;
    object_to_attach.header.frame_id = move_group.getEndEffectorLink();
    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.103 - 0.135;
    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);
    RCLCPP_INFO(LOGGER, "Attach the object to the robot");
    std::vector<std::string> touch_links;
    touch_links.push_back("tool0");
    touch_links.push_back("wrist_3_link");
    move_group.attachObject(object_to_attach.id, "tool0", touch_links);

    visual_tools.publishText(text_pose, "Object_attached_to_robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getEndEffectorLink();
    collision_object.id = "box1";
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.15;
    primitive.dimensions[primitive.BOX_Y] = 0.04;
    primitive.dimensions[primitive.BOX_Z] = 0.04;
    geometry_msgs::msg::Pose grab_pose1;
    grab_pose1.orientation.w = 1.0;
    grab_pose1.position.y = -0.08;
    grab_pose1.position.z = -0.064-0.135;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(grab_pose1);
    collision_object.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(collision_object);
    RCLCPP_INFO(LOGGER, "Attach the object to the robot");
    std::vector<std::string> touch_links1;
    //touch_links.push_back("");
    touch_links1.push_back("wrist_2_link");
    touch_links1.push_back("wrist_3_link");
    move_group.attachObject(collision_object.id, "wrist_2_link", touch_links1);

    visual_tools.publishText(text_pose, "Object_attached_to_robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Wait for MoveGroup to receive and process the attached collision object message */
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = 1.571;
    joint_group_positions[1] = -2.758;
    joint_group_positions[2] = 2.758;
    joint_group_positions[3] = -3.141;
    joint_group_positions[4] = -1.553;
    joint_group_positions[5] = 0; // Modify this for UR robot
    bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
    if (!within_bounds)
    {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but will be clamped.");
    }

    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan (joint-space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    move_group.move();

    const char*  command = "ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command robotiq_2f_urcap_adapter/GripperCommand '{ command: { position: 0.060, max_effort: 140, max_speed: 0.05 }}'";
    int result = system(command);
    if (result == 0) {
        std::cout << "Command executed successfully." << std::endl;
    } else {
        std::cout << "Command execution failed." << std::endl;
    }

    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "wrist_2_link";
    ocm.header.frame_id = "base_link";
    ocm.orientation.w = 1.000;
    ocm.orientation.z = 0.008;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
    moveit_msgs::msg::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);

    //rclcpp::sleep_for(std::chrono::seconds(1)); // Allow time for the subscriber to process

    //Get the target pose after the first message is received
    //rclcpp::sleep_for(std::chrono::seconds(1)); // Allow time for the subscriber to process
    while(true){

        moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
        joint_group_positions[0] = 1.571;
        joint_group_positions[1] = -2.758;
        joint_group_positions[2] = 2.758;
        joint_group_positions[3] = -3.141;
        joint_group_positions[4] = -1.553;
        joint_group_positions[5] = 0; // Modify this for UR robot
        bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
        if (!within_bounds)
        {
        RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but will be clamped.");
        }
    
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);
    
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Visualizing plan (joint-space goal) %s", success ? "" : "FAILED");
    
        visual_tools.deleteAllMarkers();
        visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    
        move_group.move();

        float fruit_width = move_group_reciever->getFruitWidth();
        geometry_msgs::msg::Pose target_pose1 = move_group_reciever->getTargetPose();
        target_pose1.position.y -= 0.10;
        target_pose1.position.z -= 0.010;
        // target_pose1.position.x += 0.010; // minus gives more left
        move_group.setStartStateToCurrentState();
        move_group.setJointValueTarget(target_pose1);
        move_group.setMaxVelocityScalingFactor(0.2);
        move_group.setMaxAccelerationScalingFactor(0.2);

        RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
        success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

        visual_tools.publishAxisLabeled(target_pose1, "pose1");
        visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

        move_group.move();

        //move_group.setPlannerId("RRTConnectkConfigDefault");
        current_state = move_group.getCurrentState(10);
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        geometry_msgs::msg::Pose target_pose2 = target_pose1;
        target_pose2.position.y += 0.065;
        within_bounds = move_group.setJointValueTarget(target_pose2);
        if (!within_bounds)
        {
        RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but will be clamped.");
        }

        //move_group.setStartStateToCurrentState();
        //move_group.setJointValueTarget(target_pose1);
        //move_group.setPlanningTime(1.0);

        move_group.setMaxVelocityScalingFactor(0.05);
        move_group.setMaxAccelerationScalingFactor(0.05);

        RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
        success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

        visual_tools.publishAxisLabeled(target_pose2, "pose1");
        visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

        move_group.move();

        std::ostringstream command_stream;
        command_stream << "ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command "
                    << "robotiq_2f_urcap_adapter/GripperCommand '{ command: { position: " 
                    << fruit_width << ", max_effort: 40, max_speed: 0.05 }}'";

        // Convert to a C-style string
        std::string command_close = command_stream.str();
        result = system(command_close.c_str());

        if (result == 0) {
            std::cout << "Command executed successfully." << std::endl;
        } else {
            std::cout << "Command execution failed." << std::endl;
        }
        // current_state = move_group.getCurrentState(10);
        // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        // geometry_msgs::msg::Pose target_pose3 = target_pose2;
        // target_pose3.position.y -= 0.04;
        // within_bounds = move_group.setJointValueTarget(target_pose3);
        // if (!within_bounds)
        // {
        // RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but will be clamped.");
        // }

        // move_group.setMaxVelocityScalingFactor(0.05);
        // move_group.setMaxAccelerationScalingFactor(0.05);

        // RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
        // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        // RCLCPP_INFO(LOGGER, "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

        // visual_tools.publishAxisLabeled(target_pose3, "pose1");
        // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
        // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        // visual_tools.trigger();
        // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

        // move_group.move();

        current_state = move_group.getCurrentState(10);
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        joint_group_positions[5] = 0; // Modify this for UR robot
        within_bounds = move_group.setJointValueTarget(joint_group_positions);
        if (!within_bounds)
        {
        RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but will be clamped.");
        }

        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);

        success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Visualizing plan (joint-space goal) %s", success ? "" : "FAILED");

        visual_tools.deleteAllMarkers();
        visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

        move_group.move();


        current_state = move_group.getCurrentState(10);
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        joint_group_positions[0] = 1.571;
        joint_group_positions[1] = -2.758;
        joint_group_positions[2] = 2.758;
        joint_group_positions[3] = -3.141;
        joint_group_positions[4] = -1.553;
        joint_group_positions[5] = 0; // Modify this for UR robot
        within_bounds = move_group.setJointValueTarget(joint_group_positions);
        if (!within_bounds)
        {
        RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but will be clamped.");
        }

        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);

        success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Visualizing plan (joint-space goal) %s", success ? "" : "FAILED");

        visual_tools.deleteAllMarkers();
        visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

        move_group.move();

        command = "ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command robotiq_2f_urcap_adapter/GripperCommand '{ command: { position: 0.060, max_effort: 140, max_speed: 0.05 }}'";
        result = system(command);
        if (result == 0) {
            std::cout << "Command executed successfully." << std::endl;
        } else {
            std::cout << "Command execution failed." << std::endl;
        }
    }

    //Shut down the node after demo
    rclcpp::shutdown();
    return 0;
}

//        ***TEST FLUIDITY***    

// geometry_msgs::msg::Pose target_pose1;
//     target_pose1.position.x = -0.400;
//     target_pose1.position.y = 0.546;
//     target_pose1.position.z = 0.564;
//     target_pose1.orientation.x = -0.706;
//     target_pose1.orientation.y = 0.038;
//     target_pose1.orientation.z = -0.012;
//     target_pose1.orientation.w = 0.707;
//     move_group.setPoseTarget(target_pose1);

//     move_group.setMaxVelocityScalingFactor(0.1);
//     move_group.setMaxAccelerationScalingFactor(0.1);
//     // ** Cartesian Path**
//     // std::vector<geometry_msgs::msg::Pose> waypoints;
//     // waypoints.push_back(target_pose1);

//     // geometry_msgs::msg::Pose target_pose3 = target_pose1;

//     // moveit_msgs::msg::RobotTrajectory trajectory;
//     // const double jump_threshold = 0.0;
//     // const double eef_step = 0.05;
//     // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory,true);
//     // RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

//     // visual_tools.deleteAllMarkers();
//     // visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
//     // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//     // for (std::size_t i = 0; i < waypoints.size(); ++i)
//     //     visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
//     // visual_tools.trigger();
//     // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//     // move_group.execute(trajectory); 

//     // Plan and visualize
//     RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
//     success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(LOGGER, "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

//     visual_tools.publishAxisLabeled(target_pose1, "pose1");
//     visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//     visual_tools.trigger();
//     //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//     move_group.move();

//     current_state = move_group.getCurrentState(10);
//     current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

//     joint_group_positions[0] = 1.536;
//     joint_group_positions[1] = -2.356;
//     joint_group_positions[2] = 2.356;
//     joint_group_positions[3] = -3.141;
//     joint_group_positions[4] = -1.606;
//     joint_group_positions[5] = 0; // Modify this for UR robot
//     within_bounds = move_group.setJointValueTarget(joint_group_positions);
//     if (!within_bounds)
//     {
//     RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but will be clamped.");
//     }

//     move_group.setMaxVelocityScalingFactor(0.1);
//     move_group.setMaxAccelerationScalingFactor(0.1);

//     success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(LOGGER, "Visualizing plan (joint-space goal) %s", success ? "" : "FAILED");

//     visual_tools.deleteAllMarkers();
//     visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//     visual_tools.trigger();
//     //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//     move_group.move();

//     geometry_msgs::msg::Pose target_pose2;
//     target_pose2.position.x = 0.400;
//     target_pose2.position.y = 0.546;
//     target_pose2.position.z = 0.564;
//     target_pose2.orientation.x = -0.706;
//     target_pose2.orientation.y = 0.038;
//     target_pose2.orientation.z = -0.012;
//     target_pose2.orientation.w = 0.707;
//     move_group.setPoseTarget(target_pose2);

//     move_group.setMaxVelocityScalingFactor(0.1);
//     move_group.setMaxAccelerationScalingFactor(0.1);

//     RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
//     success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(LOGGER, "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

//     visual_tools.publishAxisLabeled(target_pose2, "pose1");
//     visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//     visual_tools.trigger();
//     //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//     move_group.move();

//     current_state = move_group.getCurrentState(10);
//     current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

//     joint_group_positions[0] = 1.536;
//     joint_group_positions[1] = -2.356;
//     joint_group_positions[2] = 2.356;
//     joint_group_positions[3] = -3.141;
//     joint_group_positions[4] = -1.606;
//     joint_group_positions[5] = 0; // Modify this for UR robot
//     within_bounds = move_group.setJointValueTarget(joint_group_positions);
//     if (!within_bounds)
//     {
//     RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but will be clamped.");
//     }

//     move_group.setMaxVelocityScalingFactor(0.1);
//     move_group.setMaxAccelerationScalingFactor(0.1);

//     success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(LOGGER, "Visualizing plan (joint-space goal) %s", success ? "" : "FAILED");

//     visual_tools.deleteAllMarkers();
//     visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//     visual_tools.trigger();
//     //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//     move_group.move();

//     geometry_msgs::msg::Pose target_pose3;
//     target_pose3.position.x = -0.619;
//     target_pose3.position.y = 0.494;
//     target_pose3.position.z = 0.489;
//     target_pose3.orientation.x = -0.706;
//     target_pose3.orientation.y = 0.038;
//     target_pose3.orientation.z = -0.012;
//     target_pose3.orientation.w = 0.707;
//     move_group.setPoseTarget(target_pose3);

//     move_group.setMaxVelocityScalingFactor(0.1);
//     move_group.setMaxAccelerationScalingFactor(0.1);

//     RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
//     success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(LOGGER, "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

//     visual_tools.publishAxisLabeled(target_pose3, "pose1");
//     visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//     visual_tools.trigger();
//     //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//     move_group.move();

//     current_state = move_group.getCurrentState(10);
//     current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

//     joint_group_positions[0] = 1.536;
//     joint_group_positions[1] = -2.356;
//     joint_group_positions[2] = 2.356;
//     joint_group_positions[3] = -3.141;
//     joint_group_positions[4] = -1.606;
//     joint_group_positions[5] = 0; // Modify this for UR robot
//     within_bounds = move_group.setJointValueTarget(joint_group_positions);
//     if (!within_bounds)
//     {
//     RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but will be clamped.");
//     }

//     move_group.setMaxVelocityScalingFactor(0.1);
//     move_group.setMaxAccelerationScalingFactor(0.1);

//     success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(LOGGER, "Visualizing plan (joint-space goal) %s", success ? "" : "FAILED");

//     visual_tools.deleteAllMarkers();
//     visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//     visual_tools.trigger();
//     //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//     move_group.move();

//     geometry_msgs::msg::Pose target_pose4;
//     target_pose4.position.x = 0.639;
//     target_pose4.position.y = 0.494;
//     target_pose4.position.z = 0.489;
//     target_pose4.orientation.x = -0.706;
//     target_pose4.orientation.y = 0.038;
//     target_pose4.orientation.z = -0.012;
//     target_pose4.orientation.w = 0.707;
//     move_group.setPoseTarget(target_pose4);

//     move_group.setMaxVelocityScalingFactor(0.1);
//     move_group.setMaxAccelerationScalingFactor(0.1);

//     RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
//     success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(LOGGER, "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

//     visual_tools.publishAxisLabeled(target_pose4, "pose1");
//     visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//     visual_tools.trigger();
//     //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//     move_group.move();