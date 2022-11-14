#include "../include/circle.hpp"

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "circle");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;
    
    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
    MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    //Write your code for following the circle trajectory here.
     
    geometry_msgs::Pose waypoint_1;
    waypoint_1.position.x = 0.25;
    waypoint_1.position.y = 0.2;
    waypoint_1.position.z = 1.1;
    waypoint_1.orientation.x = -0.707;
    waypoint_1.orientation.y = 0.0;
    waypoint_1.orientation.z = 0.0;
    waypoint_1.orientation.w = 0.707;


    geometry_msgs::Pose waypoint_n;
    float r = 0.1;
    for(int dx=0; dx < 100; dx++){
        float casted_dx = (float) dx;
        waypoint_n.position.x = 0.25 + casted_dx;
        waypoint_n.position.y = 0.2;
        waypoint_n.position.z = 1.1;
        waypoint_n.orientation.x = -0.707;
        waypoint_n.orientation.y = 0.0;
        waypoint_n.orientation.z = 0.0;
        waypoint_n.orientation.w = 0.707;
    }
 




    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;
    bool pose_plan_success;
    std::string reference_frame = "base_link";
    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, waypoint_1, reference_frame, pose_plan);
       
}