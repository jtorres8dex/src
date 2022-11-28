#include "../include/circle.hpp"

#include <math.h>
#include <cmath>

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
     
    float r = 0.5;

    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = 1.01229;
    joint_targets["shoulder_lift_joint"] = -0.366519;
    joint_targets["shoulder_pan_joint"] = -3.52557;
    joint_targets["wrist_1_joint"] = -2.21657;
    joint_targets["wrist_2_joint"] = 1.5708;
    joint_targets["wrist_3_joint"] = 4.97419;

    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }
    geometry_msgs::Pose waypoint_1;
    waypoint_1.position.x = -1*r;
    waypoint_1.position.y = 0.0;
    waypoint_1.position.z = 1.0;
    waypoint_1.orientation.x = 0.0;
    waypoint_1.orientation.y = 0.0;
    waypoint_1.orientation.z = 1;
    waypoint_1.orientation.w = 0;
    
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(waypoint_1);

    geometry_msgs::Pose waypoint_n;

    
    float num_points = 100;
    float theta = 0;
    float step = 2*M_PI/num_points;
    float x;
    float y;
    
    for (int i = 0; i < num_points; i++){
        x = -1*r*cos(theta);
        y = -r*sin(theta);
        theta = theta + step;

        waypoint_n.position.x = x;
        waypoint_n.position.y = y;
        waypoint_n.position.z = 1.0;
    
       
    
        waypoints.push_back(waypoint_n);
    }

    waypoints.push_back(waypoint_1);
   
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;
    bool pose_plan_success;
    std::string reference_frame = "base_link";
    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, waypoint_1, reference_frame, pose_plan);
        
      moveit_msgs::RobotTrajectory trajectory;
      trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

      n.setParam("/record_pose", true);
      arm_move_group.execute(trajectory);
      n.setParam("/record_pose", false);
      
}