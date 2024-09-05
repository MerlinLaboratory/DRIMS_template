#include "ros/ros.h"
#include <iostream>
#include "ros/service_client.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int16.h"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>

// ROS Service srv definition

#include "abb_wrapper_msgs/plan_and_execute_pose.h"
#include "abb_wrapper_msgs/open_gripper.h"
#include "abb_wrapper_msgs/close_gripper.h"
#include "abb_wrapper_msgs/plan_and_execute_joint.h"
#include "abb_wrapper_msgs/plan_and_execute_slerp.h"

// Declare ROS clients
ros::ServiceClient plan_and_execute_pose_client;
ros::ServiceClient plan_and_execute_joint_client;
ros::ServiceClient plan_and_execute_slerp_client;
ros::ServiceClient close_gripper_client;
ros::ServiceClient open_gripper_client;

geometry_msgs::Pose grasp_dice_pose;
std_msgs::Int16 dice_value;
int desired_dice_value;

//
ros::Subscriber dice_value_sub;
ros::Subscriber dice_grasp_sub;

//
int counter = 0;
bool gripper = true;
bool success = false;
bool exit_flag = false;
std::vector<double> initial_joint{0.6958229344666362, -0.832986721504323, 0.7529323343354806, 1.1712760581288713, 0.3368141414588006, 1.0205815676983665, 1.1097106282881333};

//
void dicevalueCallback(const std_msgs::Int16::ConstPtr &msg)
{
   dice_value.data = msg->data;
   // ROS_INFO("The dice_value is: %d", dice_value.data);
};

void dicePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
   grasp_dice_pose.position.x = msg->pose.position.x;
   grasp_dice_pose.position.y = msg->pose.position.y;
   grasp_dice_pose.position.z = msg->pose.position.z;

   grasp_dice_pose.orientation.x = msg->pose.orientation.x;
   grasp_dice_pose.orientation.y = msg->pose.orientation.y;
   grasp_dice_pose.orientation.z = msg->pose.orientation.z;
   grasp_dice_pose.orientation.w = msg->pose.orientation.w;
};

// Function to apply rotation offset
geometry_msgs::Pose applyRotationOffset(const geometry_msgs::Pose &pose, double angle_degrees, const Eigen::Vector3d &axis)
{
   double angle_radians = angle_degrees * M_PI / 180.0;
   Eigen::AngleAxisd rotation(angle_radians, axis); //   axis = Eigen::Vector3d::UnitX() or Eigen::Vector3d::UnitY() or Eigen::Vector3d::UnitZ()
   Eigen::Affine3d offset_eigen = Eigen::Affine3d::Identity();
   offset_eigen.rotate(rotation);

   // Convert the geometry_msgs::Pose into Eigen
   Eigen::Affine3d pose_transform_aff;
   tf::poseMsgToEigen(pose, pose_transform_aff);

   // Post multiplied the input pose for the RotationOffset
   Eigen::Affine3d offset_pose = pose_transform_aff * offset_eigen;

   // Convert the Eigen into geometry_msgs::Pose
   geometry_msgs::Pose offset_pose_msg;
   tf::poseEigenToMsg(offset_pose, offset_pose_msg);

   return offset_pose_msg;
}

geometry_msgs::Pose applyDisplacementOffset(const geometry_msgs::Pose &pose, const Eigen::Vector3d &displacement)
{
   // Create an Eigen Affine3d transformation for the displacement
   Eigen::Affine3d offset_eigen = Eigen::Affine3d::Identity();
   offset_eigen.translate(displacement);

   // Convert the geometry_msgs::Pose into Eigen
   Eigen::Affine3d pose_transform_aff;
   tf::poseMsgToEigen(pose, pose_transform_aff);

   // Post multiply the input pose for the displacement offset
   Eigen::Affine3d offset_pose = pose_transform_aff * offset_eigen;

   // Convert the Eigen into geometry_msgs::Pose
   geometry_msgs::Pose offset_pose_msg;
   tf::poseEigenToMsg(offset_pose, offset_pose_msg);

   return offset_pose_msg;
}

/**********************************************
ROS NODE MAIN TASK SEQUENCE SERVER
**********************************************/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "test_example");

   ros::NodeHandle nh_;

   // Parse the desired dice value from YAML
   if (!ros::param::get("/dice/dice_top_desired_value", desired_dice_value))
   {
      ROS_WARN("The param 'dice_top_desired_value' not found in param server! ");
   }

   ROS_INFO("The desired dice value is: %d", desired_dice_value);

   // Create subscribers for dice_value and grasp_dice_pose
   dice_value_sub = nh_.subscribe("/dice_value", 1, dicevalueCallback);
   dice_grasp_sub = nh_.subscribe("/dice_pose", 1, dicePoseCallback);

   // Wait for messages on the CallBacks
   ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/dice_pose", ros::Duration(5.0));
   ros::topic::waitForMessage<std_msgs::Int16>("/dice_value", ros::Duration(5.0));

   // Define ROS clients for calling the 5 ROS Services: PlanAndExcutePose, PlanAndExecuteJoint, PlanAndExecuteSlerp, Open and Close Gripper
   if (!ros::service::waitForService("/plan_and_execute_pose", ros::Duration(2.0)))
        return 0;
   plan_and_execute_pose_client = nh_.serviceClient<abb_wrapper_msgs::plan_and_execute_pose>("/plan_and_execute_pose");
   
   // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
   ros::AsyncSpinner spinner(2);
   spinner.start();

   ROS_INFO("At the beginning of the task");
   
   /* Create a simple grasp task for grasping the dice*/
   

   ros::waitForShutdown();
   spinner.stop();

   return 0;
}