#!/usr/bin/env python3

import rospy

from abb_wrapper_msgs.srv import plan_and_execute_pose as PlanAndExecutePose
from abb_wrapper_msgs.srv import plan_and_execute_poseRequest as PlanAndExecutePoseRequest

from abb_wrapper_msgs.srv import plan_and_execute_joint as PlanAndExecuteJoint
from abb_wrapper_msgs.srv import plan_and_execute_jointRequest as PlanAndExecuteJointRequest

from abb_wrapper_msgs.srv import plan_and_execute_slerp as PlanAndExecuteSlerp
from abb_wrapper_msgs.srv import plan_and_execute_slerpRequest as PlanAndExecuteSlerpRequest

from abb_wrapper_msgs.srv import open_gripper as OpenGripper
from abb_wrapper_msgs.srv import open_gripperRequest as OpenGripperRequest

from abb_wrapper_msgs.srv import close_gripper as CloseGripper
from abb_wrapper_msgs.srv import close_gripperRequest as CloseGripperRequest

from geometry_msgs.msg import Pose, PoseStamped

DICE_POSE_TOPIC  = '/dice_pose'
PLAN_AND_EXECUTE_POSE_SERVICE_NAME  = '/plan_and_execute_pose'
PLAN_AND_EXECUTE_JOINT_SERVICE_NAME = '/plan_and_execute_joint'
PLAN_AND_EXECUTE_SLERP_SERVICE_NAME = '/plan_and_execute_slerp'
OPEN_GRIPPER_SERVICE_NAME           = '/open_gripper'
CLOSE_GRIPPER_SERVICE_NAME          = '/close_gripper'

def plan_and_execute_joint(joint_goal):
    rospy.wait_for_service(PLAN_AND_EXECUTE_JOINT_SERVICE_NAME)
    plan_and_execute_joint = rospy.ServiceProxy(PLAN_AND_EXECUTE_JOINT_SERVICE_NAME, PlanAndExecuteJoint)

    srv_request = PlanAndExecuteJointRequest()
    srv_request.joint_goal = joint_goal
    
    try:
        response = plan_and_execute_joint(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')
                
    if response.success:
        rospy.loginfo(f'Success, msg content: {response.message}')
    else:
        rospy.logwarn(f'Failed, msg content: {response.message}')

    return response.success

def plan_and_execute_pose(goal_pose, is_relative):
    rospy.loginfo(f'Waiting {PLAN_AND_EXECUTE_POSE_SERVICE_NAME}')

    # Create a client of the service
    plan_and_execute_pose_client = rospy.ServiceProxy(PLAN_AND_EXECUTE_POSE_SERVICE_NAME, PlanAndExecutePose)

    # Create request
    srv_request = PlanAndExecutePoseRequest()
    srv_request.goal_pose = goal_pose
    srv_request.is_relative = is_relative
    
    try:
        # Call service
        response = plan_and_execute_pose_client(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')
                
    if response.success:
        rospy.loginfo(f'Success, msg content: {response.message}')
    else:
        rospy.logwarn(f'Failed, msg content: {response.message}')    
    
    return response.success


def plan_and_execute_slerp(goal_pose, is_relative):
    rospy.wait_for_service(PLAN_AND_EXECUTE_SLERP_SERVICE_NAME)
    plan_and_execute_slerp = rospy.ServiceProxy(PLAN_AND_EXECUTE_SLERP_SERVICE_NAME, PlanAndExecuteSlerp)

    srv_request = PlanAndExecuteSlerpRequest()
    srv_request.goal_pose = goal_pose
    srv_request.is_relative = is_relative
    
    try:
        response = plan_and_execute_slerp(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')
                
    if response.success:
        rospy.loginfo(f'Success, msg content: {response.message}')
    else:
        rospy.logwarn(f'Failed, msg content: {response.message}')  

    return response.success


def open_gripper():
    rospy.wait_for_service(OPEN_GRIPPER_SERVICE_NAME)
    open_gripper = rospy.ServiceProxy(OPEN_GRIPPER_SERVICE_NAME, OpenGripper)

    srv_request = OpenGripperRequest()
    srv_request.in_flag = True

    try:
        response = open_gripper(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')

def close_gripper():
    rospy.wait_for_service(CLOSE_GRIPPER_SERVICE_NAME)
    close_gripper = rospy.ServiceProxy(CLOSE_GRIPPER_SERVICE_NAME, CloseGripper)

    srv_request = CloseGripperRequest()
    srv_request.in_flag = True

    try:
        response = close_gripper(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')

def wait_for_dice_pose():
    dice_pose = rospy.wait_for_message(DICE_POSE_TOPIC, PoseStamped, timeout=None)
    rospy.loginfo(f'Pose received: {dice_pose}')
    return dice_pose.pose

def main():
    rospy.init_node('take_the_dice_example')
    
    rospy.loginfo('Going to home...')
    goal_joints = []
    goal_joints.append(0.0) # Joint 1
    goal_joints.append(-0.4) # Joint 2
    goal_joints.append(0.6) # Joint 3
    goal_joints.append(0.0) # Joint 4
    goal_joints.append(1.36) # Joint 5
    goal_joints.append(-1.30) # Joint 6
    plan_and_execute_joint(goal_joints)

    rospy.loginfo('Waiting for the dice pose...')
    dice_pose = wait_for_dice_pose()
    
    rospy.loginfo('Approaching the dice...')
    approach_pose = dice_pose
    approach_pose.position.z = approach_pose.position.z+0.3 #30 centimeters above the dice
    plan_and_execute_pose(approach_pose, False)

    rospy.loginfo('Opening gripper...')
    open_gripper()

    rospy.loginfo('Going down on the dice...')
    down_pose = Pose()
    down_pose.position.z = 0.30 
    plan_and_execute_slerp(down_pose, True)

    rospy.loginfo('Closing gripper...')
    close_gripper()

    rospy.loginfo('Going up...')
    up_pose = Pose()
    up_pose.position.z = -0.30 
    plan_and_execute_slerp(up_pose, True)

    rospy.sleep(2)

    rospy.loginfo('Releasing the dice...')
    release_pose = Pose()
    release_pose.position.x = 0.05 
    release_pose.position.y = 0.05 
    release_pose.position.z = 0.30
    plan_and_execute_slerp(release_pose, True)

    rospy.loginfo('Opening gripper...')
    open_gripper()

    rospy.loginfo('Going up...')
    up_pose = Pose()
    up_pose.position.z = -0.30 
    plan_and_execute_slerp(up_pose, True)

if __name__ == '__main__':
    main()