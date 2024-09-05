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

from geometry_msgs.msg import Pose


PLAN_AND_EXECUTE_POSE_SERVICE_NAME  = 'plan_end_execute_pose'
PLAN_AND_EXECUTE_JOINT_SERVICE_NAME = 'plan_end_execute_joint'
PLAN_AND_EXECUTE_SLERP_SERVICE_NAME = 'plan_end_execute_slerp'
OPEN_GRIPPER_SERVICE_NAME           = 'open_gripper'
CLOSE_GRIPPER_SERVICE_NAME          = 'close_gripper'


def plan_and_execute_pose_example():
    # Wait for the service to be available
    rospy.wait_for_service(PLAN_AND_EXECUTE_POSE_SERVICE_NAME)

    # Create a client of the service
    plan_and_execute_pose_client = rospy.ServiceProxy(PLAN_AND_EXECUTE_POSE_SERVICE_NAME, PlanAndExecutePose)

    # Define goal pose message
    goal_pose = Pose()
    goal_pose.position.x = 1.0
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.5
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 1.0

    # Is motion relative w.r.t the actual position?
    is_relative = False

    # Create request
    srv_request = PlanAndExecutePoseRequest()
    srv_request.goal_pose = goal_pose
    srv_request.is_relative = is_relative
    
    try:
        # Call service
        response = plan_and_execute_pose(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')
                
    if response.success:
        rospy.loginfo(f'Success, msg content: {response.message}')
    else:
        rospy.logwarn(f'Failed, msg content: {response.message}')    


def plan_and_execute_joint_example():
    rospy.wait_for_service(PLAN_AND_EXECUTE_JOINT_SERVICE_NAME)
    plan_and_execute_joint = rospy.ServiceProxy(PLAN_AND_EXECUTE_JOINT_SERVICE_NAME, PlanAndExecuteJoint)

    goal_joints = []
    goal_joints.append(0.0) # Joint 1
    goal_joints.append(0.0) # Joint 2
    goal_joints.append(0.0) # Joint 3
    goal_joints.append(0.0) # Joint 4
    goal_joints.append(0.0) # Joint 5
    goal_joints.append(0.0) # Joint 6

    srv_request = PlanAndExecuteJointRequest()
    srv_request.joint_goal = goal_joints
    
    try:
        response = plan_and_execute_joint(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')
                
    if response.success:
        rospy.loginfo(f'Success, msg content: {response.message}')
    else:
        rospy.logwarn(f'Failed, msg content: {response.message}')


def plan_and_execute_slerp_example():
    rospy.wait_for_service(PLAN_AND_EXECUTE_SLERP_SERVICE_NAME)
    plan_and_execute_slerp = rospy.ServiceProxy(PLAN_AND_EXECUTE_SLERP_SERVICE_NAME, PlanAndExecuteSlerp)

    goal_pose = Pose()
    goal_pose.position.x = 1.0
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.5
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 1.0

    is_relative = False

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


def open_gripper_example():
    rospy.wait_for_service(OPEN_GRIPPER_SERVICE_NAME)
    open_gripper = rospy.ServiceProxy(OPEN_GRIPPER_SERVICE_NAME, OpenGripper)

    srv_request = OpenGripperRequest()
    srv_request.in_flag = true

    try:
        response = open_gripper(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')
                
    if response.success:
        rospy.loginfo(f'Success, msg content: {response.message}')
    else:
        rospy.logwarn(f'Failed, msg content: {response.message}')  


def close_gripper_example():
    rospy.wait_for_service(CLOSE_GRIPPER_SERVICE_NAME)
    close_gripper = rospy.ServiceProxy(CLOSE_GRIPPER_SERVICE_NAME, CloseGripper)

    srv_request = CloseGripperRequest()
    srv_request.in_flag = true

    try:
        response = close_gripper(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')
                
    if response.success:
        rospy.loginfo(f'Success, msg content: {response.message}')
    else:
        rospy.logwarn(f'Failed, msg content: {response.message}')  


def main():
    rospy.init_node('services_examples')

    plan_and_execute_pose_example()
    rospy.sleep(1)

    plan_and_execute_joint_example()
    rospy.sleep(1)

    plan_and_execute_slerp_example()
    rospy.sleep(1)

    open_gripper_example()
    rospy.sleep(1)

    close_gripper_example()
    rospy.sleep(1)


if __name__ == '__main__':
    main()