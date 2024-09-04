#!/usr/bin/env python3

import rospy
from abb_wrapper_msgs.srv import plan_end_execute_pose, plan_end_execute_poseRequest
from geometry_msgs.msg import Pose

SERVICE_NAME = 'plan_end_execute_pose'

def main():
    rospy.init_node('plan_and_execute_pose_client')

    position_x = rospy.get_param("~position_x")
    position_y = rospy.get_param("~position_y")
    position_z = rospy.get_param("~position_z")

    rospy.wait_for_service(SERVICE_NAME)
    plan_and_execute_pose = rospy.ServiceProxy(SERVICE_NAME, plan_end_execute_pose)

    goal_pose = Pose()
    goal_pose.position.x = position_x
    goal_pose.position.y = position_y
    goal_pose.position.z = position_z
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 1.0

    is_relative = False

    srv_request = plan_end_execute_poseRequest()
    srv_request.goal_pose = goal_pose
    srv_request.is_relative = is_relative
    
    try:
        response = plan_and_execute_pose(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')
        
    if response.success:
        rospy.loginfo(f'Success, msg content: {response.message}')
    else:
        rospy.logerr(f'Failed, msg content: {response.message}')
        


if __name__ == '__main__':
    main()