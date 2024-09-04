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
    plan_and_execute_pose = rospy.ServiceProxy(SERVICE_NAME, PlanAndExecutePose)

    goal_pose = Pose()
    goal_pose.position.x = 1.0
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.5
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 1.0

    is_relative = False

    try:
        request = plan_end_execute_poseRequest(goal_pose=goal_pose, is_relative=is_relative)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')
        
    response = plan_and_execute_pose(request)
        
    if response.success:
        rospy.loginfo(f'Success, msg content: {response.message}')
    else:
        rospy.logwarn(f'Failed, msg content: {response.message}')
    
    rospy.shutdown
    


if __name__ == '__main__':
    main()