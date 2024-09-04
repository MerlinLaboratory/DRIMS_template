#! /usr/bin/env python3

import rospy
from abb_wrapper_msgs.srv import plan_and_execute_pose as PlanAndExecutePose
from geometry_msgs.msg import Pose, Position, Orientation

SERVICE_NAME = "/plan_and_execute_pose"

# def move_to_pose(x, y):
    try:
        resp1 = move_to_pose_srv(x)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    return "%s [x y]" % sys.argv[0]


if __name__ == "__main__":
    rospy.wait_for_service(SERVICE_NAME)
    move_to_pose_srv = rospy.ServiceProxy(SERVICE_NAME, PlanAndExecutePose)

    goal_pose = Pose()
    goal_pose.


    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s" % (x, y))
    print("%s + %s = %s" % (x, y, add_two_ints_client(x, y)))
