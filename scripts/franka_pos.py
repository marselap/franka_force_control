#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseArray, Point, Pose
from franka_msgs.msg import FrankaState

import numpy as np
import math


class FrankaPosListener(object):
  def __init__(self):

    rospy.init_node('franka_pos_node',
                    anonymous=True)

    subs2 = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.franka_state_callback)

    self.pose_pub = rospy.Publisher('/pose', Float64MultiArray, queue_size=1)

    self.position = None


  def franka_state_callback(self, data):
    self.position = data.O_T_EE


def main():

    listener = FrankaPosListener()

    r = rospy.Rate(1000)

    try:
        while not rospy.is_shutdown():

            if listener.position:
                msg = Float64MultiArray()
                msg.data = list(listener.position)
                listener.pose_pub.publish(msg)
                listener.position = None
            else:
                r.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()

