#!/usr/bin/env python
import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Twist, Vector3


def callback(msg):
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	z = msg.pose.pose.position.z
	x1 = msg.pose.pose.orientation.x
	y1 = msg.pose.pose.orientation.y
	z1 = msg.pose.pose.orientation.z
	w1 = msg.pose.pose.orientation.w
	pub = rospy.Publisher('/gt', Pose, queue_size=10)
	gt = Pose()
	gt.position.x = x
	gt.position.y = y
	gt.position.z = z
	gt.orientation.x = x1
	gt.orientation.y = y1
	gt.orientation.z = z1
	gt.orientation.w = w1
	#print(":)")
	pub.publish(gt)

def listener():
   rospy.init_node('ground_truth', anonymous=True)
   rospy.Subscriber("/ground_truth/state", Odometry, callback)
   rospy.spin()
 
if __name__ == '__main__':
	listener()

