#!/usr/bin/env python
import rospy 
from nav_msgs.msg import Odometry
import tf


rospy.init_node('OdomPublisher', anonymous=True)

def odom_cb(data):
    odom_br = tf.TransformBroadcaster()
    odom_p = data.pose.pose.position
    odom_quat = data.pose.pose.orientation

    odom_br.sendTransform((odom_p.x,odom_p.y, odom_p.z),(odom_quat.x,odom_quat.y,odom_quat.z,odom_quat.w),rospy.Time.now(),"base_footprint","odom")
    print(odom_p.x,odom_p.y, odom_p.z)

def main():
    pub = rospy.Subscriber('/ground_truth/state',Odometry,odom_cb)
    rospy.spin()

if __name__ == '__main__':
    main()
    
    

