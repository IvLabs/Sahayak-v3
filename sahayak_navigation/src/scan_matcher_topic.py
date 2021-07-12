#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
from tf.transformations import quaternion_from_euler


rospy.init_node('OdomPublisher', anonymous=True)
odom_pub = rospy.Publisher('/laser/odom',Odometry, queue_size=10)

def pose_cb(data):
    odom = Odometry()
    odom_quat = quaternion_from_euler(0,0,(data.theta))
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(data.x, data.y,0), Quaternion(odom_quat[0],odom_quat[1],odom_quat[2], odom_quat[3]))
    odom.child_frame_id = "base_footprint"
    odom_pub.publish(odom)

    odom_br = tf.TransformBroadcaster()
    odom_br.sendTransform((data.x,data.y, 0),odom_quat,rospy.Time.now(),"base_footprint","odom")
    
    print(data.x,data.y, data.theta)

def main():
    sub = rospy.Subscriber('/pose2D',Pose2D,pose_cb)
    rospy.spin()

if __name__ == '__main__':
    main()
    
    

