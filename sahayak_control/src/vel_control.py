#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
PI = np.pi

class sahayak:
    def __init__(self):
        #super().__init__()
        rospy.init_node('sahayak_joint_controller', anonymous=True)

        rospy.Subscriber("sahayak/joint_states", JointState, self.torque_callback)

        self.wheel_mode_time = 3.25 # secs

        self.joint_pub = {}

        for joint_indx in range(1, 5):
            
            self.joint_pub.update({'joint{}'.format(joint_indx): 
                    rospy.Publisher('/sahayak/joint{}_vel_controller/command'.format(joint_indx), 
                                    Float64, queue_size=10)})

        self.rate = rospy.Rate(100)

    def torque_callback(self, data):
        # rospy.loginfo("\r {}".format(data))
        self.data = data

    def move(self):
        self.start = rospy.get_rostime()
        
        while not rospy.is_shutdown():
            self.now = rospy.get_rostime()

            # for joint_indx in range(1, 5):                                
            self.joint_pub['joint1'].publish(-30.0*2*PI/60) # RPM
            self.joint_pub['joint2'].publish(30.0*2*PI/60) # RPM
            self.joint_pub['joint3'].publish(-30.0*2*PI/60) # RPM
            self.joint_pub['joint4'].publish(30.0*2*PI/60) # RPM
            
            self.rate.sleep()            

            # # spin() simply keeps python from exiting until this node is stopped
            # rospy.spin()

if __name__ == '__main__':
    bot = sahayak()
    bot.move()
