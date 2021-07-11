#!/usr/bin/env python

#Teleoperation
#Left arrow key - robot moves left
#right arrow key - robot moves right
#up arrow key - robot moves forward
#down arrow key - robot moves back
#Enter - robot stops
#make sure you install the pynput library

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from pynput import keyboard
from pynput.keyboard import Listener, Key

PI = np.pi


class sahayak:
    def __init__(self):

        rospy.init_node('sahayak_joint_controller', anonymous=True)

        rospy.Subscriber("joint_states", JointState, self.torque_callback)

        self.wheel_mode_time = 3.25 # secs

        self.joint_pub = {}

        for joint_indx in range(1, 5):
            
            self.joint_pub.update({'joint{}'.format(joint_indx): 
                    rospy.Publisher('joint{}_vel_controller/command'.format(joint_indx), 
                                    Float64, queue_size=10)})

        self.rate = rospy.Rate(100)

    def torque_callback(self, data):

        self.data = data

    def move(self):
        self.start = rospy.get_rostime()
        
        while not rospy.is_shutdown():
            with keyboard.Listener(on_press=bot.on_press,on_release=on_release) as listener:
                listener.join()
       
    def on_press(self,key):
        try:
            if key == Key.up:
                print('Moving Forward')
                self.now = rospy.get_rostime()

                self.joint_pub['joint1'].publish(30.0*2*PI/60) # RPM
                self.joint_pub['joint2'].publish(-30.0*2*PI/60) # RPM
                self.joint_pub['joint3'].publish(30.0*2*PI/60) # RPM
                self.joint_pub['joint4'].publish(-30.0*2*PI/60) # RPM
                            
                
                self.rate.sleep()
            elif key== Key.left:
                print('Moving left')
                
                self.now = rospy.get_rostime()

                self.joint_pub['joint1'].publish(15.0*2*PI/60) # RPM
                self.joint_pub['joint2'].publish(15.0*2*PI/60) # RPM
                self.joint_pub['joint3'].publish(15.0*2*PI/60) # RPM
                self.joint_pub['joint4'].publish(15.0*2*PI/60) # RPM
                              
                self.rate.sleep()    

            elif key==Key.right:
                print('Moving right')
                
                self.now = rospy.get_rostime()
                              
                self.joint_pub['joint1'].publish(-15.0*2*PI/60) # RPM
                self.joint_pub['joint2'].publish(-15.0*2*PI/60) # RPM
                self.joint_pub['joint3'].publish(-15.0*2*PI/60) # RPM
                self.joint_pub['joint4'].publish(-15.0*2*PI/60) # RPM
                
                self.rate.sleep()              
            elif key==Key.down:
                print('Moving back')
                self.now = rospy.get_rostime()

                self.joint_pub['joint1'].publish(-30.0*2*PI/60) # RPM
                self.joint_pub['joint2'].publish(30.0*2*PI/60) # RPM
                self.joint_pub['joint3'].publish(-30.0*2*PI/60) # RPM
                self.joint_pub['joint4'].publish(30.0*2*PI/60) # RPM
                
                self.rate.sleep()
            elif key==Key.enter:
                print('Stop')
                self.now = rospy.get_rostime()
                              
                self.joint_pub['joint1'].publish(0.0) # RPM
                self.joint_pub['joint2'].publish(0.0) # RPM
                self.joint_pub['joint3'].publish(0.0) # RPM
                self.joint_pub['joint4'].publish(0.0) # RPM
                
                self.rate.sleep()

            elif key==Key.esc:
                rospy.signal_shutdown("Shutting down")
            else:
                print('Invalid key, robot retains previous command')
        except AttributeError:
            print('Invalid Key, robot retains previous command')

def on_release(key):
    print(key) 
    if key == keyboard.Key.esc:
        return False


           
if __name__ == '__main__':
    bot = sahayak()
    bot.move()
