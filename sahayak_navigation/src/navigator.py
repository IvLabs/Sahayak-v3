#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np

PI = np.pi

rospy.init_node('OdomPublisher', anonymous=True)
joint1 = rospy.Publisher('joint1_vel_controller/command', Float64, queue_size=1)
joint2 = rospy.Publisher('joint2_vel_controller/command', Float64, queue_size=1)
joint3 = rospy.Publisher('joint3_vel_controller/command', Float64, queue_size=1)
joint4 = rospy.Publisher('joint4_vel_controller/command', Float64, queue_size=1)


# from ground truth velocity at 30rpm = 0.25m/s

max_y_vel = 0.4 #meter/sec

lin_factor = 30.0*2*PI/(60*0.25)
lin_factor *= max_y_vel

# from ground truth angular velocity at 15rpm = 0.33 rad/s
max_theta_ang = 1 
ang_factor = 15.0*2*PI/(60*0.20)
ang_factor *= max_theta_ang

def navigate_cb(data):
    
    if data.angular.z > 0.02:
        joint1.publish(0.6*data.angular.z*ang_factor/max_theta_ang)
        joint2.publish(0.6*data.angular.z*ang_factor/max_theta_ang)
        joint3.publish(0.6*data.angular.z*ang_factor/max_theta_ang)
        joint4.publish(0.6*data.angular.z*ang_factor/max_theta_ang)
        print('Going Left')
        #rospy.sleep(0.12)

    elif data.angular.z < -0.02:
        joint1.publish(0.6*data.angular.z*ang_factor/max_theta_ang)
        joint2.publish(0.6*data.angular.z*ang_factor/max_theta_ang)
        joint3.publish(0.6*data.angular.z*ang_factor/max_theta_ang)
        joint4.publish(0.6*data.angular.z*ang_factor/max_theta_ang)
        print('Going Right')
        #rospy.sleep(0.12)

    elif data.angular.z == 0:
        if data.linear.y > 0.0:
            joint1.publish(0.8*data.linear.y*lin_factor/max_y_vel)
            joint2.publish(-0.8*data.linear.y*lin_factor/max_y_vel)
            joint3.publish(0.8*data.linear.y*lin_factor/max_y_vel)
            joint4.publish(-0.8*data.linear.y*lin_factor/max_y_vel)
            print('Going Forward')
            #rospy.sleep(0.12)

        elif data.linear.y < 0.0:
            joint1.publish(0.8*data.linear.y*lin_factor/max_y_vel)
            joint2.publish(-0.8*data.linear.y*lin_factor/max_y_vel)
            joint3.publish(0.8*data.linear.y*lin_factor/max_y_vel)
            joint4.publish(-0.8*data.linear.y*lin_factor/max_y_vel)
            print('Going Backward')
            #rospy.sleep(0.12)

        else:
            joint1.publish(0)
            joint2.publish(0)
            joint3.publish(0)
            joint4.publish(0)
            print('Stopped')
            #rospy.sleep(0.12)

    else:
        joint1.publish(0)
        joint2.publish(0)
        joint3.publish(0)
        joint4.publish(0)
        print('Stopped')
        #rospy.sleep(0.12)
    
    if data.linear.y == 0 and data.angular.z == 0:
        print('Destination Reached !!')





def main():
    sub = rospy.Subscriber('/cmd_vel',Twist,navigate_cb)
    rospy.spin()

if __name__ == '__main__':
    main()
    
    

