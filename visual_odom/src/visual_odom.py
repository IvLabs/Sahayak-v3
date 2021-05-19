#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np

bridge = CvBridge()
sift = cv2.xfeatures2d.SIFT_create()
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=10)   # or pass empty dictionary
flann = cv2.FlannBasedMatcher(index_params, search_params)

orig_depth = None
orig = None
depth = None
K_inv = None


def camera_info_recieved(msg):
    global K_inv
    K = np.asarray(msg.K).reshape((3, 3))
    K_inv = np.linalg.inv(K)


def depth_recieved_callback(msg):
    global depth
    depth = bridge.imgmsg_to_cv2(msg, "passthrough")


def image_recieved_callback(msg):
    global orig, orig_depth, sift
    new = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow('new', new)
    process(orig, new, orig_depth, depth, K_inv)
    orig = new
    orig_depth = depth


def process(o, n, od, nd, Ki):
    if o is not None and od is not None and Ki is not None:
        kp1, des1 = sift.detectAndCompute(o, None)
        kp2, des2 = sift.detectAndCompute(n, None)
        matches = flann.knnMatch(des1, des2, k=2)
        good = []
        for m, r in matches:
            if m.distance < 0.5*r.distance:
                good.append([m])
        draw_params = dict(matchColor=(0, 255, 0),
                           singlePointColor=(255, 0, 0),
                           flags=0)
        for match in good:
            match = match[0]
            print('In Loop')
            pt1 = kp1[match.queryIdx]
            d1 = od[int(pt1.pt[1]), int(pt1.pt[0])]
            pt2 = kp2[match.trainIdx]
            d2 = nd[int(pt2.pt[1]), int(pt2.pt[0])]
            print(d1, d2)

        img3 = cv2.drawMatchesKnn(
            o, kp1, n, kp2, good, None, **draw_params)
        cv2.imshow('matches', img3)
        cv2.waitKey(1)


def subscribe():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber('/r200/camera/color/image_raw',
                     Image, image_recieved_callback)
    rospy.Subscriber('/r200/camera/color/camera_info',
                     CameraInfo, camera_info_recieved)
    rospy.Subscriber('/r200/camera/depth/image_raw',
                     Image, depth_recieved_callback)
    rospy.spin()


if __name__ == '__main__':
    subscribe()
