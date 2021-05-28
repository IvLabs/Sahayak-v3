import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np

bridge = CvBridge()
sift = cv2.xfeatures2d.SIFT_create()
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=10)
flann = cv2.FlannBasedMatcher(index_params, search_params)
draw_params = dict(matchColor=(0, 255, 0),
                   singlePointColor=(255, 0, 0),
                   flags=0)


class OdomState:
    def __init__(self):
        self.processing = False
        self.rgb = None
        self.depth = None
        self.K = None

    def done(self):
        return self.rgb is not None and self.depth is not None

    def add_rgb(self, rgb):
        self.rgb = rgb

    def add_depth(self, depth):
        self.depth = depth

    def add_K(self, K):
        self.K = K

    def find_odom(self, prevOdom):
        self.processing = True
        print('FindingOdom')
        # cv2.imshow('prev', prevOdom.rgb)
        # cv2.imshow('this', self.rgb)
        kp1, des1 = sift.detectAndCompute(prevOdom.rgb, None)
        kp2, des2 = sift.detectAndCompute(self.rgb, None)
        matches = flann.knnMatch(des1, des2, k=2)
        print('Matches Found!')
        img3 = cv2.drawMatchesKnn(
            prevOdom.rgb, kp1, self.rgb, kp2, matches, None, **draw_params)
        # cv2.imshow('matches', img3)
        cv2.waitKey(1000)
        print('Function End!')
        self.processing = False


prevState = None
thisState = OdomState()


def on_image_recieve(msg):
    global thisState, prevState
    if not thisState.processing:
        thisState.add_rgb(bridge.imgmsg_to_cv2(msg, "bgr8"))
        if thisState.done():
            if prevState is not None:
                thisState.find_odom(prevState)
            prevState = thisState
            thisState = OdomState()


def on_depth_recieve(msg):
    global thisState, prevState
    if not thisState.processing:
        thisState.add_depth(bridge.imgmsg_to_cv2(msg, "passthrough"))
        if thisState.done():
            if prevState is not None:
                thisState.find_odom(prevState)
            prevState = thisState
            thisState = OdomState()


def on_K_recieve(msg):
    global thisState, prevState
    if not thisState.processing:
        thisState.add_depth(np.asarray(msg.K).reshape((3, 3)))
        if thisState.done():
            if prevState is not None:
                thisState.find_odom(prevState)
            prevState = thisState
            thisState = OdomState()


def subscribe():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber('/r200/camera/color/image_raw',
                     Image, on_image_recieve)
    rospy.Subscriber('/r200/camera/color/camera_info',
                     CameraInfo, on_K_recieve)
    rospy.Subscriber('/r200/camera/depth/image_raw',
                     Image, on_depth_recieve)
    rospy.spin()


if __name__ == '__main__':
    subscribe()
