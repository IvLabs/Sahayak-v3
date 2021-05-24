#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2 , CameraInfo
from sensor_msgs import point_cloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as Rt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import rowan
from tf.transformations import quaternion_from_matrix
#from rowan import from_matrix

plt.ion()
bridge = CvBridge()
fast = cv2.FastFeatureDetector_create()
fast.setNonmaxSuppression(1)
# Parameters for good featurestotrack kanade optical flow
feature_params = dict(maxCorners=400, qualityLevel=0.2, minDistance=5, blockSize=7)

#init ros node and taking first img ,pointcloud and camera parameters
rospy.init_node("imgread", anonymous=True)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

prev_rgb = rospy.wait_for_message("/r200/mycamera/color/color_rect", Image)
prev_rgb = bridge.imgmsg_to_cv2(prev_rgb, "bgr8")

prev_cloud = rospy.wait_for_message("/camera/depth/points", PointCloud2)
prev_cloud = cloud = np.asarray(
    list(
        point_cloud2.read_points(
            prev_cloud, field_names=("x", "y", "z"), skip_nans=False
        )
    )
).reshape(prev_rgb.shape)

camera_info_msg = rospy.wait_for_message("/r200/camera/color/camera_info", CameraInfo)
K = np.asarray(camera_info_msg.K).reshape((3, 3))

#finding kps of first frame
prev_gray = cv2.cvtColor(prev_rgb, cv2.COLOR_BGR2GRAY)
# create a CLAHE object (Arguments are optional).
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
prev_gray = clahe.apply(prev_gray)
#p0 = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)
#print(p0,p0.shape)
test = fast.detect(prev_gray,None)
test = cv2.KeyPoint_convert(test)
p0 = test.reshape((-1,1,2))

#print(p0,p0.shape)
#print(test.shape)

def get_features_xyz(prev_kp, kp, prev_cloud, cloud):
    xyz1 = []
    xyz2 = []
    for old, new in zip(prev_kp, kp):
        a, b = new.ravel()
        c, d = old.ravel()
        if (
            0 <= b < cloud.shape[0]
            and 0 <= a < cloud.shape[0]
            and 0 <= c < cloud.shape[1]
            and 0 <= d < cloud.shape[1]
        ):
            pt1 = np.asarray(prev_cloud[int(b), int(a)])
            pt2 = np.asarray(cloud[int(d), int(c)])
            if not np.isnan(pt1).any() and not np.isnan(pt2).any():
                xyz1.append(pt1)
                xyz2.append(pt2)
    return np.asarray(xyz1), np.asarray(xyz2)

'''def rmat_to_quaternion(rmat):
    #r = Rt.from_matrix(rmat)
    q = rmat.as_quat()
    return q'''

def ekf_info(msg):
    filtered_tran = np.zeros((3,1))
    filtered_tran[0][0] = msg.pose.pose.position.x
    filtered_tran[1][0] = msg.pose.pose.position.y
    filtered_tran[2][0] = msg.pose.pose.position.z
    x1 = msg.pose.pose.orientation.x
    y1 = msg.pose.pose.orientation.y
    z1 = msg.pose.pose.orientation.z
    w1 = msg.pose.pose.orientation.w
    #q = Rt.from_quat([x1,y1,z1,w1])
    #rot = q.as_matrix()
    return filtered_tran

def odom_publisher(rotations,trajectory,trust):
    current_time = rospy.Time.now()
    odom_quat = rowan.from_matrix(rotations[-1])

    x = trajectory[-1][0][0]
    y = trajectory[-1][1][0]
    z = trajectory[-1][2][0]

    max_covariance = 100000
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "rs200_camera"
    odom.pose.pose = Pose(Point(x, y,z), Quaternion(*odom_quat))
    odom.pose.covariance = ((1/trust)*np.eye(6)).flatten()

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    odom.twist.covariance = (max_covariance*np.eye(6)).flatten() 

    # publish the message
    odom_pub.publish(odom)

def optical_flow_matches(gray,prev_gray,p0):
    # Parameters for lucas kanade optical flow
    lk_params = dict(
    winSize=(5, 5),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
    )

    p1, st, err = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p0, None, **lk_params)

    p0, st, err = cv2.calcOpticalFlowPyrLK(gray, prev_gray, p1, None, **lk_params)
    good_new = p1[st == 1]
    good_old = p0[st == 1]
    return good_new,good_old

def get_features_xyz_xy(prev_kp, kp, prev_cloud, cloud):
    xyz1 = []
    xy2 = []
    #print(prev_cloud.shape,cloud.shape)
    for old, new in zip(prev_kp, kp):
        a, b = old.ravel()
        c, d = new.ravel()
        if (
            0 <= b < cloud.shape[0]
            and 0 <= a < cloud.shape[0]
            and 0 <= c < cloud.shape[1]
            and 0 <= d < cloud.shape[1]
        ):
            pt1 = np.asarray(prev_cloud[int(b), int(a)])
            pt2 = np.asarray([int(c), int(d)])
            if not np.isnan(pt1).any():
                xyz1.append(pt1)
                xy2.append(pt2)
    return np.asarray(xyz1).reshape((-1,3,1)).astype(np.float32), np.asarray(xy2).reshape((-1,2,1)).astype(np.float32)

gt_msg = rospy.wait_for_message("/gt", Pose)
temp = np.zeros((3,1))
temp[0][0] = gt_msg.position.x
temp[1][0] = gt_msg.position.y
temp[2][0] = gt_msg.position.z
#g_truth.append(temp)
#z_motion = []
#y_rotations = []
NUM_CALLED = 0
trajectory = []
robot_pose = []
robot_pose.append(np.eye(4))
motion = []
rotations = []
NUM_CALLED = 0
trajectory = []
rotations.append(np.eye(3))
g_truth = []
g_truth.append(temp)
trajectory.append(temp)

def on_recieve(rgb, cloud ,g_t):
    global prev_rgb, prev_cloud, p0, prev_gray, NUM_CALLED, mask
    temp = np.zeros((3,1))
    temp[0][0] = g_t.position.x
    temp[1][0] = g_t.position.y
    temp[2][0] = g_t.position.z
    print("Started ", NUM_CALLED)
    g_truth.append(temp)
    #print(g_t)
    rgb = bridge.imgmsg_to_cv2(rgb, "bgr8")
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    good_new, good_old = optical_flow_matches(gray,prev_gray,p0)
    features_detected = good_new.shape[0] 
    if(features_detected is None or features_detected == 0):
        features_detected = 0.0001

        odom_publisher(rotations,trajectory,features_detected)
    else:
        cloud = np.asarray(
            list(
                point_cloud2.read_points(
                    cloud, field_names=("x", "y", "z"), skip_nans=False
                )
            )
        ).reshape(rgb.shape)
        xyz1, xy2 = get_features_xyz_xy(good_old, good_new, prev_cloud, cloud)
        #print(xyz1.shape)
        if(len(xyz1)<10):
            features_detected = 0.0001
            print("less features")
            print("setting high variance")
            odom_publisher(rotations,trajectory,features_detected)
            prev_rgb = rgb.copy()
            prev_gray = gray.copy()
            prev_cloud = cloud.copy()
            p0 = good_new.reshape(-1, 1, 2)
            if NUM_CALLED % 3 == 0:
                test = fast.detect(prev_gray,None)
                test = cv2.KeyPoint_convert(test)
                p0 = test.reshape((-1,1,2))
                mask = np.zeros_like(prev_rgb)
            print("Finished ", NUM_CALLED)
            NUM_CALLED += 1

        else:
            _, rvec, tvec, inliers = cv2.solvePnPRansac(xyz1, xy2, K,(0,0,0,0),useExtrinsicGuess = False ,iterationsCount = 100,reprojectionError = 5.0,confidence = 0.90,flags = 2)

            if inliers is None or len(inliers) == 0:
                trust = 0.01
                print("less inliers")
                print("using 3d to 3d approch")
                xyz1, xyz2 = get_features_xyz(good_old, good_new, prev_cloud, cloud)
                R, t = rowan.mapping.davenport(np.asarray(xyz1), np.asarray(xyz2))
                position =  trajectory[-1] - np.dot(rotations[-1],t)
                rotations.append(np.dot(rotations[-1],R))
                trajectory.append(position)
                prev_rgb = rgb.copy()
                prev_gray = gray.copy()
                prev_cloud = cloud.copy()
                odom_publisher(rotations,trajectory,trust)
                p0 = good_new.reshape(-1, 1, 2)
                if NUM_CALLED % 3 == 0:
                    test = fast.detect(prev_gray,None)
                    test = cv2.KeyPoint_convert(test)
                    p0 = test.reshape((-1,1,2))
                    mask = np.zeros_like(prev_rgb)
                print("Finished ", NUM_CALLED)
                NUM_CALLED += 1

            else:
                #print(inliers)
                print(trajectory[-1],g_truth[-1])
                trust = len(inliers)
                trust = min(trust,features_detected)
                rmat, _ = cv2.Rodrigues(rvec)
                position =  trajectory[-1] - np.dot(rotations[-1],tvec)
                rotations.append(np.dot(rotations[-1],rmat))
                trajectory.append(position)
                odom_publisher(rotations,trajectory,trust)
                #3d plottingrosrun tf view_frames
                traj = np.asarray(trajectory).reshape((-1,3))
                ground_traj = np.asarray(g_truth).reshape((-1,3))
                #plt.subplot(1, 2, 1, projection='3d').plot(traj[:, 0], traj[:, 1], traj[:, 2])
                #plt.subplot(1, 2, 2, projection='3d').plot(ground_traj[:, 0], -1*ground_traj[:, 1], -1*ground_traj[:, 2])
                plt.plot(traj[:, 0], -1*traj[:, 2],color='blue')
                plt.plot(ground_traj[:, 0], ground_traj[:, 1],color='red')
                plt.xlabel("x ")
                plt.ylabel("y")
                plt.show()
                plt.pause(0.01)
                

                #listener = tf.TransformListener()
                #(trans,rot) = listener.lookupTransform( 'base_link',"rs200_camera", rospy.Time(0))
                #print(trans,rot)
                #2d plotting
                #plt.plot(traj[:, 1], traj[:, 2])
                #plt.show()
                #plt.pause(0.01)
                
                prev_rgb = rgb.copy()
                prev_gray = gray.copy()
                prev_cloud = cloud.copy()
                p0 = good_new.reshape(-1, 1, 2)
                if NUM_CALLED % 3 == 0:
                    test = fast.detect(prev_gray,None)
                    test = cv2.KeyPoint_convert(test)
                    p0 = test.reshape((-1,1,2))
                    mask = np.zeros_like(prev_rgb)
                print("Finished ", NUM_CALLED)
                NUM_CALLED += 1


image_sub = Subscriber("/r200/mycamera/color/color_rect", Image)
cloud_sub = Subscriber("/camera/depth/points", PointCloud2)
sub1 = Subscriber("/gt", Pose)


#should subscribe to localization node which will provide starting rotation and translation in case camera is unable to detect any features. 


ats = ApproximateTimeSynchronizer(
    [image_sub, cloud_sub,sub1], 100, 100, allow_headerless=True
)
ats.registerCallback(on_recieve)

rospy.spin()
