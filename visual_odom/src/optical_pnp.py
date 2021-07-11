#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2 , CameraInfo
from sensor_msgs import point_cloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf
from tf2_msgs.msg import TFMessage
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as Rt
import scipy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import rowan

plt.ion()
bridge = CvBridge()
fast = cv2.FastFeatureDetector_create()
fast.setNonmaxSuppression(1)

#init ros node and taking first img ,pointcloud and camera parameters
rospy.init_node("imgread", anonymous=True)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()


#transformation between baselink and odometry
tf_id = 0
print("tf")
tf_msg = rospy.wait_for_message("/tf_static", TFMessage)
print("receiverd tf")
for i in range(len(tf_msg.transforms)):
    if(tf_msg.transforms[i].header.frame_id == "base_link" and tf_msg.transforms[i].child_frame_id == "rs200_camera"):
        tf_id = i
        print(tf_id)
tx_b_c = tf_msg.transforms[tf_id].transform.translation.x
ty_b_c = tf_msg.transforms[tf_id].transform.translation.y
tz_b_c = tf_msg.transforms[tf_id].transform.translation.z
qx_b_c = tf_msg.transforms[tf_id].transform.rotation.x
qy_b_c = tf_msg.transforms[tf_id].transform.rotation.y
qz_b_c = tf_msg.transforms[tf_id].transform.rotation.z
qw_b_c = tf_msg.transforms[tf_id].transform.rotation.w
R_b_c= Rt.from_quat([qx_b_c,qy_b_c ,qz_b_c,qw_b_c])
R_b_c = R_b_c.as_dcm().reshape((3,3))
transformation_b_c = np.zeros((3,4))
transformation_b_c[:,:3] = R_b_c
transformation_b_c[0,3] = tx_b_c
transformation_b_c[1,3] = ty_b_c
transformation_b_c[2,3] = tz_b_c
temp = np.zeros((3,1))
temp[0][0] = -1*tx_b_c
temp[1][0] = -1*ty_b_c
temp[2][0] = -1*tz_b_c
R_c_b= R_b_c.T
temp = np.matmul(R_b_c.T,temp)
transformation_c_b = np.zeros((3,4))
transformation_c_b[:,:3] = R_c_b
transformation_c_b[0,3] = temp[0][0]
transformation_c_b[1,3] = temp[1][0]
transformation_c_b[2,3] = temp[2][0]
print("transformations")
print(transformation_c_b,transformation_b_c)



#waiting for first msg (image,depth,camera_info)
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
# a CLAHE object (Arguments are optional).
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
prev_gray = clahe.apply(prev_gray)
p0 = fast.detect(prev_gray,None)
p0 = cv2.KeyPoint_convert(p0)
p0 = p0.reshape((-1,1,2))

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

def rmat_to_quaternion(rmat):
    r = Rt.from_dcm(rmat)
    q = r.as_quat()
    return q

def ekf_info(msg):
    filtered_tran = np.zeros((3,1))
    filtered_tran[0][0] = msg.pose.pose.position.x
    filtered_tran[1][0] = msg.pose.pose.position.y
    filtered_tran[2][0] = msg.pose.pose.position.z
    x1 = msg.pose.pose.orientation.x
    y1 = msg.pose.pose.orientation.y
    z1 = msg.pose.pose.orientation.z
    w1 = msg.pose.pose.orientation.w
    q = Rt.from_quat([x1,y1,z1,w1])
    rot = q.as_dcm().reshape((3,3))
    return filtered_tran,rot

def odom_publisher(rotations,trajectory,trust):
    trans_curr = np.eye(4)
    trans_curr[:3,:3] = rotations[-1]
    trans_curr[:3,3] = trajectory[-1].reshape(trans_curr[:3,3].shape)
    xyz = np.matmul(transformation_c_b,trans_curr)
    r_prev = xyz[:3,:3]
    t_prev = xyz[:3,3].reshape((3,1))
    print(r_prev,t_prev)
    current_time = rospy.Time.now()
    odom_quat = rmat_to_quaternion(r_prev)
    odom_broadcaster = tf.TransformBroadcaster()
    x = t_prev[0][0]
    y = t_prev[1][0]
    z = t_prev[2][0]

    max_covariance = 100000
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(x, y,z), Quaternion(*odom_quat))
    odom.pose.covariance = ((5/trust)*np.eye(6)).flatten()

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    odom.twist.covariance = (max_covariance*np.eye(6)).flatten() 

    #odom_broadcaster.sendTransform(odom)
    odom_broadcaster.sendTransform(
        (x, y,z),
        odom_quat,
        current_time,
        odom.child_frame_id,
        odom.header.frame_id
        
    )
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

#taking initial pose as gt;
print("/gt")
gt_msg = rospy.wait_for_message("/gt", Pose)
temp = np.zeros((3,1))
temp[0][0] = gt_msg.position.x
temp[1][0] = gt_msg.position.y
temp[2][0] = 0

#temp :ground truth at t = 0
rotations = []
NUM_CALLED = 0
trajectory = []

#rotations.append(np.eye(3))
g_truth = []
g_truth.append(temp)

trans_prev = np.eye(4)
trans_prev[:3,:3] = (Rt.from_quat([gt_msg.orientation.x,
    gt_msg.orientation.y, 
    gt_msg.orientation.z, 
    gt_msg.orientation.w ])).as_dcm().reshape((3,3))

trans_prev[:3,3] = temp.reshape(trans_prev[:3,3].shape)
print(":)")
xyz = np.matmul(transformation_b_c,trans_prev)
r_prev = xyz[:3,:3]
t_prev = xyz[:3,3].reshape((3,1))
rotations.append(r_prev)
trajectory.append(t_prev.reshape((3,1)))
print(g_truth,trajectory)
transformation_info = []
info = np.eye(4)
info[:3,:4] = xyz
transformation_info.append(info)
motion_estimate = []
motion_estimate.append(np.matmul(transformation_c_b,transformation_info[-1])[:,3])
print(motion_estimate)
#trajectory,rotations,transformation_info,g_truth

def on_recieve(rgb, cloud ,g_t):
    global prev_rgb, prev_cloud, p0, prev_gray, NUM_CALLED, mask
    temp = np.zeros((3,1))
    temp[0][0] = g_t.position.x
    temp[1][0] = g_t.position.y
    temp[2][0] = 0
    #print("Started ", NUM_CALLED)
    g_truth.append(temp)
    rgb = bridge.imgmsg_to_cv2(rgb, "bgr8")
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    good_new, good_old = optical_flow_matches(gray,prev_gray,p0)
    features_detected = good_new.shape[0] 
    if(features_detected is None or features_detected == 0):
        features_detected = 0.0001
        #odom_publisher(rotations,trajectory,features_detected)
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
                print("using 3d to 3d")
                xyz1, xyz2 = get_features_xyz(good_old, good_new, prev_cloud, cloud)
                R, t = rowan.mapping.davenport(np.asarray(xyz1), np.asarray(xyz2))
                position =  trajectory[-1] - np.matmul(rotations[-1],t)
                rotations.append(np.matmul(rotations[-1],R))
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
                trust = len(inliers)
                trust = min(trust,features_detected)
                rmat, _ = cv2.Rodrigues(rvec)
                position =  trajectory[-1] - np.matmul(rotations[-1],tvec)
                rotations.append(np.matmul(rotations[-1],rmat))
                trajectory.append(position)
                info = np.eye(4)
                info[:3,:3] = rotations[-1]
                info[:3,3] = trajectory[-1].reshape(3,)
                transformation_info.append(info)
                temp = np.matmul(transformation_c_b,transformation_info[-1])[:,3]
                temp[2,] = 0
                print(temp) 
                motion_estimate.append(temp)
                temp = np.matmul(transformation_c_b,transformation_info[-1])
                temp[:,3] = motion_estimate[-1]
                info = np.eye(4)
                info[:3,:4] = temp
                rotations[-1] = info[:3,:3]
                trajectory[-1] = info[:3,3].reshape(3,1) 

                #odom_publisher(rotations,trajectory,trust)
                #3d plottingrosrun tf view_frames
                traj = np.asarray(motion_estimate).reshape((-1,3))
                ground_traj = np.asarray(g_truth).reshape((-1,3))
                print(np.mean(ground_traj-traj))
                plt.plot(traj[:, 0], traj[:, 1],color='blue')
                plt.plot(ground_traj[:, 0], ground_traj[:, 1],color='red')
                #plt.plot(filtered_traj[:, 0], filtered_traj[:, 1],color='green')
                plt.xlabel("x")
                plt.ylabel("y")
                plt.show()
                plt.pause(0.01)
                prev_rgb = rgb.copy()
                prev_gray = gray.copy()
                prev_cloud = cloud.copy()
                p0 = good_new.reshape(-1, 1, 2)
                if NUM_CALLED % 3 == 0:
                    test = fast.detect(prev_gray,None)
                    test = cv2.KeyPoint_convert(test)
                    p0 = test.reshape((-1,1,2))
                    mask = np.zeros_like(prev_rgb)
                #print("Finished ", NUM_CALLED)
                NUM_CALLED += 1

image_sub = Subscriber("/r200/mycamera/color/color_rect", Image)
cloud_sub = Subscriber("/camera/depth/points", PointCloud2)
sub1 = Subscriber("/gt", Pose)
ats = ApproximateTimeSynchronizer(
    [image_sub, cloud_sub,sub1], 100, 100, allow_headerless=True
)
ats.registerCallback(on_recieve)

rospy.spin()
