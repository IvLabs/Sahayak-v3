import cv2

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs import point_cloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

plt.ion()
bridge = CvBridge()
orb = cv2.ORB_create()
index_params = dict(
    algorithm=6, table_number=6, key_size=12, multi_probe_level=1  # 12  # 20
)  # 2
search_params = dict(checks=100)
flann = cv2.FlannBasedMatcher(index_params, search_params)
draw_params = dict(
    matchColor=(0, 255, 0), flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
)

rospy.init_node("imgread", anonymous=True)


def features_deepcopy(f):
    return [
        cv2.KeyPoint(
            x=k.pt[0],
            y=k.pt[1],
            _size=k.size,
            _angle=k.angle,
            _response=k.response,
            _octave=k.octave,
            _class_id=k.class_id,
        )
        for k in f
    ]


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
prev_kp, prev_des = orb.detectAndCompute(prev_rgb, None)

z_motion = []
y_rotations = []
NUM_CALLED = 0
trajectory = []
robot_pose = []
robot_pose.append(np.eye(4))


def filter_ratio_test(matches):
    ratio_thresh = 0.8
    good_matches = []
    for match in matches:
        if len(match) == 2:
            m, n = match
            if m.distance < ratio_thresh * n.distance:
                good_matches.append([m])
    return good_matches


def draw_matches(prev_img, img, prev_kp, kp, matches):
    drawn_matches = cv2.drawMatchesKnn(
        prev_img, prev_kp, img, kp, matches, None, **draw_params
    )
    return drawn_matches


def get_features_xyz(matches, prev_kp, kp, prev_cloud, cloud):
    xyz1 = []
    xy2 = []
    for match in matches:
        if len(match) > 0:
            match = match[0]
            kp1 = prev_kp[match.queryIdx]
            pt1 = np.asarray(prev_cloud[int(kp1.pt[1]), int(kp1.pt[0])])
            x2, y2 = kp[match.trainIdx].pt
            if not np.isnan(pt1).any():
                xyz1.append(pt1)
                xy2.append([x2, y2])
    return np.asarray(xyz1), np.asarray(xy2)


def on_recieve(rgb, cloud):
    global prev_rgb, prev_cloud, prev_kp, prev_des, NUM_CALLED
    print("Started ", NUM_CALLED)
    rgb = bridge.imgmsg_to_cv2(rgb, "bgr8")
    cloud = np.asarray(
        list(
            point_cloud2.read_points(
                cloud, field_names=("x", "y", "z"), skip_nans=False
            )
        )
    ).reshape(rgb.shape)
    kp, des = orb.detectAndCompute(rgb, None)
    matches = flann.knnMatch(prev_des, des, k=2)
    good_matches = filter_ratio_test(matches)
    # drawn_matches = draw_matches(prev_rgb, rgb, prev_kp, kp, good_matches)
    # drawn_matches = cv2.cvtColor(drawn_matches, cv2.COLOR_BGR2RGB)
    # plt.subplot(3, 1, 1).imshow(drawn_matches)
    # plt.axis("off")
    xyz1, xy2 = get_features_xyz(matches, prev_kp, kp, prev_cloud, cloud)
    _, rvec, tvec, _ = cv2.solvePnPRansac(xyz1, xy2, K, None)
    rmat, _ = cv2.Rodrigues(rvec)
    current_pose = np.eye(4)
    current_pose[0:3, 0:3] = rmat
    current_pose[0:3, 3] = tvec.T
    robot_pose.append(np.dot(robot_pose[-1], np.linalg.inv(current_pose)))
    position = np.dot(robot_pose[-1], np.array([0., 0., 0., 1.]))
    trajectory.append(position)
    traj = np.asarray(trajectory)
    # plt.subplot(3, 1, 1).plot(traj[:, 0], traj[:, 2])
    plt.subplot(1, 1, 1, projection='3d').plot(traj[:, 0], traj[:, 1], traj[:, 2])
    print("R", rvec)
    print("T", tvec)
    y_rotations.append(rvec[2])
    # plt.subplot(3, 1, 2).plot(y_rotations)
    z_motion.append(tvec[2])
    # plt.subplot(3, 1, 3).plot(z_motion)
    plt.show()
    plt.pause(0.01)
    prev_rgb = rgb.copy()
    prev_cloud = cloud.copy()
    prev_kp = features_deepcopy(kp)
    prev_des = des.copy()
    NUM_CALLED += 1


image_sub = Subscriber("/r200/mycamera/color/color_rect", Image)
cloud_sub = Subscriber("/camera/depth/points", PointCloud2)

ats = ApproximateTimeSynchronizer(
    [image_sub, cloud_sub], 100, 100, allow_headerless=True
)
ats.registerCallback(on_recieve)

rospy.spin()
