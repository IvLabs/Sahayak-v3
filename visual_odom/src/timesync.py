import cv2

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rowan
import matplotlib.pyplot as plt

plt.ion()
bridge = CvBridge()
sift = cv2.ORB_create()
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

prev_kp, prev_des = sift.detectAndCompute(prev_rgb, None)

z_motion = []
y_rotations = []
NUM_CALLED = 0


def filter_ratio_test(matches):
    ratio_thresh = 0.5
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
    xyz2 = []
    for match in matches:
        match = match[0]
        kp1 = prev_kp[match.queryIdx]
        pt1 = np.asarray(prev_cloud[int(kp1.pt[1]), int(kp1.pt[0])])
        kp2 = kp[match.trainIdx]
        pt2 = np.asarray(cloud[int(kp2.pt[1]), int(kp2.pt[0])])
        if not np.isnan(pt1).any() and not np.isnan(pt2).any():
            xyz1.append(pt1)
            xyz2.append(pt2)
    return np.asarray(xyz1), np.asarray(xyz2)


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
    kp, des = sift.detectAndCompute(rgb, None)
    matches = flann.knnMatch(prev_des, des, k=2)
    good_matches = filter_ratio_test(matches)
    drawn_matches = draw_matches(prev_rgb, rgb, prev_kp, kp, good_matches)
    drawn_matches = cv2.cvtColor(drawn_matches, cv2.COLOR_BGR2RGB)
    plt.subplot(3, 1, 1).imshow(drawn_matches)
    plt.axis("off")
    xyz1, xyz2 = get_features_xyz(matches, prev_kp, kp, prev_cloud, cloud)
    R, t = rowan.mapping.davenport(np.asarray(xyz1), np.asarray(xyz2))
    rotations = rowan.to_euler(R, convention="xyz", axis_type="intrinsic")
    print("R", rotations)
    print("T", t)
    y_rotations.append(rotations[2])
    plt.subplot(3, 1, 2).plot(y_rotations)
    z_motion.append(-t[2])
    plt.subplot(3, 1, 3).plot(z_motion)
    plt.draw()
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
