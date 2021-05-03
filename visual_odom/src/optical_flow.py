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
feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

# Parameters for lucas kanade optical flow
lk_params = dict(
    winSize=(15, 15),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
)

rospy.init_node("imgread", anonymous=True)


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

prev_gray = cv2.cvtColor(prev_rgb, cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)
mask = np.zeros_like(prev_rgb)
color = np.random.randint(0, 255, (100, 3))

z_motion = []
y_rotations = []
NUM_CALLED = 0


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


def draw_mask(frame, good_new, good_old):
    global mask
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv2.line(mask, (a, b), (c, d), color[i].tolist(), 2)
        frame = cv2.circle(frame, (a, b), 5, color[i].tolist(), -1)
    img = cv2.add(frame, mask)
    return img


def on_recieve(rgb, cloud):
    global prev_rgb, prev_cloud, p0, prev_gray, NUM_CALLED, mask
    print("Started ", NUM_CALLED)
    rgb = bridge.imgmsg_to_cv2(rgb, "bgr8")
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    p1, st, err = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p0, None, **lk_params)
    good_new = p1[st == 1]
    good_old = p0[st == 1]
    cloud = np.asarray(
        list(
            point_cloud2.read_points(
                cloud, field_names=("x", "y", "z"), skip_nans=False
            )
        )
    ).reshape(rgb.shape)
    drawn_matches = draw_mask(rgb, good_new, good_old)
    plt.subplot(3, 1, 1).imshow(drawn_matches)
    plt.axis("off")
    xyz1, xyz2 = get_features_xyz(good_old, good_new, prev_cloud, cloud)
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
    prev_gray = gray.copy()
    prev_cloud = cloud.copy()
    p0 = good_new.reshape(-1, 1, 2)
    if NUM_CALLED % 3 == 0:
        p0 = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)
        mask = np.zeros_like(prev_rgb)
    print("Finished ", NUM_CALLED)
    NUM_CALLED += 1


image_sub = Subscriber("/r200/mycamera/color/color_rect", Image)
cloud_sub = Subscriber("/camera/depth/points", PointCloud2)

ats = ApproximateTimeSynchronizer(
    [image_sub, cloud_sub], 100, 100, allow_headerless=True
)
ats.registerCallback(on_recieve)

rospy.spin()
