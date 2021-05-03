import cv2

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import ros_numpy
from sensor_msgs import point_cloud2
import math
from mpl_toolkits.mplot3d import Axes3D  # <--- This is important for 3d plotting
import matplotlib.pyplot as plt

plt.ion()
bridge = CvBridge()
sift = cv2.ORB_create()

index_params = dict(
    algorithm=6,
    table_number=6,  # 12
    key_size=12,  # 20
    multi_probe_level=1)  # 2
search_params = dict(checks=100)
flann = cv2.FlannBasedMatcher(index_params, search_params)
draw_params = dict(matchColor=(0, 255, 0),
                   flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

rospy.init_node("imgread", anonymous=True)

# Checks if a matrix is a valid rotation matrix.


def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).


def rotationMatrixToEulerAngles(R):

    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def rigid_transform_3D(A, B):
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception("matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception("matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = np.dot(Am, np.transpose(Bm))

    # sanity check
    # if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2, :] *= -1
        R = np.dot(Vt.T, U.T)

    t = np.dot(-R, centroid_A) + centroid_B

    return R, t


current_position = np.zeros(shape=(3, 1))
current_rotation = np.eye(3)

all_positions = []
# ax = plt.axes(projection='3d')

while True:
    print("Waiting...")
    rgb1_msg = rospy.wait_for_message("/r200/mycamera/color/color_rect", Image)
    rgb1 = bridge.imgmsg_to_cv2(rgb1_msg, "bgr8")
    # cv2.imshow("rgb1", rgb1)
    # cv2.waitKey(1)

    pointcloud1 = np.asarray(
        list(
            point_cloud2.read_points(rospy.wait_for_message(
                "/camera/depth/points", PointCloud2),
                                     field_names=("x", "y", "z"),
                                     skip_nans=False))).reshape(rgb1.shape)
    rgb2_msg = rospy.wait_for_message("/r200/mycamera/color/color_rect", Image)
    rgb2 = bridge.imgmsg_to_cv2(rgb2_msg, "bgr8")
    # cv2.imshow("rgb2", rgb2)
    # cv2.waitKey(1)

    pointcloud2 = np.asarray(
        list(
            point_cloud2.read_points(rospy.wait_for_message(
                "/camera/depth/points", PointCloud2),
                                     field_names=("x", "y", "z"),
                                     skip_nans=False))).reshape(rgb2.shape)
    # print(pointcloud2.shape)
    # print(pointcloud2[1, 1])
    kps1, des1 = sift.detectAndCompute(rgb1, None)
    kps2, des2 = sift.detectAndCompute(rgb2, None)
    matches = flann.knnMatch(des1, des2, k=2)
    ratio_thresh = 0.5
    good_matches = []
    for match in matches:
        if len(match) == 2:
            m, n = match
            if m.distance < ratio_thresh * n.distance:
                good_matches.append([m])
    img3 = cv2.drawMatchesKnn(rgb1, kps1, rgb2, kps2, good_matches, None,
                              **draw_params)
    cv2.imshow("matches", img3)
    cv2.waitKey(1)
    xyz1 = []
    xyz2 = []
    for match in good_matches:
        match = match[0]
        kp1 = kps1[match.queryIdx]
        pt1 = np.asarray(pointcloud1[int(kp1.pt[1]), int(kp1.pt[0])])
        kp2 = kps2[match.trainIdx]
        pt2 = np.asarray(pointcloud2[int(kp2.pt[1]), int(kp2.pt[0])])
        if not np.isnan(pt1).any() and not np.isnan(pt2).any():
            xyz1.append(pt1)
            xyz2.append(pt2)
    # print(xyz1)
    # print(xyz2)
    R, t = rigid_transform_3D(np.asarray(xyz1).T, np.asarray(xyz2).T)
    print(R)
    print(t)
    movement = np.dot(current_rotation, t)
    current_rotation = np.dot(R, current_rotation)
    current_position = current_position + movement
    all_positions.append(current_position)
    all_positions_np = np.asarray(all_positions)
    print(all_positions_np.shape)
    plt.plot(all_positions_np[:, 0, 0], all_positions_np[:, 2, 0])
    plt.show()
    plt.pause(0.001)
