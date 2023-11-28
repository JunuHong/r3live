import numpy as np
import cv2
import open3d as o3d
import yaml

# Read camera calibration parameters from r3live_config_calib.yaml
with open('/home/hong/catkin_ws/src/r3live/config/r3live_config_calib.yaml', 'r') as f:
    calib_data = yaml.safe_load(f)

camera_matrix = np.array(calib_data['r3live_vio']['camera_intrinsic']).reshape(3, 3)
dist_coeffs = np.array(calib_data['r3live_vio']['camera_dist_coeffs'])
R = np.array(calib_data['r3live_vio']['camera_ext_R']).reshape(3, 3)
T = np.array(calib_data['r3live_vio']['camera_ext_t']).reshape(3, 1)

# Read image and pointcloud data
image = cv2.imread('../log/image/0.png')
pointcloud = o3d.io.read_point_cloud('../log/lidar/0.pcd')
image_undistorted = cv2.undistort(image, camera_matrix, dist_coeffs)

# Transform point cloud to camera frame
extrinsic = np.hstack((R, T))
extrinsic = np.vstack((extrinsic, np.array([0, 0, 0, 1])))
pointcloud.transform(extrinsic)

# Project points onto image plane
points = np.asarray(pointcloud.points)*0.001
colors = np.asarray(image_undistorted)[np.round(-points[:, 1]).astype(int), np.round(-points[:, 2]).astype(int)]
print(colors.shape)
pointcloud.colors = o3d.utility.Vector3dVector(colors / 255.0)

# Create coordinate frames for image and point cloud coordinate frames
image_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
pointcloud_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

# Visualize colored point cloud and coordinate frames
o3d.visualization.draw_geometries([pointcloud, image_frame, pointcloud_frame])
