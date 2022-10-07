#!/usr/bin/env python3

import rospy
import ros_numpy
import tf
import tf.transformations

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

import numpy as np
import open3d as o3d
from collections import deque
from ctypes import * # convert float to uint32

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

def convertCloudFromRosToOpen3d(ros_cloud):
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb

        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud

def pose_to_mat(pose_msg):
    return np.matmul(tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
                     tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
            )

def inverse_se3(trans):
    trans_inverse = np.eye(4)
    trans_inverse[:3, :3] = trans[:3, :3].T                          #R
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3]) #t
    return trans_inverse

received_cloud = o3d.geometry.PointCloud()
q_pose = deque(maxlen=10)
q_point = deque(maxlen=10)
poses = []
count = 0


def pointcloud_callback(msg):
    global q_point
    q_point.append([msg.header.seq, msg.header.stamp.to_sec(), convertCloudFromRosToOpen3d(msg)])


def odom_callback(msg):
    global q_pose
    cur_pose = pose_to_mat(msg)
    q_pose.append([msg.header.seq, msg.header.stamp.to_sec(), cur_pose])

def sync():
    global q_pose, q_point
    synced = False
    if not q_pose or not q_point:
        return False, None, None, None
    print(len(q_pose), len(q_point))

    seq_pose, t_pose, pose = q_pose.popleft()
    while q_point:
        seq_point, t_point, point = q_point.popleft()
        if abs(t_point - t_pose) < 0.01:
            synced = True
            break
    q_pose.clear()
    q_point.clear()
    return synced, pose, point, t_point

def save():
    global q_pose, q_point, count, poses
    synced, cur_pose, received_cloud, time = sync()
    if not synced:
        return

    body_cloud = received_cloud.transform(inverse_se3(cur_pose))
    output_filename = output_dir + "full" + str(count) + ".pcd"
    if (o3d.io.write_point_cloud(output_filename, body_cloud)):
        print("saved {}".format("full"+str(count)))
        cur_pose[3,3] = time
        poses.append(cur_pose)
        count += 1

if __name__ == "__main__":
    rospy.init_node('save_pcd_pose', anonymous=True)

    rospy.Subscriber("/render_pts", PointCloud2, pointcloud_callback)
    rospy.Subscriber("/aft_mapped_to_init", Odometry, odom_callback)
    output_dir = "/home/hong/project_codes/BALM/datas/custom/"

    while not rospy.is_shutdown():
        save()

    poses = np.array(poses).reshape((-1,4))
    np.savetxt(output_dir+"alidarPose.csv", poses, fmt='%.5f', delimiter=',')
    print("saved aLidarPoses")
