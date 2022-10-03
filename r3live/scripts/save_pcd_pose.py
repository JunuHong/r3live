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


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
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



received_cloud = None
count = 0
poses = []
time = 0.0

def pointcloud_callback(msg):
    global received_ros_cloud, count
    received_cloud=convertCloudFromRosToOpen3d(msg)
    output_filename = output_dir + "full" + str(count) + ".pcd"
    o3d.io.write_point_cloud(output_filename, received_cloud)
    print("saved {}".format("full"+str(count)))
    count += 1

def odom_callback(msg):
    global pose
    pose = pose_to_mat(msg)
    time = msg.header.stamp.to_sec()
    pose[3,3] = time
    poses.append(pose)
    print("saved {}".format(str(msg.header.seq)))

if __name__ == "__main__":
    rospy.init_node('save_pcd_pose', anonymous=True)
    
    rospy.Subscriber("/render_pts", PointCloud2, pointcloud_callback)
    rospy.Subscriber("/aft_mapped_to_init", Odometry, odom_callback)
    
    output_dir = "/home/hong/project_codes/BALM/datas/custom/"
    
    while not rospy.is_shutdown():
        continue
    poses = np.array(poses).reshape((-1,4))
    np.savetxt(output_dir+"alidarPose.csv", poses, fmt='%.5f', delimiter=',')
    print("saved aLidarPoses")
