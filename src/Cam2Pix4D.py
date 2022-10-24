#!/usr/bin/python3

import rospkg
import rosbag
import tf

import os
import time
import yaml
import cv2
import numpy as np
import pandas as pd
import open3d as o3d
import pymap3d as pm


class Cam2Pix4D:

    def __init__(self):

        ros_Pack = rospkg.RosPack()
        path_ros2pix4d = ros_Pack.get_path('ros2pix4d')

        path_file_params = path_ros2pix4d+"/config/params.yaml"
        with open(path_file_params, 'r') as f:
            self.params = yaml.load(f, Loader=yaml.FullLoader)["ros2pix4d"]

        self.path_directory = (
            path_ros2pix4d+"_"+self.params["rosbag"]["name"]+"/")
        try:
            os.mkdir(self.path_directory)
        except:
            pass

        self.flag_gps = False
        self.flag_imu = False

        self.rvec_dedrees = np.zeros(3)
        self.tvec_ned = np.zeros(3)

        self.mp_rot = np.eye(4)
        self.mp_trans = np.eye(4)

        self.pcd_trajectory = o3d.geometry.PointCloud()

    def callback_gps(self, gps):
        if self.flag_gps == False:
            self.flag_gps = True
            self.gps_org_lat = float(gps["latitude"])
            self.gps_org_lon = float(gps["longitude"])
            self.gps_org_alt = float(gps["altitude"])
        else:
            gps_lat = float(gps["latitude"])
            gps_lon = float(gps["longitude"])
            gps_alt = float(gps["altitude"])

            tvec_ned = pm.geodetic2ned(
                gps_lat, gps_lon, gps_alt,
                self.gps_org_lat, self.gps_org_lon, self.gps_org_alt)

            self.mp_trans[0, 3] = tvec_ned[0]
            self.mp_trans[1, 3] = tvec_ned[1]
            self.mp_trans[2, 3] = tvec_ned[2]

            self.tvec_ned = tvec_ned

    def callback_imu(self, imu):
        qx = float(imu["qx"])
        qy = float(imu["qy"])
        qz = float(imu["qz"])
        qw = float(imu["qw"])

        rvec = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))

        if self.flag_imu == False:
            self.flag_imu = True
            self.imu_org_rvec = np.array(rvec)
        else:
            rvec = (np.array(rvec) - self.imu_org_rvec)

            self.mp_rot[:3, :3] = cv2.Rodrigues(rvec)[0]
            self.rvec_dedrees = np.degrees(rvec)

    def callback_img(self, img_ros):

        xyz_tmp = np.zeros((1, 3))

        pcd_trajectory_tmp = o3d.geometry.PointCloud()
        pcd_trajectory_tmp.points = o3d.utility.Vector3dVector(xyz_tmp)

        pcd_trajectory_tmp.transform(self.mp_rot)
        pcd_trajectory_tmp.transform(self.mp_trans)

        self.pcd_trajectory.points.extend(pcd_trajectory_tmp.points)

    def save_trajectory(self):
        file = self.path_directory + "trajectory.pcd"
        o3d.io.write_point_cloud(file, self.pcd_trajectory)


print("\033[1;32----------\033[0m")

cam2pix4D = Cam2Pix4D()
print("\033[1;32m-> cam2pix4D.\033[0m")
time_tic = time.time()

print("\033[1;31m-> Read data.\033[0m")
path_file_gps = cam2pix4D.path_directory + "Data_GPS.txt"
path_file_imu = cam2pix4D.path_directory + "Data_IMU.txt"

data_gps = pd.read_csv(path_file_gps, sep=" ", header=0)
gps_stamp = np.array(data_gps["stamp_nsec"])
gps_stamp = gps_stamp.astype(np.int64)

data_imu = pd.read_csv(path_file_imu, sep=" ", header=0)
imu_stamp = np.array(data_imu["stamp_nsec"])
imu_stamp = imu_stamp.astype(np.int64)
print("\033[1;32m-> Ok.\033[0m")

print("\033[1;31m-> Open rosbag.\033[0m")
path_file_rosbag = (
    cam2pix4D.params["rosbag"]["path"] + "/" +
    cam2pix4D.params["rosbag"]["name"]+".bag")
data_rosbag = rosbag.Bag(path_file_rosbag)
print("\033[1;32m-> Ok.\033[0m")

print("\033[1;31m-> MetadataImg.\033[0m")
topic_cam = cam2pix4D.params["topics"]["cam"]

data_topics = []
data_topics.append(topic_cam)

for topic_msg, msg, _ in data_rosbag.read_messages(topics=data_topics):

    if topic_msg == topic_cam:
        cam_stmap_tmp = msg.header.stamp.to_nsec()

        err_gps_stamp = np.abs(cam_stmap_tmp-gps_stamp)
        err_gps_stamp_min = np.amin(err_gps_stamp)
        err_gps_stamp_min_pose = err_gps_stamp == err_gps_stamp_min

        gps_tmp = data_gps[err_gps_stamp_min_pose]
        cam2pix4D.callback_gps(gps_tmp)

        err_imu_stamp = np.abs(cam_stmap_tmp-imu_stamp)
        err_imu_stamp_min = np.amin(err_imu_stamp)
        err_imu_stamp_min_pose = err_imu_stamp == err_imu_stamp_min

        imu_tmp = data_imu[err_imu_stamp_min_pose]
        cam2pix4D.callback_imu(imu_tmp)

        cam2pix4D.callback_img(msg)

data_rosbag.close()
print("\033[1;32m-> Ok.\033[0m")

print("\033[1;31m-> save_trajectory.\033[0m")
cam2pix4D.save_trajectory()
print("\033[1;32m-> Ok.\033[0m")

time_final = time.time()-time_tic
print("\033[1;32m-> time =\033[0m", time_final)

print("\033[1;33----------\033[0m")
