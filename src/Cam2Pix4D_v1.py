#!/usr/bin/python3

import rospkg
import rosbag

import time
import os
import yaml
import numpy as np
import pandas as pd
import progressbar
import pyexiv2
import cv2

import fractions


class DataGPS:

    def __init__(self):
        self.stamp_nsec = []
        self.latitude = []
        self.longitude = []
        self.altitude = []

    def callback_gps(self, gps_ros):
        self.stamp_nsec.append(gps_ros.header.stamp.to_nsec())
        self.latitude.append(gps_ros.latitude)
        self.longitude.append(gps_ros.longitude)
        self.altitude.append(gps_ros.altitude)


class DataOdom:

    def __init__(self):
        self.stamp_nsec = []
        self.x = []
        self.y = []
        self.z = []
        self.qx = []
        self.qy = []
        self.qz = []
        self.qw = []

    def callback_odom(self, odom_ros):
        self.stamp_nsec.append(odom_ros.header.stamp.to_nsec())
        self.x.append(odom_ros.pose.pose.position.x)
        self.y.append(odom_ros.pose.pose.position.y)
        self.z.append(odom_ros.pose.pose.position.z)
        self.qx.append(odom_ros.pose.pose.orientation.x)
        self.qy.append(odom_ros.pose.pose.orientation.y)
        self.qz.append(odom_ros.pose.pose.orientation.z)
        self.qw.append(odom_ros.pose.pose.orientation.w)

    def save(self, path_directory):
        file = path_directory+"/Data_odom.txt"
        N = len(self.stamp_nsec)
        with open(file, "w+") as f:
            # f.write("stamp_nsec x y z qx qy qz qw\n")
            for idx in range(0, N):
                f.write(str(self.stamp_nsec[idx]) + " ")
                f.write(str(self.x[idx]) + " ")
                f.write(str(self.y[idx]) + " ")
                f.write(str(self.z[idx]) + " ")
                f.write(str(self.qx[idx]) + " ")
                f.write(str(self.qy[idx]) + " ")
                f.write(str(self.qz[idx]) + " ")
                f.write(str(self.qw[idx]) + "\n")


ros_Pack = rospkg.RosPack()
path_MapPilotScan = ros_Pack.get_path('map_pilot_scan')

path_file_params = path_MapPilotScan+"/config/params.yaml"
with open(path_file_params, 'r') as f:
    params = yaml.load(f, Loader=yaml.FullLoader)["map_pilot_scan"]

path_directory = path_MapPilotScan+"_"+params["rosbag"]["name"]
try:
    os.mkdir(path_directory)
except:
    pass

print("-> Open rosbag")
time_tic = time.time()
path_file_rosbag = (params["rosbag"]["path"] + "/" +
                    params["rosbag"]["name"]+".bag")
data_rosbag = rosbag.Bag(path_file_rosbag)
print("-> ok")

print("-> Read data")
topic_gps = params["topics"]["gps"]
topic_odom = params["topics"]["odom"]

data_topics = []
data_topics.append(topic_gps)
data_topics.append(topic_odom)

data_gps = DataGPS()
data_odom = DataOdom()

for topic_msg, msg, _ in data_rosbag.read_messages(topics=data_topics):
    if topic_msg == topic_gps:
        data_gps.callback_gps(msg)
    elif topic_msg == topic_odom:
        data_odom.callback_odom(msg)

time_final = time.time()-time_tic
print("-> ok")
print("-> time_final =", time_final)

data_rosbag.close()
data_odom.save(path_directory)
