#!/usr/bin/python3

import rospkg

import time
import os
import yaml
import numpy as np
import pandas as pd
import bagpy
import progressbar
import pyexiv2
import cv2

import fractions

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

path_file_rosbag = (params["rosbag"]["path"] + "/" +
                    params["rosbag"]["name"]+".bag")
print("-> open rosbag")
time_tic = time.time()
data_rosbag = bagpy.bagreader(bagfile=path_file_rosbag, tmp=False)
# print(data_rosbag.topic_table)
# print(dir(data_rosbag))
print("-> ok")

print("-> read gps")
topic_gps = params["topics"]["gps"]
data_gps = data_rosbag.message_by_topic(topic_gps)
data_gps = pd.read_csv(data_gps)
time_final = time.time()-time_tic
print("-> time_final =", time_final)
print(data_gps)
exit()

print("-> read odom")
topic_odom = params["topics"]["odom"]
data_odom = data_rosbag.message_by_topic(topic_odom)
data_odom = pd.read_csv(data_odom)
print(data_odom)
