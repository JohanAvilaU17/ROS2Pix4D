#!/usr/bin/python3

import rospkg
import tf

import os
import yaml
import numpy as np
import pandas as pd
import progressbar
import pyexiv2

import fractions


def dec2dms(coord, coord_type):

    coord_sign = -1 if coord < 0 else 1
    coord = abs(coord)
    dd = int(coord)
    mm = int((coord - dd)*60)
    ss = (coord - dd - (mm/60.0))*3600

    dd = fractions.Fraction(dd)
    mm = fractions.Fraction(mm)

    ss = int(ss*1000)
    ss = fractions.Fraction(ss, 1000)

    dms = [dd, mm, ss]
    dms_ref = ""

    if coord_type == "latitude":
        if coord_sign < 0:
            dms_ref = "S"
        elif coord_sign > 0:
            dms_ref = "N"

    elif coord_type == "longitude":
        if coord_sign < 0:
            dms_ref = "W"
        elif coord_sign > 0:
            dms_ref = "E"

    return dms, dms_ref


ros_Pack = rospkg.RosPack()
path_MapPilotScan = ros_Pack.get_path('map_pilot_scan')

# path_example_img = path_MapPilotScan+"/example_img/DJI_0695.JPG"

# print("example_img")
# print()

# example_img = pyexiv2.ImageMetadata(path_example_img)
# example_img.read()

# print("data Exif")
# for x in example_img.exif_keys:
#     print("->",
#           x, " - ",
#           example_img[x].value, " - ",
#           type(example_img[x].value))
# print()

# print("data Xmp")
# for x in example_img.xmp_keys:
#     print("->",
#           x, " - ",
#           example_img[x].value, " - ",
#           type(example_img[x].value))
# print()

pyexiv2.xmp.register_namespace('http://example.org/Camera/', 'Camera')

path_file_params = path_MapPilotScan+"/config/params.yaml"
with open(path_file_params, 'r') as f:
    params = yaml.load(f, Loader=yaml.FullLoader)["map_pilot_scan"]

path_directory = path_MapPilotScan + "_"+params["rosbag"]["name"]+"/"

path_directory_img = path_directory + "img/"
path_file_gps = path_directory + "Data_GPS.txt"
path_file_odom = path_directory + "Data_Odom.txt"

names_files_img = sorted(os.listdir(path_directory_img))

data_gps = pd.read_csv(path_file_gps, sep=" ", header=0)
gps_stamp = np.array(data_gps["stamp_nsec"])
gps_stamp = gps_stamp.astype(np.int64)

data_odom = pd.read_csv(path_file_odom, sep=" ", header=0)
odom_stamp = np.array(data_odom["stamp_nsec"])
odom_stamp = odom_stamp.astype(np.int64)

N = len(names_files_img)
bar_cont = 0
bar = progressbar.ProgressBar(max_value=N, redirect_stdout=True)

for name_file_img in names_files_img:

    path_file_img = path_directory_img + name_file_img
    name_img = name_file_img[:name_file_img.index(".")]

    err_gps_stamp = np.abs(int(name_img)-gps_stamp)
    err_gps_stamp_min = np.amin(err_gps_stamp)
    err_gps_stamp_min_pose = err_gps_stamp == err_gps_stamp_min

    gps_tmp = data_gps[err_gps_stamp_min_pose]

    latitude_tmp = float(gps_tmp["latitude"])
    longitude_tmp = float(gps_tmp["longitude"])
    altitude_tmp = float(gps_tmp["altitude"])

    altitude_tmp = int(altitude_tmp*1000)
    altitude_tmp = fractions.Fraction(altitude_tmp, 1000)
    # print(altitude_tmp)

    err_odom_stamp = np.abs(int(name_img)-odom_stamp)
    err_odom_stamp_min = np.amin(err_odom_stamp)
    err_odom_stamp_min_pose = err_odom_stamp == err_odom_stamp_min

    odom_tmp = data_odom[err_odom_stamp_min_pose]

    qx_tmp = float(odom_tmp["qx"])
    qy_tmp = float(odom_tmp["qy"])
    qz_tmp = float(odom_tmp["qz"])
    qw_tmp = float(odom_tmp["qw"])

    rot = list(tf.transformations.euler_from_quaternion(
        (qx_tmp, qy_tmp, qz_tmp, qw_tmp)))

    roll_tmp = (rot[0]*180)/np.pi
    pitch_tmp = (rot[1]*180)/np.pi
    Yaw_tmp = (rot[2]*180)/np.pi

    img_tmp = pyexiv2.ImageMetadata(path_file_img)
    img_tmp.read()

    img_tmp["Exif.GPSInfo.GPSLatitude"], img_tmp["Exif.GPSInfo.GPSLatitudeRef"] = dec2dms(
        latitude_tmp, "latitude")
    img_tmp["Exif.GPSInfo.GPSLongitude"], img_tmp["Exif.GPSInfo.GPSLongitudeRef"] = dec2dms(
        longitude_tmp, "longitude")
    img_tmp["Exif.GPSInfo.GPSAltitude"] = altitude_tmp
    img_tmp["Exif.GPSInfo.GPSAltitudeRef"] = "0"

    img_tmp["Xmp.Camera.Roll"] = roll_tmp
    img_tmp["Xmp.Camera.Pitch"] = pitch_tmp
    img_tmp["Xmp.Camera.Yaw"] = Yaw_tmp

    img_tmp.write()

    bar_cont = bar_cont+1
    bar.update(bar_cont)

bar.finish()
