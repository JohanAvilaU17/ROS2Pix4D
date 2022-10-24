#pragma once

#ifndef ULTILITY_H
#define ULTILITY_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>

#include <bits/stdc++.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

class ParamServer
{
public:
    ros::NodeHandle nh;
    string path_base_topics = "/ros2pix4d/topics/";
    string path_format = "/ros2pix4d/img/format";

    string img_format;

    ParamServer()
    {
        nh.param<std::string>(path_format, img_format, "jpg");
    }

    string get_topic(const char *name)
    {
        string path_topic;
        path_topic.append(path_base_topics);
        path_topic.append(name);

        string topics;
        nh.param<string>(path_topic, topics, "");
        return topics;
    }

    string get_path_package()
    {
        string path_package;
        path_package = ros::package::getPath("ros2pix4d");
        return path_package;
    }
};
#endif