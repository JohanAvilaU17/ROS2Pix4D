#include "utility.h"

class ROSToPix4D
{
public:
    ParamServer parSer;

    ros::Subscriber sub_img;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_odom;

    ros::Publisher pub_flag_img;
    ros::Publisher pub_flag_gps;
    ros::Publisher pub_flag_odom;

    std_msgs::Int8 pub_flag_1;
    std_msgs::Int8 pub_flag_0;

    string path_directory;
    string path_directory_img;

    string data_gps;
    string data_odom;

    string trajectory_odom;

    ROSToPix4D()
    {
        this->pub_flag_1.data = 1;
        this->pub_flag_0.data = 0;

        this->path_directory.append(this->parSer.get_path_package());
        this->path_directory.append("_result/");

        this->path_directory_img.append(this->path_directory);
        this->path_directory_img.append("img/");

        this->data_gps.append("stamp_nsec");
        this->data_gps.append(" ");
        this->data_gps.append("latitude");
        this->data_gps.append(" ");
        this->data_gps.append("longitude");
        this->data_gps.append(" ");
        this->data_gps.append("altitude");
        this->data_gps.append("\n");

        this->data_odom.append("stamp_nsec");
        this->data_odom.append(" ");
        this->data_odom.append("x");
        this->data_odom.append(" ");
        this->data_odom.append("y");
        this->data_odom.append(" ");
        this->data_odom.append("z");
        this->data_odom.append(" ");
        this->data_odom.append("qx");
        this->data_odom.append(" ");
        this->data_odom.append("qy");
        this->data_odom.append(" ");
        this->data_odom.append("qz");
        this->data_odom.append(" ");
        this->data_odom.append("qw");
        this->data_odom.append("\n");
    }

    void setting_subscriber()
    {
        ros::NodeHandle nh;
        string topics_img = this->parSer.get_topic("img");
        string topics_gps = this->parSer.get_topic("gps");
        string topics_odom = this->parSer.get_topic("odom");

        this->sub_img = nh.subscribe<sensor_msgs::Image>(
            topics_img.c_str(),
            100,
            &ROSToPix4D::callback_img,
            this,
            ros::TransportHints().tcpNoDelay());

        this->sub_gps = nh.subscribe<sensor_msgs::NavSatFix>(
            topics_gps.c_str(),
            100,
            &ROSToPix4D::callback_gps,
            this,
            ros::TransportHints().tcpNoDelay());

        this->sub_odom = nh.subscribe<nav_msgs::Odometry>(
            topics_odom.c_str(),
            100,
            &ROSToPix4D::callback_odom,
            this,
            ros::TransportHints().tcpNoDelay());
    }

    void setting_publisher()
    {
        ros::NodeHandle nh;

        this->pub_flag_img = nh.advertise<std_msgs::Int8>(
            "/ros_to_pix4d/flag_img",
            1);

        this->pub_flag_gps = nh.advertise<std_msgs::Int8>(
            "/ros_to_pix4d/flag_gps",
            1);

        this->pub_flag_odom = nh.advertise<std_msgs::Int8>(
            "/ros_to_pix4d/flag_odom",
            1);
    }

    void callback_img(const sensor_msgs::Image::ConstPtr &img_ros)
    {
        this->pub_flag_img.publish(this->pub_flag_1);

        int32_t stamp_nsec = img_ros->header.stamp.nsec;
        int32_t stamp_sec = img_ros->header.stamp.sec;
        int64_t index_img = stamp_nsec + (stamp_sec * 1e9);

        string path_img;
        path_img.append(this->path_directory_img);
        path_img.append(std::to_string(index_img));
        path_img.append(".");
        path_img.append(this->parSer.img_format);

        cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(img_ros);

        cv::imwrite(path_img, img->image);

        this->pub_flag_img.publish(this->pub_flag_0);
    }

    void callback_gps(const sensor_msgs::NavSatFix::ConstPtr &gps_ros)
    {
        this->pub_flag_gps.publish(this->pub_flag_1);

        int32_t stamp_nsec = gps_ros->header.stamp.nsec;
        int32_t stamp_sec = gps_ros->header.stamp.sec;
        int64_t index_gps = stamp_nsec + (stamp_sec * 1e9);

        string pose_gps;
        pose_gps.append(std::to_string(index_gps));
        pose_gps.append(" ");
        pose_gps.append(std::to_string(gps_ros->latitude));
        pose_gps.append(" ");
        pose_gps.append(std::to_string(gps_ros->longitude));
        pose_gps.append(" ");
        pose_gps.append(std::to_string(gps_ros->altitude));
        pose_gps.append("\n");

        this->data_gps.append(pose_gps);
        this->pub_flag_gps.publish(this->pub_flag_0);
    }

    void callback_odom(const nav_msgs::Odometry::ConstPtr &odom_ros)
    {
        this->pub_flag_odom.publish(this->pub_flag_1);

        int32_t stamp_nsec = odom_ros->header.stamp.nsec;
        int32_t stamp_sec = odom_ros->header.stamp.sec;
        int64_t index_odom = stamp_nsec + (stamp_sec * 1e9);

        string pose_odom;

        pose_odom.append(std::to_string(index_odom));
        pose_odom.append(" ");
        pose_odom.append(std::to_string(odom_ros->pose.pose.position.x));
        pose_odom.append(" ");
        pose_odom.append(std::to_string(odom_ros->pose.pose.position.y));
        pose_odom.append(" ");
        pose_odom.append(std::to_string(odom_ros->pose.pose.position.z));
        pose_odom.append(" ");
        pose_odom.append(std::to_string(odom_ros->pose.pose.orientation.x));
        pose_odom.append(" ");
        pose_odom.append(std::to_string(odom_ros->pose.pose.orientation.y));
        pose_odom.append(" ");
        pose_odom.append(std::to_string(odom_ros->pose.pose.orientation.z));
        pose_odom.append(" ");
        pose_odom.append(std::to_string(odom_ros->pose.pose.orientation.w));
        pose_odom.append("\n");

        this->data_odom.append(pose_odom);
        this->pub_flag_odom.publish(this->pub_flag_0);
    }

    void setting_directories_result()
    {
        mkdir(this->path_directory.c_str(), 0777);
        mkdir(this->path_directory_img.c_str(), 0777);
    }

    void save_trajectories()
    {
        string path_file_gps;
        path_file_gps.append(this->path_directory);
        path_file_gps.append("Data_GPS.txt");

        ofstream file_gps;
        file_gps.open(path_file_gps);
        file_gps << this->data_gps;
        file_gps.close();

        string path_file_odom;
        path_file_odom.append(this->path_directory);
        path_file_odom.append("Data_Odom.txt");

        ofstream file_odom;
        file_odom.open(path_file_odom);
        file_odom << this->data_odom;
        file_odom.close();
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_to_pix4d");

    ROSToPix4D ros_to_pix4d;
    ros_to_pix4d.setting_directories_result();
    ros_to_pix4d.setting_subscriber();
    ros_to_pix4d.setting_publisher();

    ROS_INFO("\033[1;32m-> ROSTOPix4D.\033[0m");

    while (ros::ok())
    {
        ros::spinOnce();
    }

    ros_to_pix4d.save_trajectories();

    return 0;
}