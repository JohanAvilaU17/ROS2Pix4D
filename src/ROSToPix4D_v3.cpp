#include "utility.h"

class ROSToPix4D
{
public:
    ParamServer parSer;

    ros::Subscriber sub_img;
    ros::Subscriber sub_gps;

    ros::Publisher pub_flag_img;
    ros::Publisher pub_flag_gps;
    ros::Publisher pub_flag_save;

    bool state_img;
    bool state_gps;

    std_msgs::Int8 pub_flag_1;
    std_msgs::Int8 pub_flag_0;

    sensor_msgs::Image img_tmp;
    sensor_msgs::NavSatFix gps_tmp;

    string path_directory;
    string path_directory_img;

    int64_t index_img;

    string trajectory_gps;

    ROSToPix4D()
    {
        this->state_img = false;
        this->state_gps = false;

        this->pub_flag_1.data = 1;
        this->pub_flag_0.data = 0;

        this->path_directory.append(this->parSer.get_path_package());
        this->path_directory.append("_result/");

        this->path_directory_img.append(this->path_directory);
        this->path_directory_img.append("img/");

        this->index_img = 0;

        this->trajectory_gps.append("index_img");
        this->trajectory_gps.append(" ");
        this->trajectory_gps.append("header_seq");
        this->trajectory_gps.append(" ");
        this->trajectory_gps.append("latitude");
        this->trajectory_gps.append(" ");
        this->trajectory_gps.append("longitude");
        this->trajectory_gps.append(" ");
        this->trajectory_gps.append("altitude");
        this->trajectory_gps.append("\n");
    }

    void setting_subscriber()
    {
        ros::NodeHandle nh;
        string topics_img = this->parSer.get_topic("img");
        string topics_gps = this->parSer.get_topic("gps");

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

        this->pub_flag_save = nh.advertise<std_msgs::Int8>(
            "/ros_to_pix4d/flag_save",
            1);
    }

    void callback_img(const sensor_msgs::Image::ConstPtr &img_ros)
    {
        this->pub_flag_img.publish(this->pub_flag_1);

        this->img_tmp.header = img_ros->header;
        this->img_tmp.height = img_ros->height;
        this->img_tmp.width = img_ros->width;
        this->img_tmp.encoding = img_ros->encoding;
        this->img_tmp.data = img_ros->data;
        this->img_tmp.step = img_ros->step * 3;

        this->state_img = true;
        this->pub_flag_img.publish(this->pub_flag_0);
    }

    void callback_gps(const sensor_msgs::NavSatFix::ConstPtr &gps_ros)
    {
        this->pub_flag_gps.publish(this->pub_flag_1);

        this->gps_tmp.header = gps_ros->header;
        this->gps_tmp.latitude = gps_ros->latitude;
        this->gps_tmp.longitude = gps_ros->longitude;
        this->gps_tmp.altitude = gps_ros->altitude;

        this->state_gps = true;
        this->pub_flag_gps.publish(this->pub_flag_0);
    }

    void setting_directories_result()
    {
        mkdir(this->path_directory.c_str(), 0777);
        mkdir(this->path_directory_img.c_str(), 0777);
    }

    void save_img()
    {
        string path_img;
        path_img.append(this->path_directory_img);
        path_img.append(std::to_string(this->index_img));
        path_img.append(".");
        path_img.append(this->parSer.img_format);

        cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(this->img_tmp);

        cv::imwrite(path_img, img->image);

        this->index_img++;
        this->state_img = false;
    }

    void trajectory_tracking()
    {
        string pose_gps;
        pose_gps.append(std::to_string(this->index_img));
        pose_gps.append(" ");
        pose_gps.append(std::to_string(this->gps_tmp.header.seq));
        pose_gps.append(" ");
        pose_gps.append(std::to_string(this->gps_tmp.latitude));
        pose_gps.append(" ");
        pose_gps.append(std::to_string(this->gps_tmp.longitude));
        pose_gps.append(" ");
        pose_gps.append(std::to_string(this->gps_tmp.altitude));
        pose_gps.append("\n");

        this->trajectory_gps.append(pose_gps);
        this->state_gps = false;
    }

    void save_trajectories()
    {
        string path_file_gps;
        path_file_gps.append(this->path_directory);
        path_file_gps.append("trajectory_gps.txt");

        ofstream file_gps;
        file_gps.open(path_file_gps);
        file_gps << this->trajectory_gps;
        file_gps.close();
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
        if (ros_to_pix4d.state_img && ros_to_pix4d.state_gps)
        {
            ros_to_pix4d.pub_flag_save.publish(ros_to_pix4d.pub_flag_1);

            ros_to_pix4d.trajectory_tracking();
            ros_to_pix4d.save_img();

            ros_to_pix4d.pub_flag_save.publish(ros_to_pix4d.pub_flag_0);
        }
        ros::spinOnce();
    }

    ros_to_pix4d.save_trajectories();

    return 0;
}