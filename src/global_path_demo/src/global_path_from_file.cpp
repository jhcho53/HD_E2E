#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_path_from_file");
    ros::NodeHandle nh;

    // 글로벌 경로 발행자 설정
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/global_path", 20);
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map"; // 좌표계 설정

    // 파일에서 경로 데이터를 읽기
    std::ifstream infile("/home/viplab/hd/hd_ws/hmg_mission11_global_path.txt"); // 데이터 파일 경로 설정
    std::string line;

    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        std::string id;
        double x, y, z;

        // 파일 파싱
        if (!(iss >> id >> x >> y >> z)) { break; }

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose);
    }

    ros::Rate rate(1);
    while (ros::ok())
    {
        path_msg.header.stamp = ros::Time::now();
        path_pub.publish(path_msg);
        ROS_INFO("Publishing global path...");
        rate.sleep();
    }

    return 0;
}
