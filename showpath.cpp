#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h> //无人机当前位置
#include <tf/tf.h>
#include <fstream>
#include <iostream>
/************************************************************************************
 *本程序用于无人机测试无人机在rviz中显示运动轨迹
 *Li Zhiheng
 *2021.03.28
 *************************************************************************************/
using namespace std;
nav_msgs::Odometry current_position;
void position_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_position = *msg;
}

main (int argc, char **argv)
{
    ofstream outfile;   //输出流

    ros::init (argc, argv, "showpath");

    ros::NodeHandle ph;
    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory",1, true);

    //订阅无人机当前位置
    ros::Subscriber position_sub = ph.subscribe<nav_msgs::Odometry>
            ("mavros/global_position/local", 10, position_cb);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    nav_msgs::Path path;
    path.header.stamp=current_time;
    path.header.frame_id="odom";

    ros::Rate loop_rate(50);

    outfile.open("pos_log.txt", ios::app);   //每次写都定位的文件结尾，不会丢失原来的内容，用out则会丢失原来的内容
    if(!outfile.is_open ())
        cout << "Open file failure" << endl;

    while (ros::ok())
    {

        current_time = ros::Time::now();

        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = current_position.pose.pose.position.x;
        this_pose_stamped.pose.position.y = current_position.pose.pose.position.y;
        this_pose_stamped.pose.position.z = current_position.pose.pose.position.z;
        outfile << current_position.pose.pose.position.x << "," << current_position.pose.pose.position.y << "," << current_position.pose.pose.position.z << endl;  //在result.txt中写入结果

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="odom";
        path.poses.push_back(this_pose_stamped);

        path_pub.publish(path);
        ros::spinOnce();               // check for incoming messages

        last_time = current_time;
        loop_rate.sleep();
    }

    return 0;
}

