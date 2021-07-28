#include <ros/ros.h>
#include <k210_target/BoundingBox.h> //地面目标位置
k210_target::BoundingBox Target_pose;//地面目标位置

// 接收到订阅的消息后，会进入消息回调函数
void Target_pose_cb(const k210_target::BoundingBox::ConstPtr& msg)
{
// 将接收到的消息打印出来
    Target_pose = *msg;
}

int main(int argc, char **argv) {
// 初始化ROS节点
    ros::init(argc, argv, "K210_listener");

// 创建节点句柄
    ros::NodeHandle nh;

    //订阅地面目标位置
    ros::Subscriber Target_pose_sub = nh.subscribe<k210_target::BoundingBox>
            ("k210_detection_msgs", 100, Target_pose_cb);
    ros::Rate rate(20.0);
    while(1) {
        ROS_INFO_STREAM("center_x = " << Target_pose.center_x << ", " << "center_y = " << Target_pose.center_y << ", " << "wall_time = " << Target_pose.wall_time);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}