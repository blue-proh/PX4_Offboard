#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //无人机目标位置
#include <geometry_msgs/TwistStamped.h> //无人机目标速度
#include <mavros_msgs/CommandBool.h> //无人机解锁指令
#include <mavros_msgs/SetMode.h> //设置无人机飞行模式
#include <mavros_msgs/State.h> //无人机当前状态
#include <nav_msgs/Odometry.h> //无人机当前位置
#include <darknet_ros_msgs/ObjectCount.h> //调用darknet_ros功能包中的消息
#include <darknet_ros_msgs/BoundingBoxes.h>

#define image_width 320
#define image_height 240

mavros_msgs::State current_state;
nav_msgs::Odometry current_position;
darknet_ros_msgs::ObjectCount current_ObjectCount;
darknet_ros_msgs::BoundingBoxes current_BoundingBoxes;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void position_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_position = *msg;
}

void ObjectCount_cb(const darknet_ros_msgs::ObjectCount::ConstPtr& msg){
    current_ObjectCount = *msg;
}

void BoundingBoxes_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
    current_BoundingBoxes = *msg;
}

int main(int argc, char **argv)
{
    unsigned int Find_Object_Flag = 0; //找到目标的标志位
    double Cam_xmin = 0;
    double Cam_ymin = 0;
    double Cam_xmax = 0;
    double Cam_ymax = 0;
    double Cam_x = 0; //相机坐标系
    double Cam_y = 0;
    double Target_x = 0; //地球坐标系
    double Target_y = 0;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //订阅无人机状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
          ("mavros/state", 10, state_cb);

    //发布无人机目标位置
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    //发布无人机目标速度
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);

    //客户端申请无人机解锁服务
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    //客户端申请无人机模式设置服务
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //订阅无人机当前位置
    ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/global_position/local", 10, position_cb);

    //订阅无人机检测到物体的数量
    ros::Subscriber ObjectCount_sub = nh.subscribe<darknet_ros_msgs::ObjectCount>
            ("darknet_ros/found_object", 10, ObjectCount_cb);

    //订阅无人机检测到物体的位置参数
    ros::Subscriber BoundingBoxes_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>
            ("darknet_ros/bounding_boxes", 30, BoundingBoxes_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ROS_INFO("ROS and Mavros connect Wait");
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("ROS and Mavros OK");

    geometry_msgs::PoseStamped pose;//位置控制
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2.0;

    geometry_msgs::TwistStamped vel;//速度控制

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    //无人机起飞至指定高度
    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
        }
        else
        {
            if(ros::Time::now() - last_request > ros::Duration(3.0) && current_position.pose.pose.position.z > 1.95)
                break;

        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    //无人机寻找着陆目标
    last_request = ros::Time::now();
    while(ros::ok())
    {
        if(current_ObjectCount.count == 1) //找到目标
        {
            Find_Object_Flag = 1;
            ROS_INFO("Find Object");
        }

        if(current_position.pose.pose.position.x > 5.00)
        {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            set_mode_client.call(offb_set_mode);
            ROS_INFO("Call for LAND");
            vel.twist.linear.x = 0;
            vel.twist.linear.y = 0;
            vel.twist.linear.z = 0;
        }

        else if(Find_Object_Flag)
        {
            Cam_xmin = current_BoundingBoxes.bounding_boxes[0].xmin;
            Cam_ymin = current_BoundingBoxes.bounding_boxes[0].ymin;
            Cam_xmax = current_BoundingBoxes.bounding_boxes[0].xmax;
            Cam_ymax = current_BoundingBoxes.bounding_boxes[0].ymax;

            //计算目标在相机坐标系的位置
            Cam_x = (Cam_xmin + Cam_xmax) / 2;
            Cam_y = (Cam_ymin + Cam_ymax) / 2;

            Target_x = Cam_y;
            Target_y = Cam_x;
            ROS_INFO_STREAM("Target_x = " << Target_x << ", " << "Target_y = " << Target_y);
            //计算两方向err
            double err_x = image_height / 2.0 - Target_x;
            double err_y = image_width / 2.0 - Target_y;
            ROS_INFO_STREAM("err_x = "<< err_x <<", "<< "err_y = " << err_y);
            //速度控制
            vel.twist.linear.x = err_x/400;
            vel.twist.linear.y = err_y/400;

            //如果位置很正开始降落
            if(err_x < 10 && err_y < 10)
                vel.twist.linear.z = -0.2;
            else
                vel.twist.linear.z = 0;

            //高度低于0.3时转为降落模式
            if(current_position.pose.pose.position.z < 0.3)
                break;

        }
        else{
            vel.twist.linear.x = 0.5;
            vel.twist.linear.y = 0;
            vel.twist.linear.z = 0;
        }
        local_vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("Offboard enabled");
        last_request = ros::Time::now();
    }

    return 0;
}