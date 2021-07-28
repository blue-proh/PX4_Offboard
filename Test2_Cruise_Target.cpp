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
#define Target_Height 1.0 //无人机巡航高度m
/************************************************************************************
 *本程序用于无人机OFFBOARD模式下在Gazebo中按一定路径搜索地面目标
 *Li Zhiheng
 *2021.03.28
 *************************************************************************************/
mavros_msgs::State current_state;
nav_msgs::Odometry current_position;
geometry_msgs::PoseStamped target_pose;//设置目标高度
geometry_msgs::TwistStamped vel;//速度控制
darknet_ros_msgs::ObjectCount current_ObjectCount;
darknet_ros_msgs::BoundingBoxes current_BoundingBoxes;

/************************反馈************************/
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
/******************目标速度位置设定**********************/
void Set_TargetPoint(float x, float y, float z) {
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
}
void Set_TargetVel(float x, float y, float z) {
    vel.twist.linear.x = x;
    vel.twist.linear.y = y;
    vel.twist.linear.z = z;
}



int main(int argc, char **argv) {
    unsigned int UAV_Reach_H_Flag = 0;
    unsigned int Task_step = 0;
    unsigned int Cruise_step = 1; //巡航飞行阶段，七边飞行
    unsigned int Now_Cruise_step = 1;
    unsigned int Last_Cruise_step = 1;
    /************目标检测变量************/
    unsigned int Find_Object_Flag = 0; //找到目标的标志位
    double Cam_xmin = 0;
    double Cam_ymin = 0;
    double Cam_xmax = 0;
    double Cam_ymax = 0;
    double Cam_x = 0; //相机坐标系
    double Cam_y = 0;
    double Target_x = 0; //地球坐标系
    double Target_y = 0;
    ros::init(argc, argv, "Test2_Cruise_Target");
    ros::NodeHandle nh;

    //订阅无人机状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    //订阅无人机当前位置
    ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/global_position/local", 10, position_cb);

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
    while (!ros::ok() && !current_state.connected) {
        if (!ros::ok()) ROS_INFO("ROS Error");
        if (!current_state.connected) ROS_INFO("Connected Error");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("ROS and Mavros OK");

    Set_TargetPoint(0, 0, 1.0);

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD"; //设置模式为offbord模式

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        //无人机模式选择与解锁
        if (Task_step == 0) {
            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(3.0))) //如果目前不是OffBoard模式并且等待时间超过3s
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) //则设置模式为offbord模式
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))) //如果目前没有解锁并且等待时间超过3s
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                        ROS_INFO("You can change model!!!!!!!");
                        Task_step = 1;
                    }
                    last_request = ros::Time::now();
                }
            }

            local_pos_pub.publish(target_pose);
        }
        //无人机起飞
        if (Task_step == 1) {
            if (current_state.armed && !UAV_Reach_H_Flag && current_position.pose.pose.position.z > Target_Height - 0.05) {
                ROS_INFO("UAV reach target height");
                UAV_Reach_H_Flag = 1; //无人机达到预定高度
                last_request = ros::Time::now();
            }
            if (UAV_Reach_H_Flag && ros::Time::now() - last_request > ros::Duration(3.0) &&
                current_state.mode == "OFFBOARD")
                Task_step = 2;

            local_pos_pub.publish(target_pose);
        }
        //无人机巡航
        if (Task_step == 2)
        {
            if(current_ObjectCount.count >= 1) //无人机在巡航的过程中找到目标
            {
                Find_Object_Flag = 1;
                ROS_INFO("Find Object!!!!!!");
                Task_step = 3; //控制无人机悬停于目标上方
            }

            Last_Cruise_step = Now_Cruise_step;
            switch(Cruise_step) {
                case 1:
                    Set_TargetPoint(2.0, 0, Target_Height);
                    if (current_position.pose.pose.position.x > 1.95) {
                        ROS_INFO("Reach Cruise point 1");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                        break;
                case 2:
                    Set_TargetPoint(2.0, 2.0, Target_Height);
                    if (current_position.pose.pose.position.y > 1.95) {
                        ROS_INFO("Reach Cruise point 2");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                    break;
                case 3:
                    Set_TargetPoint(0.0, 2.0, Target_Height);
                    if (current_position.pose.pose.position.x < 0.05) {
                        ROS_INFO("Reach Cruise point 3");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                    break;
                case 4:
                    Set_TargetPoint(0.0, 0.5, Target_Height);
                    if (current_position.pose.pose.position.y < 0.55) {
                        ROS_INFO("Reach Cruise point 4");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                    break;
                case 5:
                    Set_TargetPoint(1.5, 0.5, Target_Height);
                    if (current_position.pose.pose.position.x > 1.45) {
                        ROS_INFO("Reach Cruise point 5");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                    break;
                case 6:
                    Set_TargetPoint(1.5, 1.5, Target_Height);
                    if (current_position.pose.pose.position.y > 1.45) {
                        ROS_INFO("Reach Cruise point 6");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99; //无人机搜索完成但是未找到目标后自动降落
                    }
                    break;
                case 7:
                    Set_TargetPoint(0.5, 1.5, Target_Height);
                    if (current_position.pose.pose.position.x < 0.55) {
                        ROS_INFO("Reach Cruise point 7");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                    break;
                case 8:
                    Set_TargetPoint(0.5, 1.0, Target_Height);
                    if (current_position.pose.pose.position.y < 1.05) {
                        ROS_INFO("Reach Cruise point 8");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                    break;
                case 9:
                    Set_TargetPoint(1.0, 1.0, Target_Height);
                    if (current_position.pose.pose.position.x > 0.95) {
                        ROS_INFO("Reach Cruise point 9");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 100;
                    }
                    break;
                case 99: //等待阶段
                    if(ros::Time::now() - last_request > ros::Duration(3.0))
                        Cruise_step = Last_Cruise_step + 1;
                    break;
                case 100: //无人机准备降落
                    Task_step = 99;
                    break;
            }
            local_pos_pub.publish(target_pose);

        }
        // 无人机目标检测
        if (Task_step == 3) {
            Cam_xmin = current_BoundingBoxes.bounding_boxes[0].xmin;
            Cam_ymin = current_BoundingBoxes.bounding_boxes[0].ymin;
            Cam_xmax = current_BoundingBoxes.bounding_boxes[0].xmax;
            Cam_ymax = current_BoundingBoxes.bounding_boxes[0].ymax;

            //计算目标在相机坐标系的位置
            Cam_x = (Cam_xmin + Cam_xmax) / 2;
            Cam_y = (Cam_ymin + Cam_ymax) / 2;
            ROS_INFO_STREAM("Cam_x = " << Cam_x << ", " << "Cam_y = " << Cam_y);

            Target_x = Cam_y;
            Target_y = Cam_x;

            //计算两方向err
            double err_x = image_height / 2.0 - Target_x;
            double err_y = image_width / 2.0 - Target_y;
            //速度控制
            vel.twist.linear.x = err_x/400;
            vel.twist.linear.y = err_y/400;
            //如果位置很正开始降落
            if(err_x < 10 && err_y < 10)
                vel.twist.linear.z = -0.2;
            else
                vel.twist.linear.z = 0;

            //高度低于0.3时转为降落模式
            if(current_position.pose.pose.position.z < 0.5)
                Task_step = 99;

            local_vel_pub.publish(vel);
        }
        //无人机降落
        if (Task_step == 99) {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("AUTO.LAND enabled");
                last_request = ros::Time::now();
                break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}

