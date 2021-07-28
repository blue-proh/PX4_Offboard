#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //无人机目标位置
#include <mavros_msgs/CommandBool.h> //无人机解锁指令
#include <mavros_msgs/SetMode.h> //设置无人机飞行模式
#include <mavros_msgs/State.h> //无人机当前状态
#include <nav_msgs/Odometry.h> //无人机当前位置
/************************************************************************************
 *本程序用于无人机OFFBOARD模式下的起降测试
 *Li Zhiheng
 *2021.03.28
 *************************************************************************************/
mavros_msgs::State current_state;//
nav_msgs::Odometry current_position;
geometry_msgs::PoseStamped target_pose;//设置目标高度

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void position_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_position = *msg;
}

/******************目标位置设定**********************/
void Set_TargetPoint(float x, float y, float z) {
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
}

int main(int argc, char **argv) {
    unsigned int UAV_Reach_H_Flag = 0;
    unsigned int Task_step = 0;
    unsigned int Cruise_step = 1; //巡航飞行阶段，七边飞行
    unsigned int Now_Cruise_step = 1;
    unsigned int Last_Cruise_step = 1;

    ros::init(argc, argv, "square_flight_gazebo_node");
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

    //客户端申请无人机解锁服务
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    //客户端申请无人机模式设置服务
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

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

    target_pose.pose.position.x = 0;
    target_pose.pose.position.y = 0;
    target_pose.pose.position.z = 0.5;

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
        }
        //无人机起飞与着陆
        if (Task_step == 1) {
            if (current_state.armed && !UAV_Reach_H_Flag && current_position.pose.pose.position.z > 0.45) {
                ROS_INFO("UAV reach target height");
                UAV_Reach_H_Flag = 1; //无人机达到预定高度
                last_request = ros::Time::now();
            }
            if (UAV_Reach_H_Flag && ros::Time::now() - last_request > ros::Duration(3.0) &&
                current_state.mode == "OFFBOARD")
                Task_step = 2;
        }
        //无人机巡航
        if (Task_step == 2)
        {
            Last_Cruise_step = Now_Cruise_step;
            switch(Cruise_step) {
                case 1:
                    Set_TargetPoint(1.0, 0.0, 0.5);
                    if (current_position.pose.pose.position.x > 0.95) {
                        ROS_INFO("Reach Cruise point 1");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                    break;
                case 2:
                    Set_TargetPoint(1.0, 1.0, 0.5);
                    if (current_position.pose.pose.position.y > 0.95) {
                        ROS_INFO("Reach Cruise point 2");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                    break;
                case 3:
                    Set_TargetPoint(0.0, 1.0, 0.5);
                    if (current_position.pose.pose.position.x < 0.05) {
                        ROS_INFO("Reach Cruise point 3");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                    break;
                case 4:
                    Set_TargetPoint(0.0, 0.0, 0.5);
                    if (current_position.pose.pose.position.y < 0.05) {
                        ROS_INFO("Reach Cruise point 4");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 100;//无人机准备降落
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
        //land
        if(Task_step == 99) {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("AUTO.LAND enabled");
                last_request = ros::Time::now();
                Task_step = 100;
            }
        }
        //wait land
        if(Task_step == 100) {
        }

        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}



