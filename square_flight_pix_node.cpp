#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //无人机目标位置
#include <mavros_msgs/CommandBool.h> //无人机解锁指令
#include <mavros_msgs/SetMode.h> //设置无人机飞行模式
#include <mavros_msgs/State.h> //无人机当前状态

#define Target_Height 1.0 //无人机巡航高度m
/************************************************************************************
 *本程序用于无人机OFFBOARD模式下方框飞行的测试
 *Li Zhiheng
 *2021.03.28
 *************************************************************************************/
mavros_msgs::State current_state;//无人机状态
geometry_msgs::PoseStamped target_pose;//设置目标高度
geometry_msgs::PoseStamped t265_vision_pose;//T265位置信息

//无人机状态反馈
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
//无人机位置反馈
void t265_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    t265_vision_pose = *msg;
}

/******************目标位置设定**********************/
void Set_TargetPoint(float x, float y, float z) {
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
}

int main(int argc, char **argv) {
    unsigned int UAV_Reach_H_Flag = 0;//无人机达到目标高度标志位
    unsigned int Task_step = 0;//总体飞行阶段
    unsigned int Cruise_step = 1; //巡航飞行阶段，七边飞行
    unsigned int Now_Cruise_step = 1;//当前巡航阶段
    unsigned int Last_Cruise_step = 1;//上一个巡航阶段

    ros::init(argc, argv, "square_flight_pix_node");
    ros::NodeHandle nh;

    //订阅无人机状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    //订阅t265定位数据
    ros::Subscriber t265_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/vision_pose/pose", 20, t265_pose_cb);

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

    //设置目标位置
    Set_TargetPoint(0.0, 0.0, Target_Height);

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD"; //设置模式为offbord模式

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;//设置解锁命令

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
            if (current_state.armed && !UAV_Reach_H_Flag && t265_vision_pose.pose.position.z > Target_Height - 0.05) {
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
                    Set_TargetPoint(1.0, 0.0, Target_Height);
                    if (t265_vision_pose.pose.position.x > 0.95) {
                        ROS_INFO("Reach Cruise point 1");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                    break;
                case 2:
                    Set_TargetPoint(1.0, 1.0, Target_Height);
                    if (t265_vision_pose.pose.position.y > 0.95) {
                        ROS_INFO("Reach Cruise point 2");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                    break;
                case 3:
                    Set_TargetPoint(0.0, 1.0, Target_Height);
                    if (t265_vision_pose.pose.position.x < 0.05) {
                        ROS_INFO("Reach Cruise point 3");
                        Now_Cruise_step = Cruise_step;
                        last_request = ros::Time::now();
                        Cruise_step = 99;
                    }
                    break;
                case 4:
                    Set_TargetPoint(0.0, 0.0, Target_Height);
                    if (t265_vision_pose.pose.position.y < 0.05) {
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



