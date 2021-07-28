#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //无人机目标位置
#include <mavros_msgs/CommandBool.h> //无人机解锁指令
#include <mavros_msgs/SetMode.h> //设置无人机飞行模式
#include <mavros_msgs/State.h> //无人机当前状态

/************************************************************************************
 *本程序用于无人机OFFBOARD模式下pixhawk解锁测试
 *Li Zhiheng
 *2021.03.28
 *************************************************************************************/
/************************************************************************************
机头方向朝前的情况下
无人机向上移动t265_vision_pose.pose.position.z++++++
无人机向前移动t265_vision_pose.pose.position.y++++++
无人机向右移动t265_vision_pose.pose.position.x++++++
*************************************************************************************/
mavros_msgs::State current_state;
geometry_msgs::PoseStamped t265_vision_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void t265_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    t265_vision_pose = *msg;
}

int main(int argc, char **argv)
{
    int Task_step = 0;
    ros::init(argc, argv, "takeoff_and_land_gazebo_node");
    ros::NodeHandle nh;

    //订阅无人机状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    //订阅t265定位数据
    ros::Subscriber t265_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/vision_pose/pose", 20, t265_pose_cb);

    //发布无人机目标位置
    ros::Publisher target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    //客户端申请无人机解锁服务
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    //客户端申请无人机模式设置服务
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    ROS_INFO("ROS Wait");
    // wait for FCU connection
    while(!ros::ok() && !current_state.connected){
        if(!ros::ok()) ROS_INFO("ROS Error");
        if(!current_state.connected) ROS_INFO("Connected Error");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("ROS OK");

    geometry_msgs::PoseStamped target_pose;//设置目标高度
    target_pose.pose.position.x = 0;
    target_pose.pose.position.y = 0;
    target_pose.pose.position.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        target_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD"; //设置模式为offbord模式

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        //无人机模式选择与解锁
        if(Task_step == 0) {
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
                        Task_step = 1;
                    }
                    last_request = ros::Time::now();
                }
            }
        }
        //无人机模式上锁
        if(Task_step == 1)
        {
            if (current_state.armed && (ros::Time::now() - last_request > ros::Duration(10.0))) //如果目前没有解锁并且等待时间超过3s
            {
                arm_cmd.request.value = false;
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle disarmed");
                    Task_step = 2;
                }
                last_request = ros::Time::now();
            }
        }

        ROS_INFO("T265_pose_X: %f, T265_pose_Y: %f, T265_pose_Z: %f",t265_vision_pose.pose.position.x,t265_vision_pose.pose.position.y,t265_vision_pose.pose.position.z);

        target_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}