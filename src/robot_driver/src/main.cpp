//
// Created by bismarck on 12/11/22.
//
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <robot_driver/serialPort.h>

#include "robot_driver/MYAPI.h"
#include "robot_driver/serialPort.h"
#include "robot_driver/vision_tx_data.h"
#include "robot_driver/msg_serialize.h"
#include "robot_driver/vision_rx_data.h"
#include "rm_msgs/GameStatus.h"
#include "rm_msgs/addblood_rx.h"
#include "rm_msgs/addblood_tx.h"
#include "rm_msgs/attacksentry_rx.h"
#include "rm_msgs/attacksentry_tx.h"
#include "rm_msgs/acessbuff_rx.h"
#include "rm_msgs/acessbuff_tx.h"
#include "rm_msgs/dense_rx.h"
#include "rm_msgs/dense_tx.h"

using namespace ly;

ros::Publisher odomPub;
serialPort serial_handle;
vision_tx_data pc_recv_mesg;
bool dodge_ctrl1;
bool dodge_ctrl2;
bool dodge_ctrl3;
bool dodge_ctrl4;

void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
   pc_recv_mesg.navigation_determine = 1;
   pc_recv_mesg.linear_x = msg->linear.x;
   pc_recv_mesg.linear_y = msg->linear.y;
   pc_recv_mesg.angle_w  = msg->angular.z;
}
void visionCallback(const robot_driver::vision_tx_data::ConstPtr &msg){
    pc_recv_mesg.aim_pitch = msg->aim_pitch;
    pc_recv_mesg.aim_yaw = msg->aim_yaw;
    pc_recv_mesg.shoot_valid = msg->shoot_valid;
    pc_recv_mesg.visual_valid = msg->visual_valid;
    pc_recv_mesg.task_mode = msg->task_mode;
}
void DenseCallback(const rm_msgs::dense_tx::ConstPtr &msg){   
    dodge_ctrl1 = msg->dodge_ctrl;
}
void AcessCallback(const rm_msgs::acessbuff_tx::ConstPtr &msg){
    dodge_ctrl2 = msg->dodge_ctrl;
}
void AttackSentry(const rm_msgs::acessbuff_tx::ConstPtr &msg){
    dodge_ctrl3 = msg->dodge_ctrl;
}
void Addblood(const rm_msgs::addblood_tx::ConstPtr &msg){
    dodge_ctrl4 = msg->dodge_ctrl;
} 

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh("~");

    nh.param<std::string>("serial_name", serial_handle.name, "/dev/ttyACM0"); ///dev/ttyUSB0
    ros::Subscriber sub1 = nh.subscribe("/cmd_vel", 1, cmdCallback);
    ros::Subscriber sub2 = nh.subscribe("/vision_tx_data", 1, visionCallback);
    ros::Subscriber sub3 = nh.subscribe("/dense_output", 1, DenseCallback);
    ros::Subscriber sub4 = nh.subscribe("/acessbuff_output", 1, AcessCallback);
    ros::Subscriber sub5 = nh.subscribe("/attack_sentry_output", 1, AttackSentry);
    ros::Subscriber sub6 = nh.subscribe("/add_blood_output", 1, Addblood);

    ros::Publisher vision_rx_data_pub = nh.advertise<robot_driver::vision_rx_data>("/vision_rx_data", 1);
    ros::Publisher game_status_pub_ = nh.advertise<rm_msgs::GameStatus>("/game_status_blackboard", 1);
    ros::Publisher addblood_pub = nh.advertise<rm_msgs::addblood_rx>("/add_blood_blackboard",1);
    ros::Publisher attacksentry_pub = nh.advertise<rm_msgs::attacksentry_rx>("/attack_sentry_blackboard",1);
    ros::Publisher acessbuff_pub = nh.advertise<rm_msgs::acessbuff_rx>("/acessbuff_blackboard",1);
    ros::Publisher dense_pub = nh.advertise<rm_msgs::dense_rx>("/dense_blackboard",1);


    ROS_INFO("pc_recv_mesg.visual_valid=%u",pc_recv_mesg.visual_valid);
    robot_driver::vision_rx_data pc_send_bag;
    rm_msgs::GameStatus GameStatus_;
    rm_msgs::addblood_rx addblood_rx_;
    rm_msgs::attacksentry_rx attacksentry_rx_;
    rm_msgs::acessbuff_rx acessbuff_rx_;
    rm_msgs::dense_rx dense_rx_;

    vision_rx_data pc_send_mesg;
    game_robot_HP_t    game_robot_HP_;
    robot_judge1_data_t robot_judge1_data_;

    serial_handle.init();

    ros::Rate loop_rate(100);
    std::cout << "listener thread start" << std::endl;
    while (ros::ok()) {
        try {
            if(dodge_ctrl1 || dodge_ctrl2 || dodge_ctrl3 || dodge_ctrl4)
            {
                pc_recv_mesg.dodge_ctrl = 1;

            }
            else pc_recv_mesg.dodge_ctrl = 0;
            dodge_ctrl1 = 0;
            dodge_ctrl2 = 0;
            dodge_ctrl3 = 0;
            dodge_ctrl4 = 0;
            serial_handle.writeData(pc_recv_mesg);     //发送数据
            serial_handle.readData(pc_send_mesg,robot_judge1_data_,game_robot_HP_);      //接收数据
            ROS_INFO("LINEAR.X = %f",pc_recv_mesg.linear_x);
            ROS_INFO("LINEAR.Y = %f",pc_recv_mesg.linear_y);
            ROS_INFO("ANGLE.Z = %f\n",pc_recv_mesg.angle_w);

            ROS_INFO("ROBOT_ID = %u",robot_judge1_data_.robot_id);
            
            ROS_INFO("ROBOT_HP = %u",game_robot_HP_.red_7_robot_HP);
            ROS_INFO("发送的pit消息为%f",pc_send_bag.robot_pitch);
            ROS_INFO("发送的yaw消息为%f\n",pc_send_bag.robot_yaw);


            serial_handle.JudgeDate_Processing(robot_judge1_data_,game_robot_HP_);
            pc_recv_mesg.navigation_determine = 0;
            pc_send_bag.robot_color=pc_send_mesg.armors_Union.info.robot_color;
            pc_send_bag.task_mode=pc_send_mesg.armors_Union.info.task_mode;
            pc_send_bag.visual_valid=pc_send_mesg.armors_Union.info.visual_valid;
            pc_send_bag.direction=pc_send_mesg.armors_Union.info.direction;
            pc_send_bag.bullet_level=pc_send_mesg.armors_Union.info.bullet_level;
            pc_send_bag.robot_pitch=pc_send_mesg.robot_pitch;
            pc_send_bag.robot_yaw=pc_send_mesg.robot_yaw;
            pc_send_bag.time_stamp=pc_send_mesg.time_stamp;
            vision_rx_data_pub.publish(pc_send_bag);

            GameStatus_.game_progress=robot_judge1_data_.game_progress;
            GameStatus_.stage_remain_time=robot_judge1_data_.stage_remain_time;
            GameStatus_.game_type=robot_judge1_data_.game_type;
            game_status_pub_.publish(GameStatus_);

            addblood_rx_.current_HP=robot_judge1_data_.current_HP;
            addblood_rx_.maximum_HP=robot_judge1_data_.maximum_HP;
            addblood_rx_.game_progress=robot_judge1_data_.game_progress;
            addblood_rx_.stage_remain_time=robot_judge1_data_.stage_remain_time;
            addblood_rx_.robot_id=robot_judge1_data_.robot_id;
            addblood_rx_.enemy_HP_Base=serial_handle.enemy_HP_Base;
            addblood_rx_.enemy_HP_Hero=serial_handle.enemy_HP_Hero;
            addblood_rx_.enemy_HP_Infansty1=serial_handle.enemy_HP_Infansty1;
            addblood_rx_.enemy_HP_Infansty2=serial_handle.enemy_HP_Infansty2;
            addblood_rx_.enemy_HP_Sentry=serial_handle.enemy_HP_Sentry;
            addblood_rx_.projectile_allowance_17mm = robot_judge1_data_.projectile_allowance_17mm;
            addblood_pub.publish(addblood_rx_);

            attacksentry_rx_.current_HP=robot_judge1_data_.current_HP;
            attacksentry_rx_.enemy_HP_Hero=serial_handle.enemy_HP_Hero;
            attacksentry_rx_.enemy_HP_Infansty1=serial_handle.enemy_HP_Infansty1;
            attacksentry_rx_.enemy_HP_Infansty2=serial_handle.enemy_HP_Infansty2;
            attacksentry_rx_.enemy_HP_Sentry=serial_handle.enemy_HP_Sentry;
            attacksentry_rx_.stage_remain_time=robot_judge1_data_.stage_remain_time;
            attacksentry_pub.publish(attacksentry_rx_);

            acessbuff_rx_.center_activate=robot_judge1_data_.center_activate;
            acessbuff_rx_.stage_remain_time=robot_judge1_data_.stage_remain_time;
            acessbuff_pub.publish(acessbuff_rx_);

            dense_rx_.stage_remain_time=robot_judge1_data_.stage_remain_time;
            dense_pub.publish(dense_rx_);

        } 
        catch (serial::IOException &e) {
            serial_handle.init();
            std::cout << "serial read error" << std::endl;
            continue;
        }
        ros::spinOnce();
        loop_rate.sleep();
}
    return 0;
}
