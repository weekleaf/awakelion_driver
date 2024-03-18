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

ros::Publisher odomPub;
serialPort serial_handle;
vision_tx_data pc_recv_mesg;

using namespace ly;

void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
   pc_recv_mesg.navigation_determine = 1;
   pc_recv_mesg.linear_x = msg->linear.x;
   pc_recv_mesg.linear_y = msg->linear.y;
   pc_recv_mesg.angle_w  = msg->angular.z;
   pc_recv_mesg.visual_valid = 0;
}
int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh("~");

    nh.param<std::string>("serial_name", serial_handle.name, "/dev/ttyACM0"); ///dev/ttyUSB0
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1, cmdCallback);
    ros::Publisher vision_rx_data_pub = nh.advertise<robot_driver::vision_rx_data>("/vision_rx_data", 1);
    ROS_INFO("pc_recv_mesg.visual_valid=%u",pc_recv_mesg.visual_valid);
    robot_driver::vision_rx_data pc_send_bag;
    vision_rx_data pc_send_mesg;

    serial_handle.init();
    ros::Rate loop_rate(100);
    std::cout << "listener thread start" << std::endl;
    while (ros::ok()) {
        try {
            serial_handle.writeData(pc_recv_mesg);     //发送数据
            serial_handle.readData(pc_send_mesg);      //接收数据
            ROS_INFO("LINEAR.X = %f",pc_recv_mesg.linear_x);
            ROS_INFO("LINEAR.Y = %f",pc_recv_mesg.linear_y);
            ROS_INFO("ANGLE.Z = %f",pc_recv_mesg.angle_w);
            ROS_INFO("DEDAOD PITCH =%f",pc_send_mesg.robot_pitch);
            ROS_INFO("DEDAOD YAW =%f\n\n",pc_send_mesg.robot_yaw);

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
            // ROS_INFO("发送的pit消息为%f",pc_send_bag.robot_pitch);
            // ROS_INFO("发送的yaw消息为%f",pc_send_bag.robot_yaw);
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
