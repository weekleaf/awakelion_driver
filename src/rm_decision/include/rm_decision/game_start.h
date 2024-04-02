#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include "rm_msgs/GameStatus.h"
#include "rm_msgs/GameResult.h"
#include "rm_msgs/GameRobotHP.h"
#include "rm_msgs/GameBuff.h"
#include "rm_msgs/RobotStatus.h"
#include "rm_msgs/RobotDamage.h"
#include "rm_msgs/gamestart_tx.h"

namespace rm_decision  {

class GameStart : public BT::SyncActionNode {
public:
    GameStart(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)                      
    {
        ros::NodeHandle nh(root_nh);
        game_status_sub_ = nh.subscribe("game_status_blackboard", 1, &GameStart::GameStatusCallback, this);
        game_status_pub_ = nh.advertise<rm_msgs::gamestart_tx>("game_status_output", 10);
    };

    static BT::PortsList providedPorts() {
        BT::PortsList ports_list;
        return ports_list;
    }
    void reset()
    {        //重置该值
        game_status_received_ = false;
        game_status_ = rm_msgs::GameStatus();
    }

    void publish(bool game_status_received_)
    {
        rm_msgs::gamestart_tx msg;
        msg.gamestart = game_status_received_;
        ROS_INFO("msg.gamestart is %s",msg.gamestart ? "true" : "false");
        ROS_INFO("game_status_received_ is %s",game_status_received_ ? "true" : "false");
        game_status_pub_.publish(msg);
    }

    ~GameStart()
    {   

    }

    BT::NodeStatus tick() override
    {

        //对获得的值进行处理
        if(game_status_.game_progress == 4 && game_status_.stage_remain_time >= 0)
        {
            std::cout << "false!" << std::endl;
            publish(game_status_received_);
            reset();
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            std::cout << "yes!" << std::endl;
            reset();
            return BT::NodeStatus::SUCCESS;
        }
    }
    bool game_status_received_;

    rm_msgs::GameStatus game_status_;
    ros::Subscriber game_status_sub_;
    ros::Publisher game_status_pub_;


private:
    void GameStatusCallback(const boost::shared_ptr<const rm_msgs::GameStatus>& msg) // const rm_msgs::GameStatus::ConstPtr& msg
    {
        std::cout << "成功gamestart" << std::endl;
        game_status_ = *msg;
        game_status_received_ = true;
    }

        };
}