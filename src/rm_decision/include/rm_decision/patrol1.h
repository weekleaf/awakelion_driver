// #pragma once
// #include <behaviortree_cpp_v3/action_node.h>
// #include <ros/ros.h>
// #include "rm_msgs/GameStatus.h"
// #include "rm_msgs/GameResult.h"
// #include "rm_msgs/GameRobotHP.h"
// #include "rm_msgs/GameBuff.h"
// #include "rm_msgs/RobotStatus.h"
// #include "rm_msgs/RobotDamage.h"
// #include "rm_msgs/patrol_rx.h"

// namespace rm_decision  {

// class Patrol : public BT::SyncActionNode {
//     public:
//     Patrol(const std::string& name , const BT::NodeConfiguration& config 
//     ,const ros::NodeHandle& root_nh
//     ,const ros::NodeHandle& tree_nh) 
//     : BT::SyncActionNode(name, config)                      
//     {
//         ros::NodeHandle nh(root_nh);
//         patrol_sub_ = nh.subscribe("patrol_blackboard", 1, &Patrol::PatrolCallback, this);
//         publisher_ = nh.advertise<rm_msgs::gamestart_tx>("game_status_output", 10);
 
//    };

   
//     static BT::PortsList providedPorts() {
//         BT::PortsList ports_list;
//         return ports_list;
//     } 

//     BT::NodeStatus tick() override
//     {
//         ROS_INFO("Patrol");

        
//         if(patrol_.patrol_decision == 0)
//         {
            
//             return BT::NodeStatus::RUNNING;

//             std::cout << "运行中" << std::endl;

//             return BT::NodeStatus::SUCCESS;
//         }
//         return BT::NodeStatus::SUCCESS;
//     }

//     bool patrol_received_;

//     rm_msgs::patrol_rx patrol_;
//     ros::Subscriber patrol_sub_;
//     ros::Publisher publisher_;

//     private:
// void PatrolCallback(const boost::shared_ptr<const rm_msgs::patrol_rx>& msg)
// {
//     patrol_ = *msg;
//     patrol_received_ = true;
// }

// };

// }