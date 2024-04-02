#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include "rm_msgs/GameStatus.h"
#include "rm_msgs/GameResult.h"
#include "rm_msgs/GameRobotHP.h"
#include "rm_msgs/GameBuff.h"
#include "rm_msgs/RobotStatus.h"
#include "rm_msgs/RobotDamage.h"

namespace rm_decision  {

class AttackBase : public BT::SyncActionNode {
    public:
        AttackBase(const std::string& name , const BT::NodeConfiguration& config )  
        : BT::SyncActionNode(name, config) 
        {};

        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            return ports_list;
        } 

        BT::NodeStatus tick() override
        {

                return BT::NodeStatus::SUCCESS;

        }
    private:   

};
}