#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include "rm_msgs/attacksentry_rx.h"
#include "rm_msgs/attacksentry_tx.h"

namespace rm_decision  {

class AttackSentry : public BT::SyncActionNode {
    public:
    AttackSentry(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)                      
    {
        ros::NodeHandle nh(root_nh);
        ac = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
        attacksentry_sub_ = nh.subscribe("attack_sentry_blackboard", 1, &AttackSentry::AttackSentryCallback, this);
        attacksentry_pub_ = nh.advertise<rm_msgs::attacksentry_tx>("attack_sentry_output", 10);
    };

        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            ports_list.insert(BT::InputPort<uint8_t>("input"));
            ports_list.insert(BT::InputPort<uint8_t>("blood_output"));
            ports_list.insert(BT::OutputPort<bool>("output"));
            return ports_list;
        } 

        void sendGoal(double goal_x, double goal_y, double goal_yaw ){
            // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
            ROS_INFO("Waiting for the move_base action server");
            ac->waitForServer(ros::Duration(10));
            ROS_INFO("Connected to move base server");
            double x = goal_x, y = goal_y, z = 0.;

            geometry_msgs::PointStamped map_point;
            map_point.header.frame_id = "map";
            map_point.header.stamp = ros::Time();
            map_point.point.x = x;
            map_point.point.y = y;
            map_point.point.z = z;

            try
            {
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position = map_point.point;
                double roll = 0.0, pitch = 0.0, yaw = goal_yaw;
                geometry_msgs::Quaternion q;
                q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw); 
                goal.target_pose.pose.orientation = q;
        
                ROS_INFO("Sending goal");
                ac->sendGoal(goal,
                            boost::bind(&AttackSentry::doneCb, this, _1, _2),
                            boost::bind(&AttackSentry::activeCb, this),
                            boost::bind(&AttackSentry::feedbackCb, this, _1));
                // Wait for the action to return
                // ac->waitForResult();
                
                // if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                //     ROS_INFO("You have reached the goal!");
                // else
                //     ROS_INFO("The base failed for some reason");
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }


        BT::NodeStatus tick() override
        {
            attacksentry_tx.dodge_ctrl = 0;
            auto enemy_HP_Sentry = getInput<uint8_t>("input");
            auto addblood_need = getInput<uint8_t>("blood_output");
            // ROS_INFO("Addblood_need_ is %d",addblood_need);
            if(addblood_need == 1)
                addblood_need_ = 1;
            else addblood_need_ = 0;

            std::cout << "addblood_need_" <<  addblood_need_ <<std::endl;

            if(addblood_need_ || attacksentry_rx_.stage_remain_time <= 244)
            {
                return BT::NodeStatus::SUCCESS;
            }
            if(!addblood_need_ == 1 
                && have_chance_to_attack == 0 
                && attacksentry_rx_.stage_remain_time > 270)
            {
                sendGoal(patrol_points[0].x,patrol_points[0].y,0);
            }


            
            if((attacksentry_rx_.enemy_HP_Hero + attacksentry_rx_.enemy_HP_Infansty1 
                + attacksentry_rx_.enemy_HP_Infansty2)<= 150)
                have_chance_to_attack = 1;
            
            if(attacksentry_rx_.stage_remain_time <= 270 
                && attacksentry_rx_.stage_remain_time >= 244 )
            {   
                if(have_chance_to_attack == 1)
                {
                    sendGoal(patrol_points[1].x,patrol_points[1].y,0);
                }
                else
                {
                    sendGoal(patrol_points[2].x,patrol_points[2].y,0);
                }
            }
            actionlib::SimpleClientGoalState state = ac->getState();
            ROS_INFO("ATTACK SENTRY :%s",state.toString().c_str());
            if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                attacksentry_pub_.publish(attacksentry_tx);
                return BT::NodeStatus::FAILURE;
            }
            else 
            {
                attacksentry_pub_.publish(attacksentry_tx);
                return BT::NodeStatus::FAILURE;
            }
            
            return BT::NodeStatus::FAILURE;
        }
    uint8_t have_chance_to_attack = 0;
    uint8_t addblood_need_ = 0;
    bool attacksentry_rx_received_;
    rm_msgs::attacksentry_rx attacksentry_rx_;
    rm_msgs::attacksentry_tx attacksentry_tx;
    ros::Subscriber attacksentry_sub_;
    ros::Publisher attacksentry_pub_;
    private:
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac;
    move_base_msgs::MoveBaseGoal goal;
    struct patrol_point //固定巡逻点
    {
        double x;
        double y;
    };

    std::vector<patrol_point> patrol_points{
      {8.4,1.0},
      {8.45,2.0},
      {6.0,1.5},
    };

    void AttackSentryCallback(const boost::shared_ptr<const rm_msgs::attacksentry_rx>& msg)
    {
        attacksentry_rx_ = *msg;
        attacksentry_rx_received_ = true;
    }

    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
    {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal reached");
            attacksentry_tx.dodge_ctrl = 1;
        }
        else
        {
            ROS_INFO("Goal was not reached");
        }
        // 在此处可以添加完成时的逻辑处理
    }

    void activeCb()
    {
        ROS_INFO("Attack sentry goal is active");
            if(addblood_need_ || attacksentry_rx_.stage_remain_time <= 244)
            {
                ac->cancelAllGoals();
                // ac->waitForResult();
                actionlib::SimpleClientGoalState state = ac->getState();
                ROS_INFO("Attacksentry action state after cancel: %s", state.toString().c_str());
                ROS_INFO("即将返回补血点");
            }


        // 在此处可以添加目标激活时的逻辑处理
    }

    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
    {
        double current_distance = sqrt(pow(goal.target_pose.pose.position.x-feedback->base_position.pose.position.x, 2) + 
                                        pow(goal.target_pose.pose.position.y-feedback->base_position.pose.position.y, 2));
        // 如果距离小于或等于0.5米，发送 dodge_ctrl = 1 的标志位
        if (current_distance <= 0.5)
        {
            ROS_INFO("Distance to goal is less than or equal to 0.5 meters. Sending dodge_ctrl = 1.");
            attacksentry_tx.dodge_ctrl = 1;
        }
        // 在此处可以添加接收到反馈时的逻辑处理
    }


};
    
}