#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include "rm_msgs/addblood_rx.h"
#include "rm_msgs/addblood_tx.h"    

namespace rm_decision  {

class AddBlood : public BT::SyncActionNode {
public:
    AddBlood(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)                      
    {
        ros::NodeHandle nh(root_nh);
        ac = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
        addblood_sub_ = nh.subscribe("add_blood_blackboard", 1, &AddBlood::AddBloodCallback, this);
        addblood_pub_ = nh.advertise<rm_msgs::addblood_tx>("add_blood_output", 10);

    };

    static BT::PortsList providedPorts() {
        BT::PortsList ports_list;
        ports_list.insert(BT::OutputPort<uint8_t>("output"));
        ports_list.insert(BT::OutputPort<uint8_t>("status"));
        // ports_list.insert(BT::OutputPort<uint8_t>("bullet_status"));
        return ports_list;
    } 
        void reset()
    {        //重置该值

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
                            boost::bind(&AddBlood::doneCb, this, _1, _2),
                            boost::bind(&AddBlood::activeCb, this),
                            boost::bind(&AddBlood::feedbackCb, this, _1));
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
        std::cout << "addblood battery" << std::endl;
        setOutput("output",addblood_rx_.enemy_HP_Sentry);
        //1.正常模式下，如果小于200血就去补给站补血
        //2.比赛离结束还有10s时，如果小于400就去补血
        //3.比赛离结束还有10s时，一旦我方哨兵血量小于敌方的就去补血
        // if(addblood_rx_.projectile_allowance_17mm == 0 )
        // {
        //     if(addblood_rx_.current_HP <= (addblood_rx_.maximum_HP - 200) 
        //     || (addblood_rx_.current_HP <= (addblood_rx_.maximum_HP - 100)
        //     && addblood_rx_.stage_remain_time <= 10)
        //     || (addblood_rx_.current_HP <= addblood_rx_.enemy_HP_Sentry && addblood_rx_.stage_remain_time <= 10)
        //     )
        //     {
        //         addblood_need = 1;
        //         sendGoal(patrol_points[0].x,patrol_points[0].y,0);
        //         std::cout << "Need to add blood!" << std::endl;
        //         setOutput("status",addblood_need);
        //     }
        //     else
        //     {
        //         bullet_need = 1;
        //         sendGoal(patrol_points[1].x,patrol_points[1].y,0);
        //         std::cout << "Protected home!" << std::endl;
        //         setOutput("bullet_status",bullet_need); 
        //         return BT::NodeStatus::FAILURE;             
        //     }

        // }
        if(addblood_need == 0)
        {
            if(addblood_rx_.current_HP <= (addblood_rx_.maximum_HP - 200) 
            || (addblood_rx_.current_HP <= (addblood_rx_.maximum_HP - 100)
            && addblood_rx_.stage_remain_time <= 10)
            || (addblood_rx_.current_HP <= addblood_rx_.enemy_HP_Sentry && addblood_rx_.stage_remain_time <= 10)
            )
            {
                addblood_need = 1;
                sendGoal(patrol_points[0].x,patrol_points[0].y,0);
                std::cout << "Need to add blood!" << std::endl;
                setOutput("status",addblood_need);
                return BT::NodeStatus::SUCCESS;
            }
            else
            {  
                std::cout << "Don't need to add blood!" << std::endl;   
                std::cout << "addblood_need ====" << addblood_need << std::endl;   
                setOutput("status",addblood_need);
                return BT::NodeStatus::SUCCESS;
            }
        }

        if(addblood_need == 1)
        {
            if(addblood_rx_.current_HP == 600)
            {
                addblood_need = 0;
                std::cout << "Add success!" << std::endl;
                setOutput("status",addblood_need);
                addblood_tx_.dodge_ctrl = 0;
                addblood_pub_.publish(addblood_tx_);
                return BT::NodeStatus::SUCCESS;
            }
            //假如未到达识别点
            // else if (rmul_supply_station == 0)
            // {
            //     ROS_INFO("Adding failure!");
            //     sendGoal(patrol_points[1].x,patrol_points[1].y,0);
            //        return BT::NodeStatus::FAILURE;
            // }
            else
            {
                std::cout << "Adding!" << std::endl;
                setOutput("status",addblood_need);
                return BT::NodeStatus::FAILURE;
            } 
            std::cout << "addblood battery2" << std::endl;
        }
        return BT::NodeStatus::FAILURE;
}
    //各个节点的任务确认标志位
    uint8_t addblood_need = 0;
    bool bullet_need;
    bool addblood_rx_received_;
    bool rmul_supply_station;

    struct patrol_point //固定巡逻点
    {
        double x;
        double y;
    };

    std::vector<patrol_point> patrol_points{
      {0.67,7.185},
      {2.47,2.567},
    };
    
    uint8_t enemy_HP_Hero;
    uint8_t enemy_HP_Infansty1;
    uint8_t enemy_HP_Infansty2;
    uint8_t enemy_HP_Sentry;
    uint8_t enemy_HP_Base;

    rm_msgs::addblood_rx addblood_rx_;
    rm_msgs::addblood_tx addblood_tx_;
    ros::Subscriber addblood_sub_;
    ros::Publisher addblood_pub_;


private:
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac;
    move_base_msgs::MoveBaseGoal goal;

    void AddBloodCallback(const boost::shared_ptr<const rm_msgs::addblood_rx>& msg)
    {
        addblood_rx_ = *msg;
        addblood_rx_received_ = true;
    }

    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
    {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal reached");
        }
        else
        {
            ROS_INFO("Goal was not reached");
        }

        // 在此处可以添加完成时的逻辑处理
    }

    void activeCb()
    {
        ROS_INFO("AddBlood goal is active");
        if(addblood_rx_.current_HP == 600)
        {
             ac->cancelAllGoals();
                // ac->waitForResult();
            actionlib::SimpleClientGoalState state = ac->getState();
            ROS_INFO("Attacksentry action state after cancel: %s", state.toString().c_str());
            ROS_INFO("即将退出补血点");
        }
        // 在此处可以添加目标激活时的逻辑处理
    }

    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
    {
     double current_distance = sqrt(pow(goal.target_pose.pose.position.x-feedback->base_position.pose.position.x, 2) + 
                                        pow(goal.target_pose.pose.position.y-feedback->base_position.pose.position.y, 2));
   
        if (current_distance <= 0.5)
        {
            ROS_INFO("Attack sentry distance to goal is less than or equal to 0.5 meters. Sending dodge_ctrl = 1.");
            addblood_tx_.dodge_ctrl = 1;
            addblood_pub_.publish(addblood_tx_);
        }
        else
        {
            addblood_tx_.dodge_ctrl = 0;
            addblood_pub_.publish(addblood_tx_);
        } 
        // 在此处可以添加接收到反馈时的逻辑处理
    }
};
    
}