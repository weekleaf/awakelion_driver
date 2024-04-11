#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include "rm_msgs/acessbuff_rx.h"
#include "rm_msgs/acessbuff_tx.h" 

namespace rm_decision  {

class AcessBuff : public BT::SyncActionNode {
public:
    AcessBuff(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)                      
    {
        ros::NodeHandle nh(root_nh);
        ac = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
        acessbuff_rx_sub_ = nh.subscribe<rm_msgs::acessbuff_rx>("acessbuff_blackboard", 1, &AcessBuff::AcessBuffCallback, this);
        acessbuff_tx_pub_ = nh.advertise<rm_msgs::acessbuff_tx>("acessbuff_output", 10);
    };

    static BT::PortsList providedPorts() {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<uint8_t>("blood_output"));
        ports_list.insert(BT::InputPort<uint8_t>("bullet_output"));
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
                        boost::bind(&AcessBuff::doneCb, this, _1, _2),
                        boost::bind(&AcessBuff::activeCb, this),
                        boost::bind(&AcessBuff::feedbackCb, this, _1));
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

    void reset()
    {        //重置该值
        acessbuff_rx_ = rm_msgs::acessbuff_rx();
    }

    BT::NodeStatus tick() override
    {
        acessbuff_tx_.dodge_ctrl = 0;
        auto addblood_need = getInput<uint8_t>("blood_output");
        auto bullet_need = getInput<uint8_t>("bullet_output");
        
        if(addblood_need == 1)
                addblood_need_ = 1;
        else addblood_need_ = 0;

        if(bullet_need == 1)
                bullet_need_ = 1;
        else bullet_need_ = 0;


        if(addblood_need_ || acessbuff_rx_.stage_remain_time > 244 || bullet_need_)
        {
            return BT::NodeStatus::SUCCESS;
        }
        if(buff_time == 0 && acessbuff_rx_.stage_remain_time < 244 
        && acessbuff_rx_.stage_remain_time >= 184)
        {
                GoBuff();
                if(acessbuff_rx_.center_activate != 1)
                {
                    acessbuff_tx_.dodge_ctrl = 1;
                    acessbuff_tx_pub_.publish(acessbuff_tx_);
                    return BT::NodeStatus::FAILURE;
                }
                buff_time++;
                next_buff_time = acessbuff_rx_.stage_remain_time - 90;
                acessbuff_tx_.dodge_ctrl = 0;
                acessbuff_tx_pub_.publish(acessbuff_tx_);
                reset();
                return BT::NodeStatus::SUCCESS;
            
        }
        else if(buff_time == 1 && acessbuff_rx_.stage_remain_time <= next_buff_time + 5 
                && acessbuff_rx_.stage_remain_time >= next_buff_time - 60)
        {
                GoBuff();
                if(acessbuff_rx_.center_activate != 1)
                {
                    acessbuff_tx_.dodge_ctrl = 1;
                    acessbuff_tx_pub_.publish(acessbuff_tx_);
                    return BT::NodeStatus::FAILURE;
                }
                acessbuff_tx_.dodge_ctrl = 0;
                acessbuff_tx_pub_.publish(acessbuff_tx_);
                reset();
                return BT::NodeStatus::SUCCESS;

        }
        else 
        {
            std::cout << "buff未开启" << std::endl;
            reset();
            acessbuff_tx_.dodge_ctrl = 0;
            acessbuff_tx_pub_.publish(acessbuff_tx_);
            return BT::NodeStatus::SUCCESS;
        }
    }

    void GoBuff()
    {
        sendGoal(patrol_points[0].x,patrol_points[0].y,0);
    }

    struct patrol_point //固定巡逻点
    {
        double x;
        double y;
    };

    std::vector<patrol_point> patrol_points{
      {6.0,3.9}
    };


    int buff_time = 0;
    int next_buff_time;

    bool addblood_rx_received_;
    uint8_t addblood_need_ = 0;
    uint8_t bullet_need_ = 0;
    
    rm_msgs::acessbuff_rx acessbuff_rx_;
    rm_msgs::acessbuff_tx acessbuff_tx_;
    ros::Subscriber acessbuff_rx_sub_;
    ros::Publisher acessbuff_tx_pub_;
private:
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac;
    move_base_msgs::MoveBaseGoal goal;

    void AcessBuffCallback(const rm_msgs::acessbuff_rx::ConstPtr& msg)
    {
        acessbuff_rx_ = *msg;
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
        ROS_INFO("AccessBuff goal is active");
        if(addblood_need_)
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
        // 计算当前机器人与目标点之间的距离
       double current_distance = sqrt(pow(goal.target_pose.pose.position.x-feedback->base_position.pose.position.x, 2) + 
                                        pow(goal.target_pose.pose.position.y-feedback->base_position.pose.position.y, 2));
        // 如果距离小于或等于0.5米，发送 dodge_ctrl = 1 的标志位
        if (current_distance <= 0.5)
        {
            ROS_INFO("Distance to goal is less than or equal to 0.5 meters. Sending dodge_ctrl = 1.");
            acessbuff_tx_.dodge_ctrl = 1;
            acessbuff_tx_pub_.publish(acessbuff_tx_);
        }
        else
        {
            acessbuff_tx_.dodge_ctrl = 0;
            acessbuff_tx_pub_.publish(acessbuff_tx_);
        } 
        // 在此处可以添加接收到反馈时的逻辑处理
    }

};
    
}