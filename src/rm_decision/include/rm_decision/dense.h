#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <rm_msgs/dense_rx.h>
#include <rm_msgs/dense_tx.h>
#include <geometry_msgs/PointStamped.h>

namespace rm_decision  {
// template <typename T>
class Dense : public BT::SyncActionNode {
    public:
        Dense(const std::string& name , const BT::NodeConfiguration& config 
        ,const ros::NodeHandle& root_nh
        ,const ros::NodeHandle& tree_nh) 
        : BT::SyncActionNode(name, config)                      
        {
            ros::NodeHandle nh(root_nh);
            ac = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
            dense_sub_ = nh.subscribe("dense_blackboard", 1, &Dense::GameStatusCallback, this);
            dense_pub_ = nh.advertise<rm_msgs::dense_tx>("dense_output", 10);
        };

        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            ports_list.insert(BT::InputPort<uint8_t>("blood_output"));
            ports_list.insert(BT::InputPort<uint8_t>("bullet_output"));
            return ports_list;
        } 

        void publish()
        {
            dense_tx_.dodge_ctrl = dodge_ctrl;
            ROS_INFO("dense_tx_.dodge_ctrl is %s",dense_tx_.dodge_ctrl ? "true" : "false");
            dense_pub_.publish(dense_tx_);
        }
        
        void reset()
        {
            dense_tx_ = rm_msgs::dense_tx();
        }
    // 比较机器人当前位置与目标位置
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
                            boost::bind(&Dense::doneCb, this, _1, _2),
                            boost::bind(&Dense::activeCb, this),
                            boost::bind(&Dense::feedbackCb, this, _1));
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

        void PresetPostion()
        {
            // patrol_points[0].x = 4.0;
            // patrol_points[0].y = 1.1;
                //   {2.47,2.567},
            patrol_points[0].x = 2.47;
            patrol_points[0].y = 2.567;
            patrol_points[1].x = 5.26;
            patrol_points[1].y = 2.2;
            patrol_points[2].x = 6.0;
            patrol_points[2].y = 1.5;

            // patrol_points[0].x = 4.016;
            // patrol_points[0].y = 4.39;
            // patrol_points[1].x = 2.646;
            // patrol_points[1].y = 2.607;
        }

        BT::NodeStatus tick() override
        {
            dense_tx_.dodge_ctrl = 0;
            PresetPostion();
            auto addblood_need = getInput<uint8_t>("blood_output");
            // auto bullet_need = getInput<uint8_t>("bullet_output");

            if(addblood_need == 1)
                addblood_need_ = 1;
            else addblood_need_ = 0;

            // if(bullet_need == 1)
            //     bullet_need_ = 1;
            // else bullet_need_ = 0;

            
            if(addblood_need_ || bullet_need_)
            {
                return BT::NodeStatus::SUCCESS;
            }
            // if(dense_rx_.stage_remain_time >= 190)
            // {
            //     sendGoal(patrol_points[0].x,patrol_points[0].y,0);
            // }
            if(dense_rx_.stage_remain_time <= 209)
            {
                sendGoal(patrol_points[0].x,patrol_points[0].y,0);
            } 

            actionlib::SimpleClientGoalState state = ac->getState();
            ROS_INFO("DENSE IS :%s",state.toString().c_str());
            if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                // dodge_ctrl = 1;
                dense_tx_.dodge_ctrl = 1;
                ROS_INFO("dense_tx_.dodge_ctrl1 =%d",dense_tx_.dodge_ctrl);
                dense_pub_.publish(dense_tx_);
                return BT::NodeStatus::SUCCESS;
            }
            else 
            {
                dense_pub_.publish(dense_tx_);
                return BT::NodeStatus::FAILURE;
            }
            // dense_pub_.publish(dense_tx_);
            // reset();
            dense_tx_.dodge_ctrl = 0;
            dense_pub_.publish(dense_tx_);
            return BT::NodeStatus::SUCCESS;
        }


        bool dense_received_;
        bool cancle_vaild;
        bool dodge_ctrl;
        
        uint8_t addblood_need_ = 0;
        uint8_t bullet_need_ = 0;
        rm_msgs::dense_rx dense_rx_;
        rm_msgs::dense_tx dense_tx_;
        ros::Subscriber dense_sub_;
        ros::Publisher dense_pub_;
    private:
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac;
    move_base_msgs::MoveBaseGoal goal;
    struct patrol_point //固定巡逻点
    {
        double x;
        double y;
    };

    patrol_point patrol_points[4];


    // std::vector<patrol_point> patrol_points{
    //   {4.0,1.1},
    //   {8.45,2.0},
    //   {6.0,1.5},
    //   {-1,1}
    // };
    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
    {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal reached");
            dense_tx_.dodge_ctrl = 1;
            dense_pub_.publish(dense_tx_);
        }
        else
        {
            ROS_INFO("Goal was not reached");
        }

        // 在此处可以添加完成时的逻辑处理
    }

    void activeCb()
    {
        ROS_INFO("Dense goal is active");
          if(addblood_need_)
            {
                ac->cancelAllGoals();
                // ac->waitForResult();
                actionlib::SimpleClientGoalState state = ac->getState();
                ROS_INFO("Action state after cancel: %s", state.toString().c_str());
                // ros::shutdown();
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
            dense_tx_.dodge_ctrl = 1;
            dense_pub_.publish(dense_tx_);
        }
        else 
        {
            dense_tx_.dodge_ctrl = 0;
            dense_pub_.publish(dense_tx_);
        }
        // 在此处可以添加接收到反馈时的逻辑处理
    }
    void GameStatusCallback(const boost::shared_ptr<const rm_msgs::dense_rx>& msg) // const rm_msgs::GameStatus::ConstPtr& msg
    {
        std::cout << "成功dense" << std::endl;
        dense_rx_ = *msg;
        dense_received_ = true;
    }
};
}