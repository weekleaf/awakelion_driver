#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include "rm_msgs/patrol_rx.h"
#include "rm_msgs/patrol_tx.h"

namespace rm_decision  {

class Patrol : public BT::SyncActionNode {
    public:
    Patrol(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)                      
    {
        ros::NodeHandle nh(root_nh);
        patrol_sub_ = nh.subscribe("patrol_blackboard", 1, &Patrol::PatrolCallback, this);
        patrol_pub_ = nh.advertise<rm_msgs::patrol_tx>("patrol_output", 10);
 
    };


    static BT::PortsList providedPorts() {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<uint8_t>("blood_output"));
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
                            boost::bind(&Patrol::doneCb, this, _1, _2),
                            boost::bind(&Patrol::activeCb, this),
                            boost::bind(&Patrol::feedbackCb, this, _1));
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
        std::cout << "patrol!!!" << std::endl;
        auto addblood_need = getInput<uint8_t>("blood_output");
        if(addblood_need == 1)
            addblood_need_ = 1;
        else addblood_need_ = 0;
        if(addblood_need_ )
        {
            return BT::NodeStatus::SUCCESS;
        }

        sendGoal(patrol_points[0].x,patrol_points[0].y,0);

        return BT::NodeStatus::SUCCESS;
    }

    int current_point = 0;
    uint8_t addblood_need_ = 0;
    bool patrol_received_;
    bool goal_arrive;

    rm_msgs::patrol_rx patrol_rx_;
    rm_msgs::patrol_tx patrol_tx_;
    ros::Subscriber patrol_sub_;
    ros::Publisher patrol_pub_;
    BT::NodeStatus status;
    private:
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac;
    move_base_msgs::MoveBaseGoal goal;
    struct patrol_point //固定巡逻点
    {
        double x;
        double y;
    };

    std::vector<patrol_point> patrol_points{
      {8.4,1.2},
      {8.45,2.0},
      {6.0,1.5},
      {-1,1}
    };

    void PatrolCallback(const boost::shared_ptr<const rm_msgs::patrol_rx>& msg)
    {
        std::cout << "成功patrol" << std::endl;
        patrol_rx_ = *msg;
        patrol_received_ = true;
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
        ROS_INFO("Patrol goal is active");
            if(addblood_need_ )
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
        ROS_INFO("Patrol goal is active");
        double current_distance = sqrt(pow(goal.target_pose.pose.position.x-feedback->base_position.pose.position.x, 2) + 
                                        pow(goal.target_pose.pose.position.y-feedback->base_position.pose.position.y, 2));
        // 如果距离小于或等于0.5米，发送 dodge_ctrl = 1 的标志位
        if (current_distance <= 0.5)
        {
            ROS_INFO("Distance to goal is less than or equal to 0.5 meters. Sending dodge_ctrl = 1.");
            patrol_tx_.dodge_ctrl = 1;
        }
        // 在此处可以添加接收到反馈时的逻辑处理
    }

};

}


    // BT::NodeStatus onStart() override
    // {
    //     ROS_INFO("Patrol");

        
    //     if(patrol_.self_color == 1 )
    //     {
    //         std::cout << "运行中" << std::endl;
    //         return BT::NodeStatus::RUNNING;

    //     }
    //     else
    //     return BT::NodeStatus::SUCCESS;
    // }

    // BT::NodeStatus onRunning() override
    // {
    //     std::cout << "运行running程序" <<std::endl;
    //     ROS_INFO("patrol_.self_color = %u",patrol_.self_color);
    //     if(patrol_.self_color == 0)
    //     {
    //         return BT::NodeStatus::SUCCESS;
    //     }
    //     else 
    //     return BT::NodeStatus::RUNNING;
    // }

    // void onHalted() override
    // {
    //     std::cout << "进入回调函数" << std::endl;
    // }