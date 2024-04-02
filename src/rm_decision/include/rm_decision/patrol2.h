#pragma once
#include "vector"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib/client/simple_action_client.h"
#include "tf/tf.h"
#include "rm_msgs/patrol_rx.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "rm_decision/add_blood.h"

using namespace std;
using namespace ros;

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseActionGoal> MoveBaseClient;


namespace rm_decision  {
class patrol : public BT::SyncActionNode {
public: 
    patrol(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)                      
    {
        NodeHandle patrol_nh(root_nh);
        MoveBaseClient = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseActionGoal>>("patrol",true);
        patrol_status_sub = patrol_nh.subscribe("patrol_status", 100, &patrol::patrolStatusCallback, this);
        publisher = patrol_nh.advertise<rm_msgs::patrol_rx>("patrol_status_output", 10);
    };   
    
    int current_point=0; //初始化
    bool patrol_status_received;
    Subscriber patrol_status_sub;
    Publisher publisher;
    rm_msgs::patrol_rx patrol_rx;

    struct patrol_point //固定巡逻点
    {
        double x;
        double y;
    };

    static BT::PortsList providedPorts() {
    BT::PortsList ports_list;
    return ports_list;
    }
    
    void patrolStatusCallback(const boost::shared_ptr<const rm_msgs::patrol_rx>& msg) // const rm_msgs::patrolStatus::ConstPtr& msg
    {
        patrol_rx = *msg;
        patrol_status_received = true;
        current_point = (current_point + 1) % patrol_points.size();
    }

    void Run(double x,double y)//,MoveBaseClient &ac
    {
        try{
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map"; 
        goal.target_pose.header.stamp = Time();
        goal.target_pose.pose.position.x=patrol_points[current_point].x;
        goal.target_pose.pose.position.y=patrol_points[current_point].y;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;                     
        geometry_msgs::Quaternion Quaternion;                                 
        Quaternion = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw); 
        goal.target_pose.pose.orientation = Quaternion;
        MoveBaseClient->sendGoal(goal);
        MoveBaseClient->waitForResult();
        if (MoveBaseClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("You have reached the goal!");
        else
            ROS_INFO("The base failed for some reason");
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }    
   
    void reset()
    {        
        patrol_status_received = false;
        patrol_rx = rm_msgs::patrol_rx();
    }

    BT::NodeStatus tick() override//MoveBaseClient &ac,
    {
        int enemy_hp,color;
        if(patrol_rx.robot_color!=patrol_rx.self_color && patrol_rx.visual_valid==1)
        {  
           if(patrol_rx.self_color==0) 
           {
            switch (patrol_rx.enemy_id)
            {
                case 1:
                enemy_hp=patrol_rx.blue_1_robot_HP;
                break;
                case 3:
                enemy_hp=patrol_rx.blue_3_robot_HP;
                break;
                case 7:
                enemy_hp=patrol_rx.blue_7_robot_HP;
                break;
            }
           }
            else
           {
            switch (patrol_rx.enemy_id)
            {
                case 1:
                enemy_hp=patrol_rx.red_1_robot_HP;
                break;
                case 3:
                enemy_hp=patrol_rx.red_3_robot_HP;
                break;
                case 7:
                enemy_hp=patrol_rx.red_7_robot_HP;
                break;
            }
           }
           if(patrol_rx.self_color<=150)
            return BT::NodeStatus::FAILURE;
        }
        else if(enemy_hp=0)
        {
            return BT::NodeStatus::SUCCESS;
        }
    }

    void modeswticher(NodeHandle root_nh)//MoveBaseClient ac,
    {
        double x,y;
        if(tick()==BT::NodeStatus::FAILURE)
        {
            try{
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map"; 
        goal.target_pose.header.stamp = Time();
        goal.target_pose.pose.position.x=0.5; //加血点
        goal.target_pose.pose.position.y=7.8;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;                     
        geometry_msgs::Quaternion Quaternion;                                 
        Quaternion = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw); 
        goal.target_pose.pose.orientation = Quaternion;
        MoveBaseClient->sendGoal(goal);
        MoveBaseClient->waitForResult();
        if (MoveBaseClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("You have reached the goal!");
        else
            ROS_INFO("The base failed for some reason");
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        }
        else if(tick()==BT::NodeStatus::SUCCESS)
        {
            Run(x,y);
        }
    }

    ~patrol()=default;

    private:
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseActionGoal>> MoveBaseClient;


    vector<patrol_point> patrol_points{
      {1,1},
      {1,-1},
      {-1,-1},
      {-1,1}
    }; //储存巡逻点
    
    };

}

