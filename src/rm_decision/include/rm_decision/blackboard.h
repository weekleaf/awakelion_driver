#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/blackboard.h>

#include <ros/ros.h>
#include "rm_msgs/GameStatus.h"
#include "rm_msgs/GameResult.h"
#include "rm_msgs/GameRobotHP.h"
#include "rm_msgs/GameBuff.h"
#include "rm_msgs/RobotStatus.h"
#include "rm_msgs/RobotDamage.h"

// #include "rm_msgs/ArmorDetection.h"
#include <geometry_msgs/PoseStamped.h>


namespace rm_decision {

class Blackboard  { //: public BT::Blackboard
    public:
        typedef std::shared_ptr<Blackboard> Ptr;
        // Blackboard(BT::Blackboard::Ptr parent) : BT::Blackboard(parent)
        // {

        //     game_status_received_ = false;
        //     game_result_received_ = false;
        //     game_robot_HP_received_ = false;
        //     game_buff_received_ = false;
        //     robot_status_received_ = false;
        //     robot_damage_received_ = false;
        //     game_status_sub_ = nh_.subscribe("game_status", 1, &Blackboard::GameStatusCallback, this);
        //     game_result_sub_ = nh_.subscribe("game_result", 1, &Blackboard::GameResultCallback, this);
        //     game_robot_HP_sub_ = nh_.subscribe("game_robot_HP", 1, &Blackboard::GameRobotHPCallback, this);
        //     game_event_sub_ = nh_.subscribe("game_buff", 1, &Blackboard::GameBuffCallback, this);
        //     robot_status_sub_ = nh_.subscribe("robot_status", 1, &Blackboard::RobotStatusCallback, this);
        //     robot_damage_sub_ = nh_.subscribe("robot_damage", 1, &Blackboard::RobotDamageCallback, this);

        // }
        Blackboard() 
        {
            std::cout << "blackboardint" <<std::endl;

            game_status_received_ = false;
            game_result_received_ = false;
            game_robot_HP_received_ = false;
            game_buff_received_ = false;
            robot_status_received_ = false;
            robot_damage_received_ = false;
            game_status_sub_ = nh_.subscribe("game_status", 1, &Blackboard::GameStatusCallback, this);
            game_result_sub_ = nh_.subscribe("game_result", 1, &Blackboard::GameResultCallback, this);
            game_robot_HP_sub_ = nh_.subscribe("game_robot_HP", 1, &Blackboard::GameRobotHPCallback, this);
            game_event_sub_ = nh_.subscribe("game_buff", 1, &Blackboard::GameBuffCallback, this);
            robot_status_sub_ = nh_.subscribe("robot_status", 1, &Blackboard::RobotStatusCallback, this);
            robot_damage_sub_ = nh_.subscribe("robot_damage", 1, &Blackboard::RobotDamageCallback, this);
            std::cout << "good and " <<  game_status_.game_progress << std::endl;
            ROS_INFO("%u",game_status_.game_progress);
            ROS_INFO("My value is: %s", game_status_received_ ? "true" : "false");

        }
        rm_msgs::GameStatus  process()
        {

                return   game_status_;

        }
        ~Blackboard() {}

        //各个节点的任务确认标志位
        bool game_status_received_;
        bool game_result_received_;
        bool game_robot_HP_received_;
        bool game_buff_received_;
        bool robot_status_received_;
        bool robot_damage_received_;
        bool robot_pose_received_GimbalAngleCallback;
        bool armor_detection_received_;
        bool sentry_pose_received_;
        bool gimble_angle_received_;
        bool team_info_received_;

        //各类使用的标志位
        enum Color
        {
            red = 0,
            blue = 1
        };
        Color current_robot_color;
        
        uint8_t enemy_HP_Hero;
        uint8_t enemy_HP_Infansty1;
        uint8_t enemy_HP_Infansty2;
        uint8_t enemy_HP_Sentry;
        uint8_t enemy_HP_Base;
        //定义消息类型的变量，用来传递变量
        rm_msgs::GameStatus game_status_;
        rm_msgs::GameResult game_result_;
        rm_msgs::GameRobotHP game_robot_HP_;
        rm_msgs::GameBuff game_buff_;
        rm_msgs::RobotStatus robot_status_;
        rm_msgs::RobotDamage robot_damage_;

        //定义互斥锁，防止多线程导致数据错误赋值
        std::mutex game_status_cbk_mutex_;
        std::mutex game_result_cbk_mutex_;
        std::mutex game_robot_hp_cbk_mutex_;
        std::mutex game_buff_cbk_mutex_;
        std::mutex robot_status_cbk_mutex_;
        std::mutex robot_damage_cbk_mutex_;
        std::mutex eskf_pose_cbk_mutex_;
        std::mutex armor_detect_cbk_mutex_;
        std::mutex sentry_pose_cbk_mutex_;
        std::mutex gimbal_angle_cbk_mutex_;
        std::mutex team_info_cbk_mutex;

        // static BT::PortsList providedPorts() {
        // return {
        //     BT::InputPort<std::shared_ptr<rm_msgs::DecisionInfo>>("decision_info"),
        //     BT::InputPort<std::shared_ptr<rm_msgs::Gimbal>>("gimbal"),
        //     BT::InputPort<std::shared_ptr<rm_msgs::Shoot>>("shoot"),
        // };
        // }
    private:


        ros::NodeHandle nh_;

        //定义多个消息订阅节点
        ros::Subscriber game_status_sub_;
        ros::Subscriber game_result_sub_;
        ros::Subscriber game_robot_HP_sub_;
        ros::Subscriber game_event_sub_;
        ros::Subscriber robot_status_sub_;
        ros::Subscriber robot_damage_sub_;
        ros::Subscriber robot_pose_sub_;
        ros::Subscriber armor_detection_sub_;
        ros::Subscriber sentry_pose_sub_;
        ros::Subscriber gimbal_angle_sub_;
        ros::Subscriber team_info_sub_;
        void GameStatusCallback(const rm_msgs::GameStatus::ConstPtr& msg)
        {
            int i = 0;
            i++;
            game_status_cbk_mutex_.lock();
            std::cout <<  i << " callback" << std::endl;
            game_status_.game_progress = msg->game_progress;

            game_status_received_ = true;
            ROS_INFO("GAME_Stauts is %u",game_status_.game_progress);
                game_status_cbk_mutex_.unlock();
        }
        void GameResultCallback(const rm_msgs::GameResult::ConstPtr msg)
        {
            game_result_cbk_mutex_.lock();
            game_result_ = *msg;
            game_result_received_ = true;
            game_result_cbk_mutex_.unlock();
        }
        void GameRobotHPCallback(const rm_msgs::GameRobotHP::ConstPtr msg)
        {
            game_robot_hp_cbk_mutex_.lock();
            game_robot_HP_ = *msg;
            game_robot_HP_received_ = true;
            game_robot_hp_cbk_mutex_.unlock();
        }
        void GameBuffCallback(const rm_msgs::GameBuff::ConstPtr msg)
        {
            game_buff_cbk_mutex_.lock();
            game_buff_ = *msg;
            game_buff_received_ = true;
            game_buff_cbk_mutex_.unlock();
        }
        void RobotStatusCallback(const rm_msgs::RobotStatus::ConstPtr msg)
        {
            robot_status_cbk_mutex_.lock();
            robot_status_ = *msg;
            robot_status_received_ = true;
            robot_status_cbk_mutex_.unlock();
        }
        void RobotDamageCallback(const rm_msgs::RobotDamage::ConstPtr msg)
        {
            robot_damage_cbk_mutex_.lock();
            robot_damage_ = *msg;
            robot_damage_received_ = true;
            robot_damage_cbk_mutex_.unlock();
        }
        void JudgeDate_Processing()
        {
            if(robot_status_.robot_id >= 101)
                current_robot_color = blue;
            else
                current_robot_color = red;


            if(current_robot_color = blue)
            {
                if(game_robot_HP_.red_1_robot_HP != 0)
                {
                    enemy_HP_Hero = game_robot_HP_.red_1_robot_HP;
                    if(game_robot_HP_.red_3_robot_HP = 0)
                    enemy_HP_Infansty1 = game_robot_HP_.red_4_robot_HP;
                    else
                    enemy_HP_Infansty1 = game_robot_HP_.red_3_robot_HP;
                }  
                else
                {
                    enemy_HP_Infansty1 = game_robot_HP_.red_3_robot_HP;
                    enemy_HP_Infansty2 = game_robot_HP_.red_4_robot_HP;
                }
                enemy_HP_Sentry = game_robot_HP_.red_7_robot_HP;
                enemy_HP_Base   = game_robot_HP_.red_base_HP;
            }
            else if (current_robot_color = red)
            {
                if(game_robot_HP_.blue_1_robot_HP != 0)
                {
                    enemy_HP_Hero = game_robot_HP_.blue_1_robot_HP;
                    if(game_robot_HP_.blue_3_robot_HP = 0)
                    enemy_HP_Infansty1 = game_robot_HP_.blue_4_robot_HP;
                    else 
                    enemy_HP_Infansty1 = game_robot_HP_.blue_3_robot_HP;
                }
                else
                {
                    enemy_HP_Infansty1 = game_robot_HP_.blue_3_robot_HP;
                    enemy_HP_Infansty2 = game_robot_HP_.blue_4_robot_HP;
                }
                enemy_HP_Sentry = game_robot_HP_.blue_7_robot_HP;
                enemy_HP_Base   = game_robot_HP_.blue_base_HP;
            }
            else
                std::cout << "Judge_Color ERROR!" << std::endl;
            

        }
};
} // namespace rm_decision