#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <ros/ros.h>

#include "rm_decision/blackboard.h"
#include "rm_decision/game_start.h"
#include "rm_decision/acessbuff.h"
#include "rm_decision/add_blood.h"
// #include "rm_decision/attackbase.h"
#include "rm_decision/attacksentry.h"
#include "rm_decision/dense.h"
#include "rm_decision/patrol.h"
#include "rm_msgs/GameStatus.h"

int main(int argc, char** argv) {
  setlocale(LC_ALL,"");
  ros::init(argc, argv, "tree_node");
  ros::NodeHandle root_nh;
  ros::NodeHandle root_nh2;
  ros::NodeHandle bh_tree_nh("~");
  BT::BehaviorTreeFactory factory;

  //  1. 游戏开始
  BT::NodeBuilder builder_gamestart = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<rm_decision::GameStart>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<rm_decision::GameStart>("GameStart", builder_gamestart);

  BT::NodeBuilder builder_addblood = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<rm_decision::AddBlood>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<rm_decision::AddBlood>("AddBlood", builder_addblood);

  BT::NodeBuilder builder_attacksentry = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<rm_decision::AttackSentry>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<rm_decision::AttackSentry>("AttackSentry", builder_attacksentry);

  BT::NodeBuilder builder_acessbuff = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<rm_decision::AcessBuff>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<rm_decision::AcessBuff>("AcessBuff", builder_acessbuff);

  BT::NodeBuilder builder_dense = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<rm_decision::Dense>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<rm_decision::Dense>("Dense", builder_dense);


  // BT::NodeBuilder builder_patrol = [&root_nh, &bh_tree_nh](const std::string& name,
  //                                                             const BT::NodeConfiguration& config) {
  //   return std::make_unique<rm_decision::Patrol>(name, config, root_nh,bh_tree_nh);
  // };
  // factory.registerBuilder<rm_decision::Patrol>("Patrol", builder_patrol);

  //  5. 读取行为树的xml文件, 根据上面具体行为的实现方法生成行为树
  //  ref: https://www.behaviortree.dev/tutorial_06_subtree_ports/
  //  ref: https://www.behaviortree.dev/tutorial_07_multiple_xml/
  std::string file_path = bh_tree_nh.param("file_path", std::string(" "));
  auto tree = factory.createTreeFromFile(file_path);

  ros::Rate loop_rate(5);  // 5HZ,设置行为树的运行频率
  while (ros::ok()) {
    ROS_INFO("3");
    BT::NodeStatus status = tree.tickRoot();
    std::cout << status <<  std::endl; //'\n' <<

      while(status == BT::NodeStatus::RUNNING) 
  {
    // Sleep to avoid busy loops.
    // do NOT use other sleep functions!
    // Small sleep time is OK, here we use a large one only to
    // have less messages on the console.
    tree.sleep(std::chrono::milliseconds(100));

    std::cout << "--- ticking\n";
    status = tree.tickRoot();
    std::cout << "--- status: " << toStr(status) << "\n\n";
    ros::spinOnce();
  }
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}