#include <iostream>
#include <chrono>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <vector>

using namespace std::chrono_literals;

class FindBall : public BT::SyncActionNode
{
public:
  explicit FindBall(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<std::vector<int>>("ball_location")};
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(5s);
    std::vector<int> v{1, 2, 3};
    BT::TreeNode::setOutput("ball_location", v);
    std::cout << "Ball Found" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

BT::NodeStatus BallClose(BT::TreeNode &self)
{
  BT::Optional<std::vector<int>> msg = self.getInput<std::vector<int>>("ball_location");

  if (!msg)
  {
    throw BT::RuntimeError("missing required input: ", msg.error());
  }

  //std::cout << "Ball at " << msg.value() << " .That's far away" << std::endl;
  for (const auto position_coordinate : msg.value())
  {
    std::cout << position_coordinate << ' ';
  }
  std::cout << "That's far away" << std::endl;

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ApproachBall()
{
  std::this_thread::sleep_for(5s);
  std::cout << "Approached Ball" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BallGrasped()
{
  std::cout << "Ball not grasped" << std::endl;
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GraspBall()
{
  std::this_thread::sleep_for(5s);
  std::cout << "Grasped Ball" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

class PlaceBall : public BT::SyncActionNode
{
public:
  explicit PlaceBall(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(5s);
    std::cout << "Ball Placed" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

int main()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<FindBall>("FindBall");
  factory.registerNodeType<PlaceBall>("PlaceBall");

  BT::PortsList say_something_ports = {BT::InputPort<std::vector<int>>("ball_location")};
  factory.registerSimpleCondition("BallClose", BallClose,
                                  say_something_ports);

  //factory.registerSimpleCondition("BallClose", std::bind(BallClose));
  factory.registerSimpleAction(
      "ApproachBall",
      std::bind(ApproachBall));

  factory.registerSimpleCondition("BallGrasped", std::bind(BallGrasped));
  factory.registerSimpleAction(
      "GraspBall",
      std::bind(GraspBall));

  //Create Tree
  auto tree = factory.createTreeFromFile("./../bt_tree.xml");

  //execute the tree
  tree.tickRoot();

  return 0;
}