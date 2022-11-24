#include <iostream>
#include <chrono>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include <vector>

using namespace std::chrono_literals;

BT::NodeStatus ballFound()
{
  std::cout << "Ball not found" << std::endl;
  return BT::NodeStatus::FAILURE;
}

class FindBall : public BT::SyncActionNode
{
public:
  explicit FindBall(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<std::vector<int>>("ball_location")};
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(3s);
    std::vector<int> ballLocation{1, 2, 3};
    BT::TreeNode::setOutput("ball_location", ballLocation);
    std::cout << "Ball Found" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

BT::NodeStatus ballClose(BT::TreeNode &self)
{
  auto msg = self.getInput<std::vector<int>>("ball_location");

  if (!msg)
  {
    throw BT::RuntimeError("missing required input[message]: ", msg.error());
  }

  for (const auto position_coordinate : msg.value())
  {
    std::cout << position_coordinate << ' ';
  }
  std::cout << "That's far away" << std::endl;

  return BT::NodeStatus::FAILURE;
}

class ApproachBall : public BT::SyncActionNode
{
public:
  explicit ApproachBall(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::vector<int>>("ball_location")};
  }

  BT::NodeStatus tick() override
  {
    auto msg = getInput<std::vector<int>>("ball_location");

    if (!msg)
    {
      throw BT::RuntimeError("missing required input[message]: ", msg.error());
    }

    for (const auto position_coordinate : msg.value())
    {
      std::cout << position_coordinate << ' ';
    }
    std::this_thread::sleep_for(3s);
    std::cout << "Ball Approached" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

BT::NodeStatus ballGrasped()
{
  std::cout << "Ball not grasped" << std::endl;
  return BT::NodeStatus::FAILURE;
}

class GraspBall : public BT::SyncActionNode
{
public:
  explicit GraspBall(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(3s);
    std::cout << "Ball grasped" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

BT::NodeStatus binClose()
{
  std::cout << "Bin not close" << std::endl;
  return BT::NodeStatus::FAILURE;
}

class ApproachBin : public BT::SyncActionNode
{
public:
  explicit ApproachBin(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(3s);
    std::cout << "Bin approached" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

BT::NodeStatus ballPlaced()
{
  std::cout << "Ball not placed" << std::endl;
  return BT::NodeStatus::FAILURE;
}

class PlaceBall : public BT::SyncActionNode
{
public:
  explicit PlaceBall(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(3s);
    std::cout << "Ball placed" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

class AskForHelp : public BT::SyncActionNode
{
public:
  explicit AskForHelp(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::cout << "Asking for help. Waiting for 10 seconds here" << std::endl;
    std::this_thread::sleep_for(10s);
    return BT::NodeStatus::SUCCESS;
  }
};

int main()
{
  BT::BehaviorTreeFactory factory;

  factory.registerSimpleCondition("BallFound", std::bind(ballFound));
  factory.registerNodeType<FindBall>("FindBall");

  BT::PortsList say_something_ports = {BT::InputPort<std::vector<int>>("ball_location")};
  factory.registerSimpleCondition("BallClose", ballClose,
                                  say_something_ports);
  factory.registerNodeType<ApproachBall>("ApproachBall");

  factory.registerSimpleCondition("BallGrasped", std::bind(ballGrasped));
  factory.registerNodeType<GraspBall>("GraspBall");

  factory.registerSimpleCondition("BinClose", std::bind(binClose));
  factory.registerNodeType<ApproachBin>("ApproachBin");

  factory.registerSimpleCondition("BallPlaced", std::bind(ballPlaced));
  factory.registerNodeType<ApproachBin>("PlaceBall");

  factory.registerNodeType<AskForHelp>("AskForHelp");

  // Create Tree
  auto tree = factory.createTreeFromFile("./../bt_tree.xml");

  // execute the tree
  tree.tickWhileRunning();

  return 0;
}