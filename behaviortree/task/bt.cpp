#include <iostream>
#include <chrono>
#include <thread>
#include "behaviortree_cpp/bt_factory.h"

using namespace std::chrono_literals;

// Custom Nodes
class NavigateToRoom : public BT::SyncActionNode {
public:
  NavigateToRoom(const std::string &name) : BT::SyncActionNode(name, {}) {}
  BT::NodeStatus tick() override {
    std::cout << "[NavigateToRoom] Entering the room..." << std::endl;
    std::this_thread::sleep_for(2s);
    return BT::NodeStatus::SUCCESS;
  }
};

class OpenFridge : public BT::SyncActionNode {
public:
  OpenFridge(const std::string &name) : BT::SyncActionNode(name, {}) {}
  BT::NodeStatus tick() override {
    std::cout << "[OpenFridge] Opening the fridge..." << std::endl;
    std::this_thread::sleep_for(2s);
    return BT::NodeStatus::SUCCESS;
  }
};

class PickApple : public BT::SyncActionNode {
public:
  PickApple(const std::string &name) : BT::SyncActionNode(name, {}) {}
  BT::NodeStatus tick() override {
    std::cout << "[PickApple] Picking the apple..." << std::endl;
    std::this_thread::sleep_for(2s);
    return BT::NodeStatus::SUCCESS;
  }
};

class ExitRoom : public BT::SyncActionNode {
public:
  ExitRoom(const std::string &name) : BT::SyncActionNode(name, {}) {}
  BT::NodeStatus tick() override {
    std::cout << "[ExitRoom] Exiting the room..." << std::endl;
    std::this_thread::sleep_for(2s);
    return BT::NodeStatus::SUCCESS;
  }
};

// Main
int main() {
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<NavigateToRoom>("NavigateToRoom");
  factory.registerNodeType<OpenFridge>("OpenFridge");
  factory.registerNodeType<PickApple>("PickApple");
  factory.registerNodeType<ExitRoom>("ExitRoom");

  auto tree = factory.createTreeFromFile("/home/citc-26/Desktop/anand/rosbt/task/bt.xml");


  std::cout << "=== Running the Behavior Tree ===" << std::endl;
  tree.tickWhileRunning();

  return 0;
}
