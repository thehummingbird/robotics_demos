#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

class Listener : public rclcpp::Node
{
public:
  explicit Listener(const std::string & nodeName)
  : Node(nodeName)
  {
    messageSubscription = this->create_subscription<std_msgs::msg::String>(
      "messages",
      10,
      std::bind(&Listener::messageCallback, this, std::placeholders::_1)
    );

    notificationSubscription = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "lc_talker/transition_event",
      10,
      std::bind(&Listener::notificationCallback, this, std::placeholders::_1)
    );
  }

  void messageCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(),"messageCallback: %s",msg->data.c_str());
  }

  void notificationCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
  {
    RCLCPP_INFO(
      get_logger(),
      "notificationCallback: Transition from state %s to %s",
      msg->start_state.label.c_str(),
      msg->goal_state.label.c_str()
    );
  }

private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> messageSubscription;
  
  std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> notificationSubscription; 
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto lc_listener = std::make_shared<Listener>("lc_listener");
  rclcpp::spin(lc_listener);

  rclcpp::shutdown();

  return 0;
}