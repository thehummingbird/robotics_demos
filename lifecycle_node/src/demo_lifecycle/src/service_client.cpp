#include <memory>
#include <chrono>
//#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"


using namespace std::chrono_literals;

static constexpr char const *  talker_node = "lc_talker";

static constexpr char const * node_get_state_topic = "lc_talker/get_state";
static constexpr char const * node_change_state_topic = "lc_talker/change_state";

template <typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
  FutureT & future,
  WaitTimeT time_to_wait
)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do{
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;

    if (time_left<=std::chrono::seconds(0))
    {
      break;
    }
    status = future.wait_for((time_left< wait_period)? time_left: wait_period);

  }
  while(rclcpp::ok() && status!=std::future_status::ready);
  return status;
}

class ServiceClient : public rclcpp::Node
{
public:
  explicit ServiceClient(const std::string & node_name)
  : Node(node_name)
  {
    client_get_state = this->create_client<lifecycle_msgs::srv::GetState>(
      node_get_state_topic);
    client_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(
    node_change_state_topic);

  }

  unsigned int
  get_state(std::chrono::seconds timeout = 3s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state->wait_for_service(timeout))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Service %s not available",
        client_get_state->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto future_result = client_get_state->async_send_request(request);

    auto future_status = wait_for_result(future_result,timeout);

    if (future_status != std::future_status::ready)
    {
      RCLCPP_ERROR(
      get_logger(),
      "Server timed out while getting current state for node %s",
      talker_node);

      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if (future_result.get())
    {
      auto state = future_result.get()->current_state.id;
      RCLCPP_INFO(
        get_logger(),
        "Node %s has current state %s",
        talker_node, future_result.get()->current_state.label.c_str());

      return state;
    }
    else{
      RCLCPP_ERROR(
      get_logger(),
      "Failed to get current state for node %s",
      talker_node);

      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  bool
  change_state(std::uint8_t transition, std::chrono::seconds timeout = 3s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state->wait_for_service(timeout))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Service %s not available",
        client_change_state->get_service_name());
        return false;
    }

    auto future_result = client_change_state->async_send_request(request);

    auto future_status = wait_for_result(future_result,timeout);

    if (future_status!=std::future_status::ready)
    {
      RCLCPP_ERROR(
      get_logger(),
      "Server timed out while getting current state for node %s",
      talker_node);

      return false;
    }

    if (future_result.get()->success)
    {
      RCLCPP_INFO(
      get_logger(),
      "Transition %d successfully triggered",
      static_cast<unsigned int>(transition));
      return true;
    }
    else{
      RCLCPP_WARN(
      get_logger(),
      "Failed to get trigger transition %d for node %s",
      static_cast<unsigned int>(transition),
      talker_node);

      return false;
    }
  }

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state;
};

void
callee_script(std::shared_ptr<ServiceClient> service_client)
{
  rclcpp::WallRate time_between_state_changes(0.1); //10s

  //configure
  {
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //activate
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //deactivate
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //activate
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //deactivate
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //cleanup
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //unconfigured shutdown
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

}

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto service_client = std::make_shared<ServiceClient>("lc_client");
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(service_client);

  std::shared_future<void> script = std::async(
    std::launch::async,
    std::bind(callee_script,service_client));

  executor.spin_until_future_complete(script);

  rclcpp::shutdown();

  return 0;
}