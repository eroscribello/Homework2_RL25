#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2_kdl_package/action/execute_trajectory.hpp"

using namespace std::chrono_literals;

class ExecuteTrajectoryClient : public rclcpp::Node
{
public:
  using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
  using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

  ExecuteTrajectoryClient()
  : Node("execute_trajectory_client")
  {
    client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "ExecuteTrajectory");

    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action not available after 10 seconds");
      rclcpp::shutdown();
    }
  }

  void send_goal(int order)
  {
    auto goal_msg = ExecuteTrajectory::Goal();
    goal_msg.order = order;

    RCLCPP_INFO(this->get_logger(), "Sending Goal: %d", order);

    auto options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
    options.goal_response_callback =
        std::bind(&ExecuteTrajectoryClient::goal_response_callback, this, std::placeholders::_1);
    options.feedback_callback =
        std::bind(&ExecuteTrajectoryClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback =
        std::bind(&ExecuteTrajectoryClient::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, options);
  }

private:
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr client_;

  void goal_response_callback(const GoalHandleExecuteTrajectory::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rifiutato dal server");
    } else {
      RCLCPP_INFO(get_logger(), "Goal accettato, in attesa del risultato...");
    }
  }

  void feedback_callback(
    GoalHandleExecuteTrajectory::SharedPtr,
    const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(),
                "Feedback: position_error = [%f, %f, %f]",
                feedback->position_error[0],
                feedback->position_error[1],
                feedback->position_error[2]);
  }

  void result_callback(const GoalHandleExecuteTrajectory::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal Completed Succesfully: %s",
                    result.result->success ? "true" : "false");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal Aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "Goal Cancelled");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown Result");
        break;
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExecuteTrajectoryClient>();

  // Esempio: invio goal 1
  node->send_goal(1);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

