#include "lbr_fri_ros2/client.hpp"

namespace lbr_fri_ros2 {
Client::Client(const rclcpp::Node::SharedPtr node_ptr)
    : logging_interface_ptr_(node_ptr->get_node_logging_interface()),
      parameters_interface_ptr_(node_ptr->get_node_parameters_interface()),
      clock_ptr_(node_ptr->get_clock()),
      command_interface_(node_ptr),
      state_interface_(logging_interface_ptr_, parameters_interface_ptr_), open_loop_(true) {
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Configuring client.");
  if (!node_ptr->has_parameter("open_loop")) {
    node_ptr->declare_parameter("open_loop", true);
  }
  node_ptr->get_parameter("open_loop", open_loop_);
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "Configured client with open_loop '%s'.",
              open_loop_ ? "true" : "false");
}

void Client::onStateChange(KUKA::FRI::ESessionState old_state, KUKA::FRI::ESessionState new_state) {
  RCLCPP_INFO(logging_interface_ptr_->get_logger(), "LBR switched from %s to %s.",
              EnumMaps::session_state_map(old_state).c_str(),
              EnumMaps::session_state_map(new_state).c_str());
  command_interface_.init_command(robotState());
}

void Client::monitor() { 
  auto now = clock_ptr_->now();
  state_interface_.set_state(robotState(),now); 
 };

void Client::waitForCommand() {
  auto now = clock_ptr_->now();
  KUKA::FRI::LBRClient::waitForCommand();
  state_interface_.set_state(robotState(), now);

  if (robotState().getClientCommandMode() == KUKA::FRI::EClientCommandMode::TORQUE) {
    command_interface_.get_torque_command(robotCommand(), robotState());
  }

  if (robotState().getClientCommandMode() == KUKA::FRI::EClientCommandMode::WRENCH) {
    command_interface_.get_wrench_command(robotCommand(), robotState());
  }
}

void Client::command() {
  auto now = clock_ptr_->now();
  if (open_loop_) {
    state_interface_.set_state_open_loop(robotState(),
                                         command_interface_.get_command().joint_position, now);
  } else {
    state_interface_.set_state(robotState(), now);
  }

  switch (robotState().getClientCommandMode()) {
  case KUKA::FRI::EClientCommandMode::POSITION:
    command_interface_.get_joint_position_command(robotCommand(), robotState());
    return;
  case KUKA::FRI::EClientCommandMode::TORQUE:
    command_interface_.get_torque_command(robotCommand(), robotState());
    return;
  case KUKA::FRI::EClientCommandMode::WRENCH:
    command_interface_.get_wrench_command(robotCommand(), robotState());
    return;
  default:
    std::string err =
        "Unsupported command mode: " + std::to_string(robotState().getClientCommandMode()) + ".";
    RCLCPP_ERROR(logging_interface_ptr_->get_logger(), err.c_str());
    throw std::runtime_error(err);
  }
}
} // end of namespace lbr_fri_ros2
