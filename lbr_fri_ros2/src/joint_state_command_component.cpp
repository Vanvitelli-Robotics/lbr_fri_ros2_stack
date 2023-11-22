#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <lbr_fri_msgs/msg/lbr_position_command.hpp>

namespace lbr_fri_ros2
{

class JointStateCommand : public rclcpp::Node
{
public:
  JointStateCommand(const rclcpp::NodeOptions& options) : Node("joint_state_command", options)
  {
    joint_command_publisher_ = this->create_publisher<lbr_fri_msgs::msg::LBRPositionCommand>("command/position", 1);

    joint_state_command_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "command/joint_states", rclcpp::SensorDataQoS(),
        std::bind(&JointStateCommand::jointStateCommandCallback, this, std::placeholders::_1));
  }

  void jointStateCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Create a JointState message
    lbr_fri_msgs::msg::LBRPositionCommand::UniquePtr joint_cmd_msg =
        std::make_unique<lbr_fri_msgs::msg::LBRPositionCommand>();

    // Set the positions of the joints
    memcpy(joint_cmd_msg->joint_position.data(), msg->position.data(), sizeof(double) * 7);

    // Publish the JointState message
    joint_command_publisher_->publish(std::move(joint_cmd_msg));
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_command_subscriber_;
  rclcpp::Publisher<lbr_fri_msgs::msg::LBRPositionCommand>::SharedPtr joint_command_publisher_;

};  // class JointStateCommand

}  // namespace lbr_fri_ros2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_fri_ros2::JointStateCommand)