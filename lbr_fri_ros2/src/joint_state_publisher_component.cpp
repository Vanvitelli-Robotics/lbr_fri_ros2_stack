#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <lbr_fri_msgs/msg/lbr_state.hpp>

namespace lbr_fri_ros2
{

class JointStatePublisher : public rclcpp::Node
{
public:
  JointStatePublisher(const rclcpp::NodeOptions& options) : Node("joint_state_publisher", options)
  {

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                                                  rclcpp::SensorDataQoS());

    joint_state_subscriber_ = this->create_subscription<lbr_fri_msgs::msg::LBRState>(
        "state", rclcpp::SensorDataQoS(),
        std::bind(&JointStatePublisher::jointStateCallback, this, std::placeholders::_1));
  }

  void jointStateCallback(const lbr_fri_msgs::msg::LBRState::SharedPtr msg)
  {
    // Create a JointState message
    sensor_msgs::msg::JointState::UniquePtr joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();

    // Set the timestamp of the JointState message
    joint_state_msg->header = msg->header;

    // Set the names of the joints
    joint_state_msg->name = { "A1", "A2", "A3", "A4", "A5", "A6", "A7" };

    // Set the positions of the joints
    joint_state_msg->position.resize(msg->measured_joint_position.size());
    memcpy(joint_state_msg->position.data(), msg->measured_joint_position.data(),
           sizeof(double) * msg->measured_joint_position.size());

    // velocity not available (yet)
    // joint_state_msg->velocity.resize(msg->measured_joint_velocity.size());
    // memcpy(joint_state_msg->velocity.data(), msg->measured_joint_velocity.data(),
    //        sizeof(double) * msg->measured_joint_velocity.size());

    // Set the efforts of the joints
    joint_state_msg->effort.resize(msg->measured_torque.size());
    memcpy(joint_state_msg->effort.data(), msg->measured_torque.data(), sizeof(double) * msg->measured_torque.size());

    // Publish the JointState message
    joint_state_publisher_->publish(std::move(joint_state_msg));
  }

  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

};  // class JointStatePublisher

}  // namespace lbr_fri_ros2


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_fri_ros2::JointStatePublisher)