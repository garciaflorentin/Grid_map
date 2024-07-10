#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TransformPublisherNode : public rclcpp::Node
{
public:
  TransformPublisherNode()
    : Node("transform_publisher_node")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "/model/robot/pose", 10, std::bind(&TransformPublisherNode::transform_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    RCLCPP_INFO(this->get_logger(), "Transform Publisher Node has been started.");
  }

private:
  void transform_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    tf2_msgs::msg::TFMessage tf_message;
    tf_message.transforms.push_back(*msg);
    publisher_->publish(tf_message);
    // RCLCPP_INFO(this->get_logger(), "Published a transform to /tf");
  }

  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
