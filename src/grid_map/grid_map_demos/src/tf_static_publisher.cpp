#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;

class TfStaticPublisher : public rclcpp::Node
{
public:
  TfStaticPublisher()
    : Node("tf_static_publisher")
  {
    // Définir la politique de durabilité pour que les messages TF statiques persistent
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.keep_last(10);
    qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);

    // Créer un éditeur pour le topic /tf_static
    tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", qos_profile);

    // Créer le premier message de transformation
    geometry_msgs::msg::TransformStamped transform_stamped_1;
    transform_stamped_1.header.frame_id = "robot";
    transform_stamped_1.child_frame_id = "robot/base_link";
    transform_stamped_1.transform.translation.x = 0.0;
    transform_stamped_1.transform.translation.y = 0.0;
    transform_stamped_1.transform.translation.z = 0.0;
    transform_stamped_1.transform.rotation.x = 0.0;
    transform_stamped_1.transform.rotation.y = 0.0;
    transform_stamped_1.transform.rotation.z = 0.0;
    transform_stamped_1.transform.rotation.w = 1.0;

    // Créer le deuxième message de transformation
    geometry_msgs::msg::TransformStamped transform_stamped_2;
    transform_stamped_2.header.frame_id = "robot/base_link";
    transform_stamped_2.child_frame_id = "robot/base_link/lidar";
    transform_stamped_2.transform.translation.x = 0.03;
    transform_stamped_2.transform.translation.y = 0.0;
    transform_stamped_2.transform.translation.z = 0.64;
    transform_stamped_2.transform.rotation.x = 0.0;
    transform_stamped_2.transform.rotation.y = 0.0;
    transform_stamped_2.transform.rotation.z = 0.0;
    transform_stamped_2.transform.rotation.w = 1.0;

    // Créer le troisième message de transformation
    geometry_msgs::msg::TransformStamped transform_stamped_3;
    transform_stamped_3.header.frame_id = "robot/base_link";
    transform_stamped_3.child_frame_id = "robot/base_link/camera";
     transform_stamped_3.transform.translation.x = 0.05;
    transform_stamped_3.transform.translation.y = 0.0;
    transform_stamped_3.transform.translation.z = 0.52;
    transform_stamped_3.transform.rotation.x = 0.0;
    transform_stamped_3.transform.rotation.y = 0.0;
    transform_stamped_3.transform.rotation.z = 0.0;
    transform_stamped_3.transform.rotation.w = 1.0;

    // Créer le message TFMessage et y ajouter les transformations
    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(transform_stamped_1);
    tf_msg.transforms.push_back(transform_stamped_2);
    tf_msg.transforms.push_back(transform_stamped_3);

    // Planifier la publication du message à intervalles réguliers
    timer_ = this->create_wall_timer(
      500ms, [this, tf_msg]() {
        tf_publisher_->publish(tf_msg);
        // RCLCPP_INFO(this->get_logger(), "TF static published");
      });
  }

private:
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfStaticPublisher>());
  rclcpp::shutdown();
  return 0;
}
