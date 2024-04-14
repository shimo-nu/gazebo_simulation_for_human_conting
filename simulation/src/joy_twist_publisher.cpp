#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoyTwistPublisher : public rclcpp::Node
{
public:
  JoyTwistPublisher() : Node("joy_twist_publisher")
  {
    // Joyノードからのデータを受け取るサブスクライバを作成
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoyTwistPublisher::joyCallback, this, std::placeholders::_1));

    // cmd_velをパブリッシュするパブリッシャーを作成
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // パラメータを初期化
    linear_x_ = 0.0;
    linear_y_ = 0.0;
    angular_z_ = 0.0;
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    // Joyスティックの値に応じてパラメータを設定
    linear_x_ = joy_msg->axes[1] * 0.5;  // y軸(-1.0 to 1.0)
    // linear_y_ = joy_msg->axes[0] * 0.5;  // x軸(-1.0 to 1.0)
    angular_z_ = joy_msg->axes[2] * 0.5; // z軸(-1.0 to 1.0)

    // cmd_velメッセージを作成
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = linear_x_;
    // twist_msg.linear.y = linear_y_;
    twist_msg.angular.z = angular_z_;

    // cmd_velをパブリッシュ
    cmd_vel_pub_->publish(twist_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  double linear_x_, linear_y_, angular_z_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyTwistPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}