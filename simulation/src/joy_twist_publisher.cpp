#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoyTwistPublisher : public rclcpp::Node
{
public:

  JoyTwistPublisher() : Node("joy_twist_publisher")
  {
    // パラメータを初期化
    linear_x_ = 0.0;
    linear_y_ = 0.0;
    angular_z_ = 0.0;

    
    declare_parameter("gain_linear_x", 0.0);
    declare_parameter("gain_angular_z", 0.0);
    declare_parameter("is_stamped", false);
    declare_parameter("max_mode", 0);
    get_parameter("gain_linear_x", gain_linear_x_);
    get_parameter("gain_angular_z", gain_angular_z_);
    get_parameter("is_stamped", is_stamped_);
    get_parameter("max_mode", max_mode_);

    RCLCPP_INFO(this->get_logger(), "Parameter: Joy Twist Publisher");
    RCLCPP_INFO(this->get_logger(), "gain_linear_x : %f", gain_linear_x_);
    RCLCPP_INFO(this->get_logger(), "gain_angular_z : %f", gain_angular_z_);
    RCLCPP_INFO(this->get_logger(), "Max Robots : %d", max_mode_);


    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoyTwistPublisher::joyCallback, this, std::placeholders::_1));


    for (int i = 1; i < max_mode_ + 1; i++)
    {
      if (is_stamped_) {
        cmd_vel_stamped_pub_.push_back(this->create_publisher<geometry_msgs::msg::TwistStamped>("/robot" + std::to_string(i) + "/diff_drive_controller/cmd_vel", 10));
      } else {
        cmd_vel_pub_.push_back(this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_" + std::to_string(i), 10));
      }
    }
    mode_ = 0;
  }

private:

  double gain_linear_x_;
  double gain_angular_z_;

  bool is_stamped_;
  int mode_;
  int max_mode_;
  int before_change_mode_time = 0;
  


  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    if (joy_msg->buttons[0] && before_change_mode_time + 1 < joy_msg->header.stamp.sec){
      mode_ += 1;
      before_change_mode_time = joy_msg->header.stamp.sec;
      RCLCPP_INFO(this->get_logger(), "Change Mode : Robot%d", mode_ + 1);
      if (mode_ >= max_mode_){
        mode_ = 0;
      }
    }
    

    linear_x_ = joy_msg->axes[1] * gain_linear_x_; 
    angular_z_ = joy_msg->axes[2] * gain_angular_z_;


    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = linear_x_;
    twist_msg.angular.z = angular_z_;

    if (is_stamped_)
    {

      geometry_msgs::msg::TwistStamped twist_stamped_msg;
      twist_stamped_msg.twist = twist_msg;
      twist_stamped_msg.header = joy_msg->header;
      cmd_vel_stamped_pub_[mode_]->publish(twist_stamped_msg);
    } else {
      cmd_vel_pub_[mode_]->publish(twist_msg);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_pub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> cmd_vel_stamped_pub_;
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