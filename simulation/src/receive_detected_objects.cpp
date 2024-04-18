#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/lexical_cast.hpp>


#include <iostream>
#include <fstream>
#include <string>
#include <vector>

class VehicleControlCommandSubscriber : public rclcpp::Node {
public:
  VehicleControlCommandSubscriber() : Node("vehicle_control_command_subscriber") {
    // VehicleControlCommandメッセージを受信するサブスクライバを作成
    vehicle_control_command_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::TrackedObjects>(
        "/perception/object_recognition/tracking/objects", 10, std::bind(&VehicleControlCommandSubscriber::onVehicleControlCommand, this, std::placeholders::_1));
  }

  ~VehicleControlCommandSubscriber() {
      saveDataToFile();
  }


private:
  void onVehicleControlCommand(const autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr msg) {
    // VehicleControlCommandメッセージを受信したときの処理

    RCLCPP_INFO(this->get_logger(), "Received VehicleControlCommand:");
    for (size_t i = 0; i < msg->objects.size(); ++i)
    {
      const auto& object = msg->objects[i];
      std::stringstream ss;
      for (auto i = 0; i < 16; ++i) {
        ss << std::hex << std::setfill('0') << std::setw(2) << +object.object_id.uuid[i];
      }
      const std::string uuid_str = ss.str();

      RCLCPP_INFO(this->get_logger(), "object_id : 0x%04X", object.object_id.uuid);
      dataToSave_.emplace_back(std::make_tuple(uuid_str, msg->header.stamp.sec , msg->header.stamp.nanosec, 
        object.kinematics.pose_with_covariance.pose.position.x,
        object.kinematics.pose_with_covariance.pose.position.y,
        object.kinematics.pose_with_covariance.pose.position.z));
       
    }
  }

  void saveDataToFile() {
      std::ofstream csvFile("output.csv", std::ios::out);
      csvFile << "UUID,Timestamp,x,y,z" << std::endl;
      for (const auto& [uuid_str, sec, nanosec, x, y, z] : dataToSave_) {
          csvFile << uuid_str << "," << sec << "." << std::setw(9) << std::setfill('0') << nanosec << "," << x << "," << y << "," << z << std::endl;
      }
      csvFile.close();
  }

  rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr vehicle_control_command_sub_;
  std::vector<std::tuple<std::string, int, uint, double, double, double>> dataToSave_;

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleControlCommandSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}