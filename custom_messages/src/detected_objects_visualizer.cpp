#include <rclcpp/rclcpp.hpp>

#include "custom_messages/msg/detected_objects.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class DetectedObjectsVisualizer : public rclcpp::Node
{
public:
  DetectedObjectsVisualizer() : Node("detected_objects_visualizer"){
    std::string topic_name = this->declare_parameter("topic_name", "detected_objects");
    detected_objects_subscription_ = this->create_subscription<custom_messages::msg::DetectedObjects>(
      topic_name, 10, std::bind(&DetectedObjectsVisualizer::detected_objects_callback, this, std::placeholders::_1));
    cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster", 10);
    detected_objects_publisher_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>("detected_objects", 10);
  };

private:
  void detected_objects_callback(const custom_messages::msg::DetectedObjects::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Detected %ld objects", msg->detected_object.size());
    sensor_msgs::msg::PointCloud2::SharedPtr cluster = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    autoware_auto_perception_msgs::msg::DetectedObjects detected_objects;
    std::vector<autoware_auto_perception_msgs::msg::DetectedObject> objects;

    for (const auto &detected_object : msg->detected_object)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(detected_object.cluster, *cloud);
        *merged_cloud += *cloud;
        objects.push_back(detected_object.detected_object);
    }
  

    pcl::toROSMsg(*merged_cloud, *cluster);
    cluster->header = msg->header;
    cluster->header.frame_id = msg->header.frame_id;
    cluster_publisher_->publish(*cluster);

    detected_objects.objects = objects;
    detected_objects.header = msg->header;
    detected_objects_publisher_->publish(detected_objects);
  }

  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr detected_objects_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_publisher_;
  rclcpp::Subscription<custom_messages::msg::DetectedObjects>::SharedPtr detected_objects_subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectedObjectsVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
