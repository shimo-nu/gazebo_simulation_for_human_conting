#ifndef MULTI_ROBOT_OBJECT_DETECTION_MERGER_HPP_
#define MULTI_ROBOT_OBJECT_DETECTION_MERGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "custom_messages/msg/detected_objects.hpp"
#include "custom_messages/msg/detected_object.hpp"
#include "autoware_auto_perception_msgs/msg/detected_object.hpp"
#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include <mutex>
#include <unordered_map>

class MultiRobotObjectDetectionMerger : public rclcpp::Node
{
public:
  MultiRobotObjectDetectionMerger();

private:
  custom_messages::msg::DetectedObjects::SharedPtr merged_detected_objects;
  std::vector<rclcpp::Subscription<custom_messages::msg::DetectedObjects>::SharedPtr> subscriptions_;

  double thres_min_distance;
  int robot_number_merged_objects;
  std::vector<double> human_cluster_threshold;
  
  void detectedObjectCallback(const custom_messages::msg::DetectedObjects::SharedPtr msg, int robot_num);
  sensor_msgs::msg::PointCloud2::SharedPtr concatPointClouds(const sensor_msgs::msg::PointCloud2::SharedPtr cloud1, const sensor_msgs::msg::PointCloud2::SharedPtr cloud2);
  sensor_msgs::msg::PointCloud2 concatPointClouds(const sensor_msgs::msg::PointCloud2& cloud1, const sensor_msgs::msg::PointCloud2& cloud2);
  bool specifyHumanFromCloud(const sensor_msgs::msg::PointCloud2 &cloud);
  double calculateDistance(const autoware_auto_perception_msgs::msg::DetectedObject& obj1, const autoware_auto_perception_msgs::msg::DetectedObject& obj2);
  void timerCallback();
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_publisher;
  rclcpp::Publisher<custom_messages::msg::DetectedObjects>::SharedPtr detected_objects_publisher;

  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;

  std::unordered_map<int, std::shared_ptr<custom_messages::msg::DetectedObjects>> data;
};

#endif  // MULTI_ROBOT_OBJECT_DETECTION_MERGER_HPP_