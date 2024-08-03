#include "rclcpp/rclcpp.hpp"
#include "journal1-sim/multi_robot_object_detection_merger.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include "autoware_auto_perception_msgs/msg/object_classification.hpp"
// #include 


MultiRobotObjectDetectionMerger::MultiRobotObjectDetectionMerger()
 : Node("multi_robot_object_detection_merger")
{
  int robot_num = this->declare_parameter<int>("robot_num", 2);
  human_cluster_threshold = this->declare_parameter<std::vector<double>>("human_cluster_threshold", std::vector<double>{0.1, 0.8, 0.1, 0.8, 0.5, 2.0});

  int timer_hz = this->declare_parameter<int>("timer_hz", 20);

  thres_min_distance = this->declare_parameter<double>("thres_min_distance", 1.0);
  
  for (int i = 1; i <= robot_num; i++) {
    std::string topic = "/robot" + std::to_string(i) + "/perception/object_detection/merged_objects";
    auto robot_sub = this->create_subscription<custom_messages::msg::DetectedObjects>(
      topic, 10, [this, i](const custom_messages::msg::DetectedObjects::SharedPtr msg) {
        this->detectedObjectCallback(msg, i);
      });
    subscriptions_.push_back(robot_sub);
  }

  cluster_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_objects_cluster", 1);
  detected_objects_publisher = this->create_publisher<custom_messages::msg::DetectedObjects>("/merged_objects", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds((int)1000 / timer_hz), std::bind(&MultiRobotObjectDetectionMerger::timerCallback, this));
}


void MultiRobotObjectDetectionMerger::detectedObjectCallback(const custom_messages::msg::DetectedObjects::SharedPtr msg, int robot_num)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data[robot_num] = msg;
    // RCLCPP_INFO(this->get_logger(), "Data from robot%d stored successfully", robot_num);
  }

  if (data[robot_num] == nullptr) {
    // RCLCPP_ERROR(this->get_logger(), "Stored data for robot%d is nullptr", robot_num);
  }
}

void MultiRobotObjectDetectionMerger::timerCallback(){
  std::lock_guard<std::mutex> lock(mutex_);
  // Use steady clock for time comparison
  // RCLCPP_INFO(this->get_logger(), "Timer callback");
  if (data.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "No data received");
    return;
  }
  else if (data.size() == 1) {
    // RCLCPP_INFO(this->get_logger(), "Only one robot detected objects");
    // auto robot_number = data.begin()->first;
    // detected_objects_publisher->publish(*data[robot_number]);
    return;
  } else if (data.size() < 2 || data[1] == nullptr || data[2] == nullptr) {
    // RCLCPP_INFO(this->get_logger(), "No data stored");
    return; 
  }

  // RCLCPP_INFO(this->get_logger(), "Timer callback1");
  // RCLCPP_INFO(this->get_logger(), "Timer callback2");


  // RCLCPP_INFO(this->get_logger(), "Data size: %d", data.size());
  // RCLCPP_INFO(this->get_logger(), data[1]->header.frame_id.c_str());
  auto time_diff = rclcpp::Time(data[1]->header.stamp) - rclcpp::Time(data[2]->header.stamp);

  if (time_diff.seconds() > 1.0) {
    RCLCPP_WARN(this->get_logger(), "Time difference between two robots is too large : %f", time_diff.seconds());
    return;
  }

  custom_messages::msg::DetectedObjects::SharedPtr new_merged_detected_objects = std::make_shared<custom_messages::msg::DetectedObjects>();
  sensor_msgs::msg::PointCloud2::SharedPtr new_merged_cluster = std::make_shared<sensor_msgs::msg::PointCloud2>();

  RCLCPP_INFO(this->get_logger(), "Process Start");


  for (auto& first_robot_detected_objects : data[1]->detected_object) {
    double min_distance = std::numeric_limits<double>::max();
    custom_messages::msg::DetectedObject closest_merged_object;

    // RCLCPP_INFO(this->get_logger(), "Detected Object");
    for (auto& second_robot_detected_objects : data[2]->detected_object) {
      double distance = calculateDistance(first_robot_detected_objects.detected_object, second_robot_detected_objects.detected_object);
      if (distance < min_distance) {
        min_distance = distance;
        closest_merged_object = second_robot_detected_objects;
      } 
    }

    RCLCPP_INFO(this->get_logger(), "Specify Object");
    if (min_distance < thres_min_distance) {
      // The closest object is found
      RCLCPP_INFO(this->get_logger(), "The closest object is found");
      
      if (closest_merged_object.detected_object.classification[0].label == 7 && first_robot_detected_objects.detected_object.classification[0].label == 7) {
        // RCLCPP_INFO(this->get_logger(), "Two Robot found same pedestrian");
        new_merged_detected_objects->detected_object.push_back(first_robot_detected_objects);
      } else {
        // RCLCPP_INFO(this->get_logger(), "Two Robot found different object");
        sensor_msgs::msg::PointCloud2 concatenated_cloud = concatPointClouds(
          closest_merged_object.cluster, 
          first_robot_detected_objects.cluster);
        // RCLCPP_INFO(this->get_logger(), "Concatenated PCD");
        autoware_auto_perception_msgs::msg::ObjectClassification classification;
        classification.label = (closest_merged_object.detected_object.classification[0].label == 7 || first_robot_detected_objects.detected_object.classification[0].label == 7) ? 
          autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN : 
          autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;


        // RCLCPP_INFO(this->get_logger(), "Make new detected object");
        custom_messages::msg::DetectedObject new_merged_detected_object;
        new_merged_detected_object.header = first_robot_detected_objects.header;

        new_merged_detected_object.cluster = concatenated_cloud;
        new_merged_detected_object.cluster.header = first_robot_detected_objects.cluster.header;

        new_merged_detected_object.detected_object = first_robot_detected_objects.detected_object;
        new_merged_detected_object.detected_object.classification.emplace_back(std::move(classification));
        new_merged_detected_objects->detected_object.push_back(new_merged_detected_object);

        // RCLCPP_INFO(this->get_logger(), "concat new merged cluster");
        new_merged_cluster = concatPointClouds(new_merged_cluster, std::make_shared<sensor_msgs::msg::PointCloud2>(concatenated_cloud));
      }
    } else {
      // The closest object is not found
      RCLCPP_INFO(this->get_logger(), "The closest object is not found");
      sensor_msgs::msg::PointCloud2 concatenated_cloud = concatPointClouds(
        closest_merged_object.cluster, 
        first_robot_detected_objects.cluster);
      custom_messages::msg::DetectedObject new_merged_detected_object;
      new_merged_detected_object.header = first_robot_detected_objects.header;

      new_merged_detected_object.cluster = concatenated_cloud;
      new_merged_detected_object.cluster.header = first_robot_detected_objects.cluster.header;


      autoware_auto_perception_msgs::msg::ObjectClassification label;
      if (specifyHumanFromCloud(concatenated_cloud)) {
        label.label = autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
      } else {
        label.label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
      }

      new_merged_detected_object.detected_object = first_robot_detected_objects.detected_object;
      new_merged_detected_object.detected_object.classification.emplace_back(std::move(label));
      new_merged_detected_objects->detected_object.push_back(new_merged_detected_object);

      new_merged_cluster = concatPointClouds(new_merged_cluster, std::make_shared<sensor_msgs::msg::PointCloud2>(concatenated_cloud));
    }
  }
  // merged_detected_objects = new_merged_detected_objects;
  // merged_detected_objects->header = msg->header;
  // merged_detected_objects->header.frame_id = "map";

  RCLCPP_INFO(this->get_logger(), "Publishing merged objects");

  new_merged_cluster->header = data[2]->header;
  new_merged_detected_objects->header = data[2]->header;

  cluster_publisher->publish(*new_merged_cluster);
  detected_objects_publisher->publish(*new_merged_detected_objects);
}


bool MultiRobotObjectDetectionMerger::
specifyHumanFromCloud(const sensor_msgs::msg::PointCloud2& cloud)
{
  RCLCPP_INFO(this->get_logger(), "Specifying human from cloud");
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud, *pcl_cloud);

  pcl::PointXYZ min_point, max_point;
  pcl::getMinMax3D(*pcl_cloud, min_point, max_point);

  double cluster_size = max_point.x - min_point.x;
  cluster_size *= (max_point.y - min_point.y);
  cluster_size *= (max_point.z - min_point.z);

  bool is_human = (cluster_size > human_cluster_threshold[0] && cluster_size < human_cluster_threshold[1] &&
            cluster_size > human_cluster_threshold[2] && cluster_size < human_cluster_threshold[3] &&
            cluster_size > human_cluster_threshold[4] && cluster_size < human_cluster_threshold[5]);
  RCLCPP_INFO(this->get_logger(), "is_human : %d", is_human);
  return is_human;

}

sensor_msgs::msg::PointCloud2::SharedPtr MultiRobotObjectDetectionMerger::concatPointClouds(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud1, 
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud2)
{
  // RCLCPP_INFO(this->get_logger(), "Concatenating two point clouds");

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud1;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud2;
  
  try {
    pcl::fromROSMsg(*cloud1, pcl_cloud1);
    pcl::fromROSMsg(*cloud2, pcl_cloud2);

  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "Error in converting point cloud to pcl");
      for (const auto& field : cloud1->fields) {
        std::cout << "Cloud1 : Field name: " << field.name << std::endl;
      }
      for (const auto& field : cloud1->fields) {
        std::cout << "Cloud2 : Field name: " << field.name << std::endl;
      }
  }
  
  pcl::PointCloud<pcl::PointXYZ> pcl_concatenated_cloud;
  pcl_concatenated_cloud = pcl_cloud1 + pcl_cloud2;


  sensor_msgs::msg::PointCloud2::SharedPtr concatenated_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(pcl_concatenated_cloud, *concatenated_cloud);


  concatenated_cloud->header = cloud1->header;

  // RCLCPP_INFO(this->get_logger(), "Concatenated");
  return concatenated_cloud;
}


sensor_msgs::msg::PointCloud2 MultiRobotObjectDetectionMerger::concatPointClouds(
  const sensor_msgs::msg::PointCloud2& cloud1, 
  const sensor_msgs::msg::PointCloud2& cloud2)
{
  // RCLCPP_INFO(this->get_logger(), "Concatenating two point clouds");

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud1;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud2;

  try {
    pcl::fromROSMsg(cloud1, pcl_cloud1);
    pcl::fromROSMsg(cloud2, pcl_cloud2);

  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "Error in converting point cloud to pcl");
      for (const auto& field : cloud1.fields) {
        std::cout << "Cloud1 : Field name: " << field.name << std::endl;
      }
      for (const auto& field : cloud1.fields) {
        std::cout << "Cloud2 : Field name: " << field.name << std::endl;
      }
  }


  pcl::PointCloud<pcl::PointXYZ> pcl_concatenated_cloud;
  pcl_concatenated_cloud = pcl_cloud1 + pcl_cloud2;

  sensor_msgs::msg::PointCloud2 concatenated_cloud;
  pcl::toROSMsg(pcl_concatenated_cloud, concatenated_cloud);

  concatenated_cloud.header = cloud1.header;

  return concatenated_cloud;
}

double MultiRobotObjectDetectionMerger::calculateDistance(const autoware_auto_perception_msgs::msg::DetectedObject& obj1, const autoware_auto_perception_msgs::msg::DetectedObject& obj2) {
  // Return the calculated distance
  // RCLCPP_INFO(this->get_logger(), "Calculating distance between two objects");
  geometry_msgs::msg::PoseWithCovariance pose1 = obj1.kinematics.pose_with_covariance;
  geometry_msgs::msg::PoseWithCovariance pose2 = obj2.kinematics.pose_with_covariance;

  double distance = std::sqrt(std::pow(pose1.pose.position.x - pose2.pose.position.x, 2) +
                              std::pow(pose1.pose.position.y - pose2.pose.position.y, 2) +
                              std::pow(pose1.pose.position.z - pose2.pose.position.z, 2));
  return distance;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiRobotObjectDetectionMerger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
