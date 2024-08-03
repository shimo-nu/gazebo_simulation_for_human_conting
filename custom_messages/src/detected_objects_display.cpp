#include "detected_objects_display.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace rviz_plugins
{
  DetectedObjectsDisplay::DetectedObjectsDisplay()
  {
  }

  DetectedObjectsDisplay::~DetectedObjectsDisplay()
  {
  }

  void DetectedObjectsDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    point_cloud_ = std::make_shared<rviz_rendering::PointCloud>();
    scene_node_->attachObject(point_cloud_.get());
  }

  void DetectedObjectsDisplay::reset()
  {
    MFDClass::reset();
    point_cloud_->clear();
  }

  void DetectedObjectsDisplay::processMessage(const custom_messages::msg::DetectedObjects::ConstSharedPtr msg)
  {
    point_cloud_->clear();

    for (const auto &detected_object : msg->detected_objects)
    {
      sensor_msgs::msg::PointCloud2 cluster = detected_object.cluster;

      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::fromROSMsg(cluster, pcl_cloud);

      for (const auto &point : pcl_cloud.points)
      {
        rviz_rendering::PointCloud::Point new_point;
        new_point.position.x = point.x;
        new_point.position.y = point.y;
        new_point.position.z = point.z;
        new_point.setColor(1.0, 0.0, 0.0); // Set color (red in this example)

        point_cloud_->addPoint(new_point);
      }
    }

    context_->queueRender();
  }

} // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::DetectedObjectsDisplay, rviz_common::Display)
