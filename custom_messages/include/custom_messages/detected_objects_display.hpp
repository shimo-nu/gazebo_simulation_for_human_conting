#ifndef DETECTED_OBJECTS_DISPLAY_HPP
#define DETECTED_OBJECTS_DISPLAY_HPP

#include <rviz_common/message_filter_display.hpp>
#include <custom_messages/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>

namespace rviz_plugins
{
  class DetectedObjectsDisplay : public rviz_common::MessageFilterDisplay<custom_messages::msg::DetectedObjects>
  {
    Q_OBJECT
  public:
    DetectedObjectsDisplay();
    ~DetectedObjectsDisplay() override;

  protected:
    void onInitialize() override;
    void reset() override;
    void processMessage(const custom_messages::msg::DetectedObjects::ConstSharedPtr msg) override;

  private:
    std::shared_ptr<rviz_rendering::PointCloud> point_cloud_;
  };
} // namespace rviz_plugins

#endif // DETECTED_OBJECTS_DISPLAY_HPP
