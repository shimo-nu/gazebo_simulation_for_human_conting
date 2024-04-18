#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class TrackedObjectsSubscriber : public rclcpp::Node {
public:
    TrackedObjectsSubscriber() : Node("tracked_objects_subscriber") {
        tracked_objects_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::TrackedObjects>(
            "/perception/object_recognition/tracking/objects", 10,
            std::bind(&TrackedObjectsSubscriber::onTrackedObjects, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 1);

        declare_parameter("lifetime", 0.0);

        get_parameter("lifetime", lifetime_);
    }

private:
    void onTrackedObjects(const autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr msg) {
        visualization_msgs::msg::MarkerArray marker_array;

        for (const auto& id : displayed_marker_ids_) {
          visualization_msgs::msg::Marker delete_marker;
          delete_marker.header = msg->header;
          delete_marker.ns = "tracked_objects";
          delete_marker.id = id;
          delete_marker.action = visualization_msgs::msg::Marker::DELETE;
          marker_array.markers.push_back(delete_marker);
        }
        displayed_marker_ids_.clear();

        for (const auto& object : msg->objects) {
            visualization_msgs::msg::Marker marker;
            marker.header = msg->header;
            marker.ns = "tracked_objects";
            marker.id = object.object_id.uuid[0]; // 先頭の1バイトでIDを設定
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = object.kinematics.pose_with_covariance.pose.position.x;
            marker.pose.position.y = object.kinematics.pose_with_covariance.pose.position.y;
            marker.pose.position.z = object.kinematics.pose_with_covariance.pose.position.z;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = rclcpp::Duration(lifetime_, 0);
            marker_array.markers.push_back(marker);
        }

        RCLCPP_INFO(this->get_logger(), "objects size is %d", msg->objects.size());
        RCLCPP_INFO(this->get_logger(), "displayed marker size is %d", displayed_marker_ids_.size());
        marker_pub_->publish(marker_array);
    }

    rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr tracked_objects_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    double lifetime_;
    std::vector<int> displayed_marker_ids_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackedObjectsSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}