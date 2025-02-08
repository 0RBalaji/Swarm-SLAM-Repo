#include <memory>
#include <functional>
#include <string>
#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class FixedFrameBroadcaster : public rclcpp::Node {
public:
    FixedFrameBroadcaster() : Node("worldframe_game") {
        
        this->declare_parameter<std::string>("namespace", "", rcl_interfaces::msg::ParameterDescriptor());
        this->get_parameter("namespace", namespace_);

        // Check if the namespace is provided
        if (namespace_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Namespace parameter is required.");
            rclcpp::shutdown();
            return;
        }
        
        // Initialize TF buffer and listener to read the robot's isolated TF tree
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

        // Broadcaster for global TF tree (world -> virtual namespaced map)
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Timer to periodically bridge the isolated TF data to the global tree
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FixedFrameBroadcaster::bridge_tf, this)
        );
    }

private:

void bridge_tf() {
        try {
            // Create virtual namespaced frames in the global TF tree
            geometry_msgs::msg::TransformStamped world_to_virtual_map;
            world_to_virtual_map.header.stamp = this->now();
            world_to_virtual_map.header.frame_id = "world";
            world_to_virtual_map.child_frame_id = namespace_ + "/map"; // e.g., botA/map
            
            // Set your desired static transform from world to virtual map here
            world_to_virtual_map.transform.translation.x = 0.0; // Adjust as needed
            world_to_virtual_map.transform.translation.y = 0.0;
            world_to_virtual_map.transform.translation.z = 0.0;
            world_to_virtual_map.transform.rotation.w = 1.0;

            // Publish to global TF tree
            tf_broadcaster_->sendTransform(world_to_virtual_map);

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF bridge error: %s", ex.what());
        }
    }
    std::string namespace_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]){
    auto logger = rclcpp::get_logger("logger");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
    rclcpp::shutdown();
    return 0;
}