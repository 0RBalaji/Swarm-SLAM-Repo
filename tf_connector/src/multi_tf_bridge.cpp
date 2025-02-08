#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class MultiTfBridge : public rclcpp::Node {
public:
    MultiTfBridge() : Node("multi_tf_bridge") {
        // Declare and get parameter
        this->declare_parameter<std::vector<std::string>>("robot_namespaces");
        if (!this->get_parameter("robot_namespaces", robot_namespaces_)) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_namespaces' not found!");
            rclcpp::shutdown();
            return;
        }
        if (robot_namespaces_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_namespaces' is empty.");
            rclcpp::shutdown();
            return;
        }

        try{
            // Initialize ROS entities only if parameters are valid
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            RCLCPP_INFO(this->get_logger(), "GInitialised............");

            for (const auto& ns : robot_namespaces_) {
                auto buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
                auto options = rclcpp::SubscriptionOptions();
                options.qos_overriding_options = rclcpp::QosOverridingOptions();

                // Subscribe to namespaced TF topics (e.g., /botA/tf, /botB/tf)
                auto listener = std::make_shared<tf2_ros::TransformListener>(
                    *buffer, this, 
                    false, 
                    tf2_ros::DynamicListenerQoS(),
                    options,
                    ns + "/tf",      // Remap to namespaced topics
                    ns + "/tf_static"
                );
                RCLCPP_INFO(this->get_logger(), "Listent");
                
                tf_buffers_[ns] = buffer;

                auto timer = this->create_wall_timer(
                    100ms,
                    [this, ns]() { bridge_tf(ns); }
                );
                timers_.push_back(timer);
            }

            publish_world_transforms();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Initialization failed: %s", e.what());
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "Going IN");
    }


private:
    void bridge_tf(const std::string& ns) {
        try {
            // Get the robot's internal map->odom transform from its isolated TF tree
            auto map_to_odom = tf_buffers_[ns]->lookupTransform(
                "map", "odom", tf2::TimePointZero
            );

            // Republish with namespaced frames to the global TF tree
            geometry_msgs::msg::TransformStamped map_to_odom_global;
            map_to_odom_global.header = map_to_odom.header;
            map_to_odom_global.header.frame_id = ns + "/map";
            map_to_odom_global.child_frame_id = ns + "/odom";
            map_to_odom_global.transform = map_to_odom.transform;

            tf_broadcaster_->sendTransform(map_to_odom_global);
            RCLCPP_INFO(this->get_logger(), "I guess I sent the map -> odom tansform just now");
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF bridge error for %s: %s", ns.c_str(), ex.what());
        }
    }

    void publish_world_transforms() {
        for (const auto& ns : robot_namespaces_) {
            geometry_msgs::msg::TransformStamped world_to_map;
            world_to_map.header.stamp = this->now();
            world_to_map.header.frame_id = "world";
            world_to_map.child_frame_id = ns + "/map";
            // Adjust these values to position each robot relative to world
            // 
            // 
            // 
            // ADD A LISTNER & LET IT HAVE THAT DATA FOR 'X', 'Y' & 'Z', ALSO ROTATION
            // 
            // 
            // 
            world_to_map.transform.translation.x = (ns == "botA") ? 0.0 : 5.0;
            world_to_map.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(world_to_map);
            RCLCPP_INFO(this->get_logger(), "I guess I sent the world -> map tansform just now");
        }
    }

    std::vector<std::string> robot_namespaces_;
    std::unordered_map<std::string, std::shared_ptr<tf2_ros::Buffer>> tf_buffers_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiTfBridge>();
    if (rclcpp::ok()) { // Check if ROS context is still valid
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}