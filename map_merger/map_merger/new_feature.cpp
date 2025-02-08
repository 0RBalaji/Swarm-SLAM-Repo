#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <algorithm>
#include <stdexcept>

class MapUpdateBroadcast : public rclcpp::Node
{
public:
    MapUpdateBroadcast() : Node("map_update_broadcast")
    {
        // Declare parameters
        this->declare_parameter<std::string>("namespace", "");
        this->declare_parameter<int>("logging_level", rclcpp::Logger::Level::Info);
        
        namespace_ = this->get_parameter("namespace").as_string();
        int log_level = this->get_parameter("logging_level").as_int();
        rclcpp::Logger::Level level = static_cast<rclcpp::Logger::Level>(log_level);
        rclcpp::Logger logger = this->get_logger();
        rclcpp::Logger::set_logger_level(logger.get_name(), level);

        if (namespace_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Namespace parameter is required. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Subscribe to the map topic
        std::string map_topic = "/" + namespace_ + "/map";
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic, 10, [this](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
                this->process_map(std::move(msg));
            });

        // Publish to the new_feature topic
        std::string new_feature_topic = "/" + namespace_ + "/new_feature";
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(new_feature_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Node initialized. Subscribed to: %s", map_topic.c_str());
    }

private:
    void process_map(nav_msgs::msg::OccupancyGrid::UniquePtr new_map)
    {
        try
        {
            if (!previous_map_)
            {
                previous_map_ = std::move(new_map);
                RCLCPP_DEBUG(this->get_logger(), "Stored the initial map.");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Processing new map update.");

            // Align maps
            auto [aligned_prev, aligned_new, info] = align_maps(*previous_map_, *new_map);

            // Detect changes
            auto delta = detect_changes(aligned_prev, aligned_new);
            if (std::all_of(delta.begin(), delta.end(), [](int8_t val) { return val == -1; }))
            {
                RCLCPP_DEBUG(this->get_logger(), "No changes detected.");
                return;
            }

            // Crop the delta map
            auto [cropped_delta, cropped_info] = crop_delta(delta, info);

            // Publish the new feature
            auto delta_msg = create_occupancy_grid(cropped_delta, cropped_info, new_map->header);
            publisher_->publish(std::move(delta_msg));
            RCLCPP_INFO(this->get_logger(), "Published map changes.");

            // Update the previous map
            previous_map_ = std::move(new_map);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing map update: %s", e.what());
        }
    }

    std::tuple<std::vector<int8_t>, std::vector<int8_t>, nav_msgs::msg::MapMetaData> align_maps(
        const nav_msgs::msg::OccupancyGrid &prev_map, const nav_msgs::msg::OccupancyGrid &new_map)
    {
        const auto &prev_info = prev_map.info;
        const auto &new_info = new_map.info;

        double min_x = std::min(prev_info.origin.position.x, new_info.origin.position.x);
        double min_y = std::min(prev_info.origin.position.y, new_info.origin.position.y);
        double max_x = std::max(prev_info.origin.position.x + prev_info.width * prev_info.resolution,
                                new_info.origin.position.x + new_info.width * new_info.resolution);
        double max_y = std::max(prev_info.origin.position.y + prev_info.height * prev_info.resolution,
                                new_info.origin.position.y + new_info.height * new_info.resolution);

        size_t width = static_cast<size_t>(std::ceil((max_x - min_x) / prev_info.resolution));
        size_t height = static_cast<size_t>(std::ceil((max_y - min_y) / prev_info.resolution));

        std::vector<int8_t> aligned_prev(width * height, -1);
        std::vector<int8_t> aligned_new(width * height, -1);

        copy_to_aligned(prev_map.data, prev_info, aligned_prev, min_x, min_y, width);
        copy_to_aligned(new_map.data, new_info, aligned_new, min_x, min_y, width);

        nav_msgs::msg::MapMetaData info;
        info.width = width;
        info.height = height;
        info.resolution = prev_info.resolution;
        info.origin.position.x = min_x;
        info.origin.position.y = min_y;

        return {aligned_prev, aligned_new, info};
    }

    void copy_to_aligned(const std::vector<int8_t> &map_data, const nav_msgs::msg::MapMetaData &info,
                         std::vector<int8_t> &aligned, double min_x, double min_y, size_t width)
    {
        size_t origin_x = static_cast<size_t>((info.origin.position.x - min_x) / info.resolution);
        size_t origin_y = static_cast<size_t>((info.origin.position.y - min_y) / info.resolution);

        for (size_t y = 0; y < info.height; ++y)
        {
            std::copy_n(&map_data[y * info.width], info.width,
                        &aligned[(origin_y + y) * width + origin_x]);
        }
    }

    std::vector<int8_t> detect_changes(const std::vector<int8_t> &prev, const std::vector<int8_t> &current)
    {
        std::vector<int8_t> delta(prev.size());
        std::transform(prev.begin(), prev.end(), current.begin(), delta.begin(),
                       [](int8_t prev_val, int8_t curr_val) {
                           return (prev_val != curr_val) ? curr_val : -1;
                       });
        return delta;
    }

    std::pair<std::vector<int8_t>, nav_msgs::msg::MapMetaData> crop_delta(
        const std::vector<int8_t> &delta, const nav_msgs::msg::MapMetaData &info)
    {
        size_t min_x = info.width, max_x = 0, min_y = info.height, max_y = 0;
        for (size_t y = 0; y < info.height; ++y)
        {
            for (size_t x = 0; x < info.width; ++x)
            {
                if (delta[y * info.width + x] != -1)
                {
                    min_x = std::min(min_x, x);
                    max_x = std::max(max_x, x);
                    min_y = std::min(min_y, y);
                    max_y = std::max(max_y, y);
                }
            }
        }

        size_t cropped_width = max_x - min_x + 1;
        size_t cropped_height = max_y - min_y + 1;
        std::vector<int8_t> cropped(cropped_width * cropped_height, -1);

        for (size_t y = min_y; y <= max_y; ++y)
        {
            std::copy_n(&delta[y * info.width + min_x], cropped_width,
                        &cropped[(y - min_y) * cropped_width]);
        }

        nav_msgs::msg::MapMetaData cropped_info = info;
        cropped_info.width = cropped_width;
        cropped_info.height = cropped_height;
        cropped_info.origin.position.x += min_x * info.resolution;
        cropped_info.origin.position.y += min_y * info.resolution;

        return {cropped, cropped_info};
    }

    nav_msgs::msg::OccupancyGrid::UniquePtr create_occupancy_grid(
        const std::vector<int8_t> &grid, const nav_msgs::msg::MapMetaData &info,
        const std_msgs::msg::Header &header)
    {
        auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
        msg->header = header;
        msg->info = info;
        msg->data = grid;
        return msg;
    }

    std::string namespace_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    nav_msgs::msg::OccupancyGrid::UniquePtr previous_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapUpdateBroadcast>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
