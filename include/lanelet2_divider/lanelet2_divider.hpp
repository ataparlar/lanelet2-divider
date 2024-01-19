//
// Created by ataparlar on 04.01.2024.
//

#ifndef BUILD_LANELET2_DIVIDER_HPP
#define BUILD_LANELET2_DIVIDER_HPP

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace lanelet2_divider {


class Lanelet2Divider : public rclcpp::Node
{
public:
    explicit Lanelet2Divider(const rclcpp::NodeOptions & options);

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_100km_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_10km_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_points_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_polygons_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_map_lanes_;

    bool origin_init_ = false;
};
}

#endif  // BUILD_LANELET2_DIVIDER_HPP
