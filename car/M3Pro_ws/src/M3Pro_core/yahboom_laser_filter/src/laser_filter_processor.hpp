#ifndef LASER_FILTER_PROCESSOR_HPP
#define LASER_FILTER_PROCESSOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>
#include <limits>
#include <cmath>

class LaserFilterProcessor : public rclcpp::Node {
public:
    LaserFilterProcessor(const std::string &node_name = "laser_filter_processor");
    
private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;

    double angle_min_;
    double angle_max_;
    double angle_min_rad_;
    double angle_max_rad_;
};

#endif // LASER_FILTER_PROCESSOR_HPP
