#include "laser_filter_processor.hpp"

LaserFilterProcessor::LaserFilterProcessor(const std::string &node_name)
    : Node(node_name) {
    // 参数初始化
    this->declare_parameter<double>("angle_min", -180.0); // 屏蔽范围起始角度 (单位: 度)
    this->declare_parameter<double>("angle_max", 180.0);  // 屏蔽范围结束角度 (单位: 度)

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_multi", 10, std::bind(&LaserFilterProcessor::laserCallback, this, std::placeholders::_1));
    laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // 获取参数
    this->get_parameter("angle_min", angle_min_);
    this->get_parameter("angle_max", angle_max_);

    // 转换角度为弧度
    angle_min_rad_ = angle_min_ * M_PI / 180.0;
    angle_max_rad_ = angle_max_ * M_PI / 180.0;

    RCLCPP_INFO(this->get_logger(), "LaserFilterProcessor initialized. Filtering angles: [%f, %f] degrees",
                angle_min_, angle_max_);
}

void LaserFilterProcessor::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto filtered_scan = *msg; // 创建一个副本用于修改

    // 遍历激光雷达数据
    for (size_t i = 0; i < filtered_scan.ranges.size(); ++i) {
        // 计算当前角度
        double angle = filtered_scan.angle_min + i * filtered_scan.angle_increment;

        // 如果角度在屏蔽范围内且距离小于 10cm，设置为无效数据
        if (angle >= angle_min_rad_ && angle <= angle_max_rad_ && filtered_scan.ranges[i] < 0.18) {
            filtered_scan.ranges[i] = std::numeric_limits<float>::infinity(); // 无效数据
        }
    }

    // 发布过滤后的数据
    laser_pub_->publish(filtered_scan);
}

