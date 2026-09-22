#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/srv/clear_costmap_except_region.hpp>
#include <chrono>
#include <thread>

class CostmapClearer : public rclcpp::Node
{
public:
    CostmapClearer() : Node("costmap_clearer")
    {
        // 创建全局代价地图服务客户端
        global_client_ = this->create_client<nav2_msgs::srv::ClearCostmapExceptRegion>(
            "/global_costmap/clear_except_global_costmap");
        while (!global_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the global service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Global service not available, waiting again...");
        }

        // 创建局部代价地图服务客户端
        local_client_ = this->create_client<nav2_msgs::srv::ClearCostmapExceptRegion>(
            "/local_costmap/clear_except_local_costmap");
        while (!local_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the local service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Local service not available, waiting again...");
        }
    }

    void run()
    {
        while (rclcpp::ok())
        {
            auto request = std::make_shared<nav2_msgs::srv::ClearCostmapExceptRegion::Request>();
            // 设置重置距离为 1m
            request->reset_distance = 1.0;

            // 调用全局代价地图服务
            auto global_result = global_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), global_result) !=rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Global service call failed");
            }
            // 调用局部代价地图服务
            auto local_result = local_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), local_result) !=rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Local service call failed");
            }

            // 睡眠 5 秒
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::ClearCostmapExceptRegion>::SharedPtr global_client_;
    rclcpp::Client<nav2_msgs::srv::ClearCostmapExceptRegion>::SharedPtr local_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto clearer = std::make_shared<CostmapClearer>();
    clearer->run();
    rclcpp::shutdown();
    return 0;
}    


