#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

class CartesianPathPlanning : public rclcpp::Node
{
public:
  CartesianPathPlanning()
    : Node("cartesian_path_planning")
    {
    	RCLCPP_INFO(this->get_logger(), "Initializing CartesianPathPlanning.");
    }
  void initialize()
  {
    int max_attempts = 5;  // 最大规划尝试次数
  	int attempt_count = 0;  // 当前尝试次数
    // 使用 RobotModelLoader 加载机器人模型
    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(),"robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    // 初始化 MoveGroupInterface
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm_group");
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    move_group_interface_->setNumPlanningAttempts(10);  // 设置最大规划尝试次数为 10
    move_group_interface_->setPlanningTime(5.0);         // 设置每次规划的最大时间为 5 秒
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("arm_group");

    
    moveit_visual_tools::MoveItVisualTools visual_tools_(shared_from_this(), "base_link", "arm_group",
                                                    move_group_interface_->getRobotModel());

	while (attempt_count < max_attempts)
	{
		attempt_count++;
		std::vector<double> target_joints = {0, -0.49, -0.17, 0.86, 0}; // 目标关节角度（单位：弧度）
		// 设置目标关节空间值
		move_group_interface_->setJointValueTarget(target_joints);
		    // 规划路径
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
		if (success)
    	{
          // 在RViz中可视化轨迹
      	//visual_tools_.publishTrajectoryLine(my_plan.trajectory_,move_group_interface_->getRobotModel()->getLinkModel("Gripping"),joint_model_group,rviz_visual_tools::LIME_GREEN);
      	//visual_tools_.trigger();
      	RCLCPP_INFO(this->get_logger(), "Planning succeeded, moving the arm.");
      	move_group_interface_->execute(my_plan);
      	break;
    	}
    	else
    	{
      	RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    	}
	}


    // 定义一系列目标位姿
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose;

    target_pose.position.x = 0.10755;
    target_pose.position.y = -1.35847e-05;
    target_pose.position.z = 0.400775;
    target_pose.orientation.w = 1.0;
    waypoints.push_back(target_pose);

    geometry_msgs::msg::Pose target_pose_2;
    target_pose_2.position.x = 0.126933;
    target_pose_2.position.y = 0.000215801;
    target_pose_2.position.z = 0.384009;
    target_pose_2.orientation.w = 1.0;
    waypoints.push_back(target_pose_2);
    
    geometry_msgs::msg::Pose target_pose_3;
    target_pose_2.position.x = 0.146933;
    target_pose_2.position.y = 0.000215801;
    target_pose_2.position.z = 0.364009;
    target_pose_2.orientation.w = 1.0;
    waypoints.push_back(target_pose_3);
    
    geometry_msgs::msg::Pose target_pose_4;
    target_pose_2.position.x = 0.106933;
    target_pose_2.position.y = 0.000215801;
    target_pose_2.position.z = 0.304009;
    target_pose_2.orientation.w = 1.0;
    waypoints.push_back(target_pose_4);
    

    

    // 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    double jump_threshold = 0.0;
    double eef_step = 0.01;
    int maxtries = 1000;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数   
    double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg); 
    /*while (attempts < maxtries)
    {
    	fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg,true);
		attempts++;
		if (attempts % 10 == 0) 
		{
			RCLCPP_INFO(this->get_logger(), "Still trying after %d attempts...", attempts);
		}   
    }*/
    

    if (fraction > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Cartesian path planned successfully (%.2f%% achieved)", fraction * 100.0);

      // 将 moveit_msgs::msg::RobotTrajectory 转换为 robot_trajectory::RobotTrajectory
      robot_trajectory::RobotTrajectory trajectory(move_group_interface_->getRobotModel(), move_group_interface_->getName());

      // 在RViz中可视化轨迹
      visual_tools_.publishTrajectoryLine(trajectory_msg,move_group_interface_->getRobotModel()->getLinkModel("Gripping"),joint_model_group,rviz_visual_tools::LIME_GREEN);
      visual_tools_.trigger();

      // 执行规划好的路径
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory_msg;
      move_group_interface_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to compute Cartesian path");
    }
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianPathPlanning>();
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // 启动异步 Spinner
  node->initialize();
  executor.spin();

  //rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
