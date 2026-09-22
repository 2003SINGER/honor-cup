#include "rclcpp/rclcpp.hpp"
#include "arm_interface/srv/arm_kinemarics.hpp"
//not ros
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "rcutils/logging.h"
#include "arm_kin/fk_ik_kin.h"

using namespace KDL;
using namespace std;
// 弧度转角度
const float RA2DE = 180.0f / M_PI;
// 角度转弧度
const float DE2RA = M_PI / 180.0f;
const char *urdf_file = "/home/jetson/yahboomcar_ws/src/yahboom_M3Pro_description/urdf/M3Pro.urdf";
int a = 0;



void handle_service(
  const std::shared_ptr<arm_interface::srv::ArmKinemarics::Request> request,
  std::shared_ptr<arm_interface::srv::ArmKinemarics::Response> response)
{
  cout<<"-----------------"<<endl;
  cout<<request->kin_name<<endl;
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I got it.");
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "kin name is %f.",request->tar_z);
  
  
  
      if (request->kin_name == "fk") {
        double joints[]{request->cur_joint1, request->cur_joint2, request->cur_joint3, request->cur_joint4,
                        request->cur_joint5,request->cur_joint6};
        // 定义目标关节角容器
        vector<double> initjoints;
        // 定义位姿容器
        vector<double> initpos;
        // 目标关节角度单位转换,由角度转换成弧度
        for (int i = 0; i < 6; ++i) initjoints.push_back((joints[i] - 90) * DE2RA);
        arm_getFK(urdf_file, initjoints, initpos);
        response->x = initpos.at(0);
        response->y = initpos.at(1);
        response->z = initpos.at(2);
        response->roll = initpos.at(3);
        response->pitch = initpos.at(4);
        response->yaw = initpos.at(5);
        cout<<"-----------------"<<endl;
    }
    
    if (request->kin_name == "ik") {
        // 夹抓长度
        double tool_param = 0.12;
        // 抓取的位姿
        double Roll = request->roll ;
        double Pitch = request->pitch;
        double Yaw = request->yaw ;
        double x=request->tar_x;
        double y=request->tar_y;
        double z=request->tar_z;
        // 末端位置(单位: m)
        double xyz[]{x, y, z};
        cout << x << y << z << endl;
        // 末端姿态(单位: 弧度)
        //double rpy[]{Roll * DE2RA, Pitch * DE2RA, Yaw * DE2RA};
    	double rpy[]{Roll , Pitch, Yaw };
        // 创建输出角度容器
        vector<double> outjoints;
        // 创建末端位置容器
        vector<double> targetXYZ;
        // 创建末端姿态容器
        vector<double> targetRPY;
        for (int k = 0; k < 3; ++k) targetXYZ.push_back(xyz[k]);
        for (int l = 0; l < 3; ++l) targetRPY.push_back(rpy[l]);
        // 反解求到达目标点的各关节角度
        arm_getIK(urdf_file, targetXYZ, targetRPY, outjoints);
        // 打印反解结果
        for (int i = 0; i < 5; i++) cout << (outjoints.at(i) * RA2DE) + 90 << ",";
        cout << endl;
        a++;
        response->joint1 = (outjoints.at(0) * RA2DE) + 90;
        response->joint2 = (outjoints.at(1) * RA2DE) + 90;
        response->joint3 = (outjoints.at(2) * RA2DE) + 90;
        response->joint4 = (outjoints.at(3) * RA2DE) + 90;
        response->joint5 = (outjoints.at(4) * RA2DE) + 90;
        cout<<"-----------------"<<endl;
    }
  
  

}

int main(int argc,char **argv)
{
	rclcpp::init(argc, argv);

    rcutils_logging_set_logger_level("kdl_parser", RCUTILS_LOG_SEVERITY_ERROR);

	auto node = rclcpp::Node::make_shared("kinemarics_arm");
	auto service = node->create_service<arm_interface::srv::ArmKinemarics>("get_kinemarics", handle_service);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;	
}

