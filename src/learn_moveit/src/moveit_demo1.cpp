#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char * argv[])
{
  // 1. 初始化 ROS 2 节点
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("moveit_demo1");
  
  // 创建一个日志记录器，方便打印信息
  auto const logger = rclcpp::get_logger("moveit_demo1");

  // 2. 创建 MoveGroupInterface
  // ROS 2 中需要传入 node 指针
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");
  auto move_group_hand = MoveGroupInterface(node, "hand");

  // 3. 设置目标位姿

  //动态输入坐标
  double x,y,z;
  std::cout << "输入目标 X 坐标（例如 0.3)";
  std::cin >> x;
  std::cout << "输入目标 Y 坐标（例如 0.3)";
  std::cin >> y;
  std::cout << "输入目标 Z 坐标（例如 0.3)";
  std::cin >> z;
  geometry_msgs::msg::Pose target_pose;

  
  // 设置姿态 (垂直向下)
  tf2::Quaternion q;
  q.setRPY(0, M_PI, 0); // Panda 机械臂通常绕 Y 转 180 度是末端垂直向下
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();

  // 设置位置 
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  RCLCPP_INFO(logger,"目标位置设定：[X:%.2f,Y:%.2f,Z:%.2f]",x,y,z);
  move_group_interface.setPoseTarget(target_pose);

  // 4. 规划与移动
  RCLCPP_INFO(logger, "开始规划...");
  
  // 创建规划方案
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(logger, "规划成功，开始执行移动！");

    RCLCPP_INFO(logger,"准备打开夹爪...");
    move_group_hand.setNamedTarget("open");
    move_group_hand.move();
    RCLCPP_INFO(logger,"夹爪已打开");

    move_group_interface.execute(my_plan);

    RCLCPP_INFO(logger,"准备闭合夹爪...");
    move_group_hand.setNamedTarget("close");
    move_group_hand.move();
    RCLCPP_INFO(logger,"夹爪已闭合(抓取完成)");
  } else {
    RCLCPP_ERROR(logger, "规划失败！请检查目标点是否在工作空间内，或者是否发生碰撞。");
  }

  rclcpp::shutdown();
  return 0;
}