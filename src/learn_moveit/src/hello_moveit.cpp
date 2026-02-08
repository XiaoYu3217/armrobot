#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // 1. 创建一个碰撞物体对象
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "panda_link0"; // 参考坐标系：机器人的基座
  collision_object.id = "box1"; // 障碍物名字

  // 2. 定义物体的形状（一个盒子）
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4; // x 长
  primitive.dimensions[1] = 0.1; // y 宽
  primitive.dimensions[2] = 0.4; // z 高

  // 3. 定义物体的位置
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5; // 在机器人前方 0.5米
  box_pose.position.y = 0.0; // 正前方
  box_pose.position.z = 0.2; // 高度

  // 4. 组装物体
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD; // 操作类型：添加

  // 5. 提交给规划场景
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  
  // applyCollisionObjects 把物体发布到 MoveIt 的场景中
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  // 1. 初始化 ROS 2 节点
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hello_moveit");

  // 创建一个多线程执行器
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // 2. 创建 MoveGroupInterface
  static const std::string PLANNING_GROUP = "panda_arm"; 
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  //创建 PlanningSceneInterface 实例
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // 调用函数添加障碍物
  addCollisionObjects(planning_scene_interface);
  // 等待生效
  RCLCPP_INFO(node->get_logger(), "正在添加障碍物...");
  rclcpp::sleep_for(std::chrono::seconds(2));

  //3.设置目标：使用笛卡尔坐标（Pose)

  //获取当前的末端姿态 （包含xyz和朝向）
  auto current_pose = move_group.getCurrentPose().pose;
  // 打印当前位置方便查看
  RCLCPP_INFO(node->get_logger(), "当前位置：x=%f,y=%f,z=%f",
    current_pose.position.x, current_pose.position.y, current_pose.position.z);

  // 设置新目标：保持姿态不变，只改变位置
  auto target_pose = current_pose;
  target_pose.position.z +=0.1; //让机械臂向上移动0.1米

  // 告诉MoveIt 去这个坐标 
  move_group.setPoseTarget(target_pose);
  
  // 4. 计算直线路径 (Cartesian Path)
  // 不使用 OMPL 采样规划，直接在数学上切分直线
  
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose); // 加入刚才设置的目标点

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0; // 跳跃阈值，0表示禁用
  const double eef_step = 0.01;      // 步长:每1cm计算一个点

  // computeCartesianPath 返回一个比例 (0.0 到 1.0)
  // 1.0 表示成功规划了 100% 的路径。小于 1.0说明半路断了
  double fraction = move_group.computeCartesianPath(
      waypoints, 
      eef_step, 
      jump_threshold, 
      trajectory);

  RCLCPP_INFO(node->get_logger(), "直线路径规划覆盖率: %.2f%%", fraction * 100.0);

  if (fraction >= 0.90) // 如果至少规划出了 90% 的路径
  {
    // 5. 执行 (Execute)
    // computeCartesianPath 直接生成了轨迹，不需要再调用 plan()
    move_group.execute(trajectory);
    RCLCPP_INFO(node->get_logger(), "执行直线运动成功！");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "直线规划失败！只规划了 %.2f%%，可能目标太远或奇异点。", fraction * 100.0);
  }

  rclcpp::shutdown();
  return 0;
}