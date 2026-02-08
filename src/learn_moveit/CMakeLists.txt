cmake_minimum_required(VERSION 3.8)
project(learn_moveit)

# 1. 编译选项 (保留)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. 寻找依赖 (保留，CLI自动生成的)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# ================= 这里是我们要添加的核心部分 =================

# 3. 设置 C++ 标准 (MoveIt 建议使用 C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 4. 生成可执行文件
# 告诉 CMake 我们要编译 src/hello_moveit.cpp，生成的可执行文件叫 hello_moveit
add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(moveit_demo1 src/moveit_demo1.cpp)

# 5. 链接依赖库
# 告诉编译器这个程序用到了 rclcpp 和 moveit
ament_target_dependencies(hello_moveit
  rclcpp
  moveit_ros_planning_interface
)
ament_target_dependencies(moveit_demo1
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# 6. 安装规则
# 这一步非常重要！没有它，ros2 run 找不到你的程序
install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS moveit_demo1
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ==========================================================

ament_package()