from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 加载 MoveIt 配置
    moveit_config = MoveItConfigsBuilder("panda",package_name="panda_configure").to_moveit_configs()

    # 2. 定义我们要运行的节点
    run_moveit_cpp_node = Node(
        package="learn_moveit",           # 功能包名字
        executable="hello_moveit",        # 可执行文件名字
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics, # 关键：加载运动学插件
        ],
    )

    return LaunchDescription([
        run_moveit_cpp_node
    ])