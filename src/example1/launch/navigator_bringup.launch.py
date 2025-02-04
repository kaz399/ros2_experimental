import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # パッケージのパスを取得
    example1_share = get_package_share_directory("example1")
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # Nav2用のパラメータファイル (SLAM 用に調整済み)
    nav2_params_file = os.path.abspath(
        os.path.join(example1_share, "config", "nav2_params.yaml")
    )

    # 1. Gazebo で TurtleBot3 world を起動 (常に use_sim_time:=true)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, "launch", "turtlebot3_world.launch.py")
        ),
        launch_arguments={"use_sim_time": "True"}.items(),
    )

    # 2. Nav2 を起動 (slam:=true, 上記パラメータファイルを使用)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "slam": "True",
            "params_file": nav2_params_file,
            "map": "",
        }.items(),
    )

    # 3. ランダムナビゲーションノード (example1/navigator.py)
    navigator_node = Node(
        package="example1", executable="navigator", name="navigator", output="screen"
    )

    return LaunchDescription([gazebo_launch, nav2_launch, navigator_node])
