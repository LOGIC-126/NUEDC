from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import  PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Ano飞控数据获取节点
        Node(
            package='ano_data',
            executable='node_ano_data',
            name='ano_data_node',
            output='screen',
        ),
        
        # 雷达数据输出
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lslidar_driver'),
                    'launch',
                    'lsn10p_launch.py'
                ])
            ]),
            launch_arguments={
                'output': 'screen'  # 可选
            }.items()
        ),
        
        # Cartographer节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'use_sim_time': False}],
            arguments = [
                '-configuration_directory', FindPackageShare('ano_cartographer').find('ano_cartographer') + '/config',
                '-configuration_basename', 'ano_imu_2d.lua'],
            output='screen'
        ),
        
        # 地图服务节点
        Node(
            package = 'cartographer_ros',
            executable = 'cartographer_occupancy_grid_node',
            parameters = [
                {'use_sim_time': False},
                {'resolution': 0.05}],
        ),
        
        # 静态TF广播（根据实际硬件调整）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.06', '0', '0', '0', 'base_link', 'laser']
        ),

        # 静态TF广播（根据实际硬件调整）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0.03', '0', '0', '0', 'base_link','imu_link',]
        ),

    ])