# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.conditions import IfCondition, UnlessCondition
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node, SetRemap
# from launch_ros.substitutions import FindPackageShare
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# # 在launch文件中添加转换节点
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode
# import os

# def generate_launch_description():

#     ## ***** Launch arguments *****
#     # 是否使用仿真时间，真实的机器人我们不需要，设置为False
#     use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')

#     ## ***** File paths ******
#     # 找到cartographer功能包的地址
#     pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
#     ## ***** Nodes *****
#     #=====================话题转化=================================
#     scan_converter = ComposableNodeContainer(
#     name='scan_converter',
#     namespace='',
#     package='rclcpp_components',
#     executable='component_container',
#     composable_node_descriptions=[
#         ComposableNode(
#             package='pointcloud_to_laserscan',
#             plugin='pointcloud_to_laserscan::LaserScanToMultiEchoLaserScanNode',
#             name='scan_converter',
#             remappings=[('scan', 'scan'),
#                        ('multi_echo_scan', 'horizontal_laser_2d')]
#         )
#     ],
#     output='screen'
#     )
#     #=====================声明三个节点，cartographer/occupancy_grid_node/rviz_node=================================
#     cartographer_node = Node(
#         package = 'cartographer_ros',
#         executable = 'cartographer_node',
#         parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
#         arguments = [
#             '-configuration_directory', FindPackageShare('ano_cartographer').find('ano_cartographer') + '/config',
#             '-configuration_basename', 'cartographer.lua'],
#         # arguments = [
#         #     '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
#         #     '-configuration_basename', 'backpack_2d.lua'],
#         remappings = [
#             # ('echoes', 'horizontal_laser_2d'),
#             ('scan', 'horizontal_laser_2d')
#             ],
#         output = 'screen'
#         )
    
	
# 	# 可视化节点
#     rviz_node = Node(
#           package='rviz2',
#           namespace='rviz2',
#           executable='rviz2',
#           name='rviz2',
#           output='screen')

#     cartographer_occupancy_grid_node = Node(
#         package = 'cartographer_ros',
#         executable = 'cartographer_occupancy_grid_node',
#         parameters = [
#             {'use_sim_time': True},
#             {'resolution': 0.05}],
#         )

#     return LaunchDescription([
#         use_sim_time_arg,
#         # Nodes
#         scan_converter,
#         rviz_node ,
#         cartographer_node,
#         cartographer_occupancy_grid_node,
#     ])


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Cartographer节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'use_sim_time': False}],
            # arguments=[
            #     '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            #     '-configuration_basename', 'backpack2d.lua'  # 修改后的配置文件
            # ],
            arguments = [
                '-configuration_directory', FindPackageShare('ano_cartographer').find('ano_cartographer') + '/config',
                '-configuration_basename', 'ano_2d.lua'],
            output='screen'
        ),
        
        # 地图服务节点
        # Node(
        #     package='cartographer_ros',
        #     executable='occupancy_grid_node',
        #     name='occupancy_grid_node',
        #     parameters=[{'use_sim_time': False}],
        #     arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        # ),
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
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),
        
        # Node(
        #   package='rviz2',
        #   namespace='rviz2',
        #   executable='rviz2',
        #   name='rviz2',
        #   output='screen')

    ])