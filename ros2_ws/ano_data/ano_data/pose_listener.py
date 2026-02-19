#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: RZR
@说明: 监听并获取坐标和偏航角，将其发布。
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Pose2D, TransformStamped
from tf2_ros import TransformException
import tf_transformations
import math

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        
        # 参数配置
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('publish_frequency', 100.0)  # Hz
        
        # 获取参数
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        frequency = self.get_parameter('publish_frequency').value
        
        # 创建TF缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 创建位姿发布器 - 使用Pose2D只包含x, y和theta(yaw)
        self.pose_pub = self.create_publisher(Pose2D, '/uav_pose', 10)
        
        # 创建定时器
        timer_period = 1.0 / frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 存储上一次有效的变换
        self.last_valid_transform = None
        
        self.get_logger().info(
            f"位姿监听器已启动，监听 {self.source_frame}->{self.target_frame} 变换，"
            f"发布频率: {frequency}Hz"
        )
        self.get_logger().info("发布消息类型: Pose2D (x, y, theta)")

    def timer_callback(self):
        try:
            # 尝试获取最新的变换
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,  # 目标坐标系
                self.target_frame,  # 源坐标系
                rclpy.time.Time(),  # 获取最新可用变换
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 更新最后一次有效变换
            self.last_valid_transform = transform
            
            # 提取位置
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # 提取四元数并转换为偏航角(yaw)
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
            
            # 创建并发布Pose2D消息 (x, y, theta)
            pose_msg = Pose2D()
            pose_msg.x = x
            pose_msg.y = y
            pose_msg.theta = yaw
            
            self.pose_pub.publish(pose_msg)
            
            # 将弧度转换为角度用于显示
            yaw_deg = math.degrees(yaw)
            
            # 记录调试信息
            self.get_logger().debug(
                f"位置: x={x:.2f}m, y={y:.2f}m, 偏航角={yaw_deg:.1f}°",
                throttle_duration_sec=0.1
            )
            
        except TransformException as ex:
            # 如果之前有有效数据，使用最后一次有效变换
            if self.last_valid_transform:
                self.get_logger().warn(
                    f"TF异常: {str(ex)}. 使用上一次有效变换",
                    throttle_duration_sec=2.0
                )
                
                # 从上次变换提取数据
                transform = self.last_valid_transform
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                
                # 提取四元数并转换为偏航角
                quaternion = (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                )
                _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
                
                # 创建并发布Pose2D消息
                pose_msg = Pose2D()
                pose_msg.x = x
                pose_msg.y = y
                pose_msg.theta = yaw
                
                self.pose_pub.publish(pose_msg)
                
                # 将弧度转换为角度用于显示
                yaw_deg = math.degrees(yaw)
                self.get_logger().info(
                    f"[使用历史]位置: x={x:.2f}m, y={y:.2f}m, 偏航角={yaw_deg:.1f}°",
                    throttle_duration_sec=0.5
                )
            else:
                self.get_logger().error(
                    f"TF异常且无有效历史数据: {str(ex)}",
                    throttle_duration_sec=1.0
                )

def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()