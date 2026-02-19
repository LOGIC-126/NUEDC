#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: RZR
@说明: 云台相关代码
"""

import rclpy                      # ROS2 Python接口库
from rclpy.node   import Node     # ROS2 节点类
from ano_msg.msg import AnoDatasend # type: ignore # 自定义的目标位置消息

class GimbalNode(Node):

    def __init__(self, name):
        super().__init__(name)                             # ROS2节点父类初始化
        self.sub = self.create_subscription(\
            AnoDatasend, 'Ano_data', self.listener_callback, 10) # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        self.Gimbal_Run_Tick = 0           # 减速计数值

    def listener_callback(self, msg):                      # 创建回调函数，执行收到话题消息后对数据的处理
        self.Gimbal_Run_Tick += 1
        self.Servo_Pro(self.Gimbal_Run_Tick,msg,5)

    def Servo_Angle_PIT(self,Angle):
        """设置舵机角度"""
        
                        # 设置角度

    def Servo_Angle_ROL(self,Angle):
        """设置舵机角度"""
        
                        # 设置角度

    def Servo_Pro(self,Tick,msg,Set_Tick):
        """舵机驱动程序"""
        if(Tick <= Set_Tick):
            return
        Tick = 0
        self.get_logger().info(f'debug-> pit: {round(msg.pit,2)},rol: {round(msg.rol,2)}')
        self.Servo_Angle_PIT(msg.pit)
        self.Servo_Angle_ROL(msg.rol)



def main(args=None):                               # ROS2节点主入口main函数
    rclpy.init(args=args)                          # ROS2 Python接口初始化
    node = GimbalNode("GimbalNode_Ctrl")           # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                               # 循环等待ROS2退出
    node.destroy_node()                            # 销毁节点对象
    rclpy.shutdown()                               # 关闭ROS2 Python接口





