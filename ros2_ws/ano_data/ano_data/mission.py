#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: RZR
@说明: 飞行任务控制25电赛特供
"""

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from geometry_msgs.msg import Pose2D,Point            # 接收位置坐标用于检测是否到达
from ano_msg.msg import AnoDatasend, MultiArray                # 设定坐标位置
import time
from enum import Enum
import serial # type: ignore
from .build_road import GridSolver
from .MonocularPlaneMeasurer import MonocularPlaneMeasurer as mp
from vision_msgs.msg import Detection2DArray
import math


class FlightState(Enum):
    INIT = 0
    TAKEOFF = 1
    GOTOTAR = 2
    MISSION = 3
    LAND = 4

class SerialState(Enum):
    WAITING_START = 0
    RECEIVING_DATA = 1
    COMPLETE = 2

"""
任务流程
"""
class MissionNode(Node):
    def __init__(self, name):
        super().__init__(name)                     # ROS2节点父类初始化
        # 创建位姿订阅者
        self.pose_sub = self.create_subscription(
            Pose2D,
            '/uav_pose',
            self.pose_callback,
            10  # QoS队列深度
        )
        self.Ano_sub = self.create_subscription(
            AnoDatasend,
            '/ano_data',
            self.Ano_data_callback,
            10  # QoS队列深度
        )
        # 创建欧拉角订阅者
        self.Ano_euler_sub = self.create_subscription(
            MultiArray,
            '/ano_data/euler_angle',
            self.euler_callback,
            10  # QoS队列深度
        )
        # 创建视觉识别订阅者
        self.pose_sub = self.create_subscription(
            Detection2DArray, 
            '/yolov5/detections',
            self.yolo_callback,
            10  # QoS队列深度
        )
        # 创建目标位置发布者
        self.target_publisher = self.create_publisher(
            Point,
            'target_position_topic',
            10
        )
        self.state = FlightState.INIT
        # 参数设置
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        
        # 获取参数
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # 初始化串口
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout
            )
            self.get_logger().info(f"成功打开串口 {port}, 波特率 {baudrate}")
        except Exception as e:
            self.get_logger().error(f"打开串口失败: {str(e)}")
            raise
        
        """外设灯光串口相关"""
        
        # 参数设置
        self.declare_parameter('send_port', '/dev/ttyS6')
        self.declare_parameter('send_baudrate', 115200)
        
        # 获取发送串口参数
        send_port = self.get_parameter('send_port').get_parameter_value().string_value
        send_baudrate = self.get_parameter('send_baudrate').get_parameter_value().integer_value
        
        # 初始化发送串口
        try:
            self.send_ser = serial.Serial(
                port=send_port,
                baudrate=send_baudrate,
                timeout=0.5
            )
            self.get_logger().info(f"成功打开发送串口 {send_port}, 波特率 {send_baudrate}")
        except Exception as e:
            self.get_logger().error(f"打开发送串口失败: {str(e)}")
            self.send_ser = None

        FALL_POINT = (0, 0, 1.2,"pass")
        
        # 路径生成器
        self.solver = GridSolver()
        self.current_mission_index = -1  # 当前任务点索引
        # 当前位置和姿态
        self.posX = 0
        self.posY = 0
        self.posZ = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # 任务状态标志
        self.target_reached = False
        self.mission_complete = False
        self.current_target = None
        
        # 初始化串口处理相关变量
        self.serial_state = SerialState.WAITING_START
        self.serial_buffer = bytearray()
        self.raw_serial_buffer = bytearray()  # 原始数据缓冲区
        self.barriers = None  # 障碍物数据
        self.mission_points = None
        self.timer = self.create_timer(0.06, self.state_machine)
        self.target_timer = self.create_timer(0.1, self.publish_target)  # 10Hz发布频率
        # 创建定时器读取串口数据
        self.serial_timer = self.create_timer(0.05, self.read_serial)   # 20Hz读取频率


        # 初始化单目测量器
        camera_matrix = [
            [303.14839604, 0.0, 325.69011423],
            [0.0, 302.73501984, 246.41753334],
            [0.0, 0.0, 1.0]
        ]
        dist_coeffs = [0.07238952, -0.12088445, 0.00071579, 0.00371255, 0.04629666]
        
        self.monocular = mp(camera_matrix, dist_coeffs)


    def send_serial_frame(self,data):
        """外设灯光发送特定的串口数据帧 (0e 00 0f)"""
        if self.send_ser and self.send_ser.is_open:
            try:
                # 要发送的数据帧: 0e data 0f
                frame = bytes([0x0e,data, 0x0f])
                self.send_ser.write(frame)
                # self.get_logger().info("成功发送串口数据帧")
                return True
            except Exception as e:
                self.get_logger().error(f"发送串口数据帧失败: {str(e)}")
                return False
        else:
            self.get_logger().warn("发送串口未打开，无法发送数据")
            return False

    def publish_target(self):
        """定时发布目标位置"""
        if self.current_target is not None:
            try:
                self.target_publisher.publish(self.current_target)
                # 可选：降低日志频率以免刷屏
                # self.get_logger().debug(f"发布目标位置: X={self.current_target.x}, Y={self.current_target.y}, Z={self.current_target.z}")
            except Exception as e:
                self.get_logger().error(f"发布目标位置失败: {str(e)}")
    
    # def read_serial(self):
    #     """读取并解析串口数据"""
    #     if self.ser and self.ser.in_waiting > 0:
    #         try:
    #             data = self.ser.read(self.ser.in_waiting)
    #             self.serial_buffer.extend(data)
    #             self.process_serial_buffer()
                
    #         except Exception as e:
    #             self.get_logger().error(f"串口读取错误: {str(e)}")
    #             self.serial_buffer.clear()
    
    def read_serial(self):
        """简化版串口读取"""
        if self.ser and self.ser.in_waiting > 0:
            try:
                # 读取所有可用数据
                data = self.ser.read(self.ser.in_waiting)
                self.raw_serial_buffer.extend(data)
                
                # 检查完整消息
                if b'#' in self.raw_serial_buffer:
                    # 提取完整消息 (从开头到#号)
                    end_index = self.raw_serial_buffer.index(b'#')
                    full_message = self.raw_serial_buffer[:end_index]
                    
                    # 处理消息 (跳过开头的$符号)
                    if full_message.startswith(b'$'):
                        message_str = full_message[1:].decode('utf-8').strip()
                        self.barriers = self.parse_serial_frame(message_str)
                        self.get_logger().info(f"收到障碍物数据: {message_str}")
                    
                    # 清除已处理数据 (包括#号)
                    self.raw_serial_buffer = self.raw_serial_buffer[end_index+1:]
            
            except Exception as e:
                self.get_logger().error(f"串口处理错误: {str(e)}")
                self.raw_serial_buffer.clear()
    
    def parse_serial_frame(self,data):
        """
        解析串口数据帧，将其转换为障碍物元组列表
        
        参数:
            data (str): 串口数据字符串，格式如 "$A1B1,A2B2,A3B3#"
        
        返回:
            list: 包含(A,B)元组的列表，如 [('A1', 'B1'), ('A2', 'B2'), ('A3', 'B3')]
        """
        
        # 按逗号分割数据
        parts = data.split(',')
        
        # 解析每个部分为(A, B)元组
        barriers = []
        for part in parts:
            # 确保部分长度至少为4
            if len(part) >= 4:
                a = part[:2]  # 前两个字符
                b = part[2:4]  # 紧接着的两个字符
                barriers.append(self.solver.ab_to_coordinate(a, b))
        
        return barriers
    
    def yolo_callback(self, msg):
        """接收到YOLO检测结果时的回调函数"""
        try:
            # 获取检测到的目标数量
            detection_count = len(msg.detections)
            
            if detection_count > 0:
                self.get_logger().info(f'YOLO检测到 {detection_count} 个目标')
                
                # 处理每个检测目标
                for i, detection in enumerate(msg.detections):
                    # 获取类别ID和置信度
                    class_id = detection.results[0].hypothesis.class_id
                    confidence = detection.results[0].hypothesis.score
                    
                    # 获取边界框信息
                    bbox_center_x = detection.bbox.center.position.x
                    bbox_center_y = detection.bbox.center.position.y
                    bbox_width = detection.bbox.size_x
                    bbox_height = detection.bbox.size_y
                    
                    # 计算边界框的四个角点
                    x1 = bbox_center_x - bbox_width / 2
                    y1 = bbox_center_y - bbox_height / 2
                    x2 = bbox_center_x + bbox_width / 2
                    y2 = bbox_center_y + bbox_height / 2
                    
                    X,Y = self.monocular.pixel_to_world_with_decoupling(
                        bbox_center_x, 
                        bbox_center_y, 
                        self.posZ - 0.075,  # 减去相机到无人机中心的高度偏移
                        self.roll, 
                        self.pitch, 
                    )

                    W_X , W_Y = self.monocular.get_world_position(
                        self.posX,
                        self.posY,
                        self.yaw,
                        X,
                        Y
                    )

                    self.get_logger().info(
                        f'目标 {i+1}: 类别={class_id}, 置信度={confidence:.2f}, 位置=({W_X:.2f}m, {W_Y:.2f}m)')
            else:
                self.get_logger().info('YOLO未检测到目标')
                
        except Exception as e:
            self.get_logger().error(f'处理YOLO检测结果失败: {str(e)}')
    
    def Ano_data_callback(self, msg):
        """接收到飞控消息时的回调函数"""
        try:
            self.posZ = msg.z
        except Exception as e:
            self.get_logger().error(f"处理姿态消息失败: {str(e)}")
    
    def euler_callback(self,msg):
        """
        接收到欧拉角消息时的回调函数
        """
        try:
            self.roll = math.degrees(msg.roll)
            self.pitch = math.degrees(msg.pitch)
            # self.yaw = msg.yaw
            # self.get_logger().info(f"Get: {self.roll:.2f}, {self.pitch:.2f}")
        except Exception as e:
            self.get_logger().error(f"处理欧拉角消息失败: {str(e)}")

    def pose_callback(self, msg):
        """接收到姿态消息时的回调函数"""
        try:
            self.posX = msg.x
            self.posY = msg.y
            self.yaw = msg.theta
            # theta = msg.theta
            # self.get_logger().info(f"位置: X={self.posX}cm, Y={self.posY}cm, Z={self.posZ}cm")
        except Exception as e:
            self.get_logger().error(f"处理姿态消息失败: {str(e)}")

    def set_target_position(self, x, y, z):
        """设置新的目标位置"""
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.current_target = msg
        self.get_logger().info(f"设置新目标位置: X={x}, Y={y}, Z={z}")
        return True
    
    def check_distance(self, target_x, target_y, target_z, threshold=0.1):
        """检查当前位置与目标点的距离"""
        dx = self.posX - target_x
        dy = self.posY - target_y
        dz = self.posZ - target_z
        distance = (dx**2 + dy**2 + dz**2)**0.5
        return distance < threshold
    
    def execute_mission(self, mission_type):
        """执行特定任务"""
        if mission_type == "wait":
            self.Wait()
            return True
        # 可扩展其他任务类型
        if mission_type == "pass":
            self.Pass()
            return True
        if mission_type == "scan":
            self.Scan()
            return True
        return True
    """
    任务列表
    """
    def Pass(self):
        """逃过"""
        self.get_logger().info("跳过...")

    def Scan(self):
        """查找"""
        self.get_logger().info("执行查找任务...")
    
    def Wait(self):
        self.get_logger().info("执行等待任务...")
        time.sleep(5)  # 等待5秒
    """=============================="""
    def state_machine(self):
        """状态机主逻辑"""
        if self.state == FlightState.INIT:
            self.get_logger().info("初始状态，准备起飞", throttle_duration_sec=1.0)
            if self.barriers is not None:
                # 设置障碍物并求解路径
                self.solver.set_barriers(self.barriers)
                # 求解最短路径
                result = self.solver.solve()
                # 任务参数
                self.mission_points = [
                    (x, y, 1.2, "scan") for x, y in result['world_path']
                    # (0.0, -0.5, 1.2,"pass"),  # 任务点1 (x, y, z, mission)       
                ]
                self.solver.draw_path(result['path'])
                # # 打印所有任务点（航点 + 任务）
                # self.get_logger().info("====== 任务航点列表 ======")
                # for i, (x, y, z, mission) in enumerate(self.mission_points):
                #     self.get_logger().info(f"航点 {i:2d}: (x={x:.2f}, y={y:.2f}, z={z:.2f}), 任务: {mission}")
                # self.get_logger().info("=======================")
                if self.check_distance(0, 0, 0, 0.15):
                    self.send_serial_frame(0x02)
                self.state = FlightState.TAKEOFF
            else:
                self.get_logger().info("等待障碍物数据...", throttle_duration_sec=1.0)  # 节流输出
                self.send_serial_frame(0x00)
                    

        elif self.state == FlightState.TAKEOFF:
            # 检查是否到达起飞高度
            if self.set_target_position(0, 0, 1.2):
                self.get_logger().info("起飞指令已发送")
            if self.check_distance(0, 0, 1.2, 0.15):
                self.get_logger().info("起飞完成，准备前往第一个任务点")
                self.current_mission_index = 0
                self.state = FlightState.GOTOTAR
        
        elif self.state == FlightState.GOTOTAR:
            if self.current_mission_index < len(self.mission_points):
                point = self.mission_points[self.current_mission_index]
                target_x, target_y, target_z, _ = point
                
                # 发送目标位置
                if not self.target_reached:
                    self.set_target_position(target_x, target_y, target_z)
                    self.get_logger().info(f"前往任务点{self.current_mission_index+1}: [{target_x}, {target_y}, {target_z}]")
                    self.target_reached = True  # 防止重复发送
                
                # 检查是否到达目标点
                if self.check_distance(target_x, target_y, target_z):
                    self.get_logger().info(f"到达任务点{self.current_mission_index+1}")
                    self.state = FlightState.MISSION
                    self.target_reached = False  # 重置标志
        
        elif self.state == FlightState.MISSION:
            if self.current_mission_index < len(self.mission_points):
                _, _, _, mission_type = self.mission_points[self.current_mission_index]
                
                # 执行任务
                if self.execute_mission(mission_type):
                    self.get_logger().info(f"任务点{self.current_mission_index+1}任务完成")
                    
                    # 检查是否所有任务完成
                    if self.current_mission_index >= len(self.mission_points) - 1:
                        self.get_logger().info("所有任务完成，准备降落")
                        self.state = FlightState.LAND
                    else:
                        # 前往下一个任务点
                        self.current_mission_index += 1
                        self.state = FlightState.GOTOTAR
        
        elif self.state == FlightState.LAND:
            # 发送降落指令
            if not self.mission_complete:
                self.set_target_position(0, 0, 0)  # 降落到原点
                self.get_logger().info("降落指令已发送")
                self.send_serial_frame(0x03)
                self.mission_complete = True
            
            # 检查是否着陆
            if self.check_distance(0, 0, 0, 0.1):
                self.get_logger().info("成功着陆，任务结束")
                self.timer.cancel()  # 停止状态机
                # 停止定时器
                self.target_timer.cancel()
                # 清除目标位置
                self.current_target = None
    



def main(args=None):                               # ROS2节点主入口main函数
    rclpy.init(args=args)                          # ROS2 Python接口初始化
    node = MissionNode("node_mission") # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                               # 循环等待ROS2退出
    node.destroy_node()                            # 销毁节点对象
    rclpy.shutdown()                               # 关闭ROS2 Python接口