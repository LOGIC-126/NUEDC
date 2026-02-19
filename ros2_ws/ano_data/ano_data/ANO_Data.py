#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: RZR
@说明: 获取串口的信息并解读发布，发送指令帧
"""

import rclpy
import math,time
from rclpy.node import Node
import serial              # type: ignore
import serial.tools.list_ports  # type: ignore
from ano_msg.msg import AnoDatasend, MultiArray # type: ignore # 自定义的目标位置消息
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Point, Quaternion, Twist, Vector3
from ano_msg.srv import SetTarget
from tf_transformations import quaternion_from_euler
import numpy as np # type: ignore
# import subprocess


class ANO_Serial(Node):
    def __init__(self,name):
        super().__init__(name)
        
        # 创建发布者
        self.inertial_pub = self.create_publisher(  # 飞控数据发布者
            AnoDatasend,   # 消息类型
            '/ano_data',  # 话题名称
            10                # QoS队列深度
        )

        self.MultiArray_pub = self.create_publisher(  # 飞控数据发布者
            MultiArray,   # 消息类型
            '/ano_data/euler_angle',  # 话题名称
            10                # QoS队列深度
        )

        self.imu_publisher = self.create_publisher( # 创建IMU发布器
            Imu,    # 消息类型
            '/imu', # 话题名称
            10      # QoS队列深度
            )
        
        self.odom_publisher = self.create_publisher( # 创建Odom发布器
            Odometry,    # 消息类型
            '/odom', # 话题名称
            10      # QoS队列深度
            )
        
        # 创建位姿订阅者
        self.pose_sub = self.create_subscription(
            Pose2D,
            '/uav_pose',
            self.pose_callback,
            10  # QoS队列深度
        )
        # # 创建目标服务
        # self.srv = self.create_service(
        #     SetTarget,
        #     'set_target_position',
        #     self.target_callback
        # )
        # 创建目标位置订阅者
        self.sub = self.create_subscription(
            Point,  # 使用 geometry_msgs/msg/Point 消息类型
            'target_position_topic',  # 订阅的主题名称
            self.target_callback,
            10  # 消息队列大小
        )

        # 参数设置
        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 500000)
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

        # 初始化缓冲区
        self.raw_buffer = bytearray()  # 原始数据缓冲区
        self.frame_buffer = []         # 完整帧缓冲区

        self.ano_data = AnoDatasend()
        self.imu_msg = Imu()            # 创建IMU消息
        self.odom_msg = Odometry()
        self.angles = MultiArray()
        
        self.position = np.zeros(3, dtype=np.float64)  # 单位:米
        self.velocity = np.zeros(3, dtype=np.float64)  # 单位:米/秒 (机体坐标系)
        
        # 启用定时器读取数据
        self.timer = self.create_timer(0.001, self.read_serial)  # 1000Hz高频读取
        # # 定时器检测距离
        # self.check_timer = self.create_timer(0.1, self.check_position)

    def target_callback(self, msg):
        """处理目标位置消息"""
        try:
            # 提取目标位置（假设消息单位为米）
            x = int(msg.x * 100)  # 转换为厘米
            y = int(msg.y * 100)
            z = int(msg.z * 100)
            
            # 高度保护：限制高度不超过2米
            MAX_HEIGHT = 200  # 最大允许高度（cm）
            if z > MAX_HEIGHT:
                self.get_logger().warning(
                    f"请求高度 {z}cm 超过最大允许高度 {MAX_HEIGHT}cm，已自动限制"
                )
                z = MAX_HEIGHT

            # 调用目标构建函数
            data_bytes = build_tar_frame(x, y, z)
            for i in range(3):
                self.ser.write(data_bytes)
                time.sleep(0.01)
            self.get_logger().info(f"设置目标位置: X={x}cm, Y={y}cm, Z={z}cm", throttle_duration_sec=2.0)
            
        except Exception as e:
            self.get_logger().error(f"处理目标位置失败: {str(e)}")

    def pose_callback(self, msg):
        """接收到姿态消息时的回调函数"""
        try:
            x = int(msg.x * 100)
            y = int(msg.y * 100)
            theta = msg.theta
            hx_x = int(math.cos(theta) * 10000)
            hx_y = int(math.sin(theta) * 10000)
            data_bytes = build_pos_frame(x,y,hx_x,hx_y)
            
            self.ser.write(data_bytes)
            
            # self.get_logger().info(f"发送姿态数据: x={x}, y={y}")
        
        except Exception as e:
            self.get_logger().error(f"处理姿态消息失败: {str(e)}")
    
    def read_serial(self):
        """持续读取串口数据到缓冲区"""
        try:
            while self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)  # 读取全部可用数据
                self.raw_buffer.extend(data)
                self.get_logger().debug(f"收到 {len(data)} 字节,缓冲区长度: {len(self.raw_buffer)}")
        except Exception as e:
            self.get_logger().error(f"读取串口失败: {str(e)}")
    
    def send_raw_data(self, data_array):
        """直接将字节数组数据发送到串口（无协议封装）
        
        参数:
            data_array: 要发送的字节数组 (每个元素为整数，范围0-255)
        """
        try:
            # 将整数列表转换为字节数组并发送
            self.ser.write(bytes(data_array))
            self.get_logger().debug(f"发送原始数据: {data_array}")
        except Exception as e:
            self.get_logger().error(f"串口发送失败: {str(e)}")

    def process_buffer(self):
        """处理缓冲区中的完整帧"""
        while len(self.raw_buffer) >= 4:  # 至少需要帧头+ID+LEN
            # 查找帧头 0xAA
            head_pos = self.raw_buffer.find(0xAA)
            if head_pos == -1:
                self.raw_buffer.clear()
                return
            
            # 丢弃帧头前的无效数据
            if head_pos > 0:
                self.raw_buffer = self.raw_buffer[head_pos:]
                head_pos = 0

            # 检查长度是否足够
            if len(self.raw_buffer) < 4:
                return
            
            # 获取关键字段
            d_addr = self.raw_buffer[1]
            frame_id = self.raw_buffer[2]
            data_len = self.raw_buffer[3]
            total_len = data_len + 6  # 总帧长 = 数据长度 + 6字节头尾
            
            # 检查是否收到完整帧
            if len(self.raw_buffer) < total_len:
                return
            
            # 提取完整帧
            frame = self.raw_buffer[:total_len]
            del self.raw_buffer[:total_len]  # 从缓冲区移除已处理数据
            
            # 校验数据
            if self.validate_checksum(frame):
                self.frame_buffer.append(frame)
                self.parse_frame(frame)
            else:
                self.get_logger().warning("校验失败,丢弃帧")

    def validate_checksum(self, frame):
        """校验和验证"""
        data_len = frame[3]
        sum_check = 0
        add_check = 0
        
        # 计算校验和(HEAD到DATA结束)
        for i in range(data_len + 4):
            sum_check = (sum_check + frame[i]) & 0xFF
            add_check = (add_check + sum_check) & 0xFF
        
        # 对比校验值
        return (sum_check == frame[-2]) and (add_check == frame[-1])

    def parse_frame(self, frame):
        """解析有效数据帧"""
        try:
            d_addr = frame[1]
            frame_id = frame[2]
            data_len = frame[3]
            data = frame[4:-2]  # 排除校验位
            
            # 根据ID分发处理
            if frame_id == 0x01:
                self.parse_0x01(data)
            elif frame_id == 0x02:
                self.parse_0x02(data)
            elif frame_id == 0x03:
                self.parse_0x03(data)
            elif frame_id == 0x04:
                self.parse_0x04(data)
            elif frame_id == 0x05:
                self.parse_0x05(data)
            elif frame_id == 0x06:  # 飞控运行模式
                self.parse_0x06(data)
            elif frame_id == 0x07:  # 飞行速度数据
                self.parse_0x07(data)
            elif frame_id == 0x08:  # 位置偏移数据
                self.parse_0x08(data)
            elif frame_id == 0x09:  # 风速估计
                self.parse_0x09(data)
            elif frame_id == 0x0A:  # 目标姿态数据
                self.parse_0x0A(data)
            elif frame_id == 0x0B:  # 目标速度数据
                self.parse_0x0B(data)
            elif frame_id == 0x0C:  # 回航信息
                self.parse_0x0C(data)
            elif frame_id == 0x0D:  # 电压电流数据
                self.parse_0x0D(data)
            elif frame_id == 0x0E:  # 外接模块状态
                self.parse_0x0E(data)
            # ...添加其他ID处理...
            self.Pub_Odom()
            # self.get_logger().info(f"解析成功 ID:0x{frame_id:02X}")
        except Exception as e:
            self.get_logger().error(f"解析帧错误: {str(e)}")

    def parse_0x01(self, data):
        """
        解析惯性传感器数据 (ID:0x01)
        
        # 协议定义(DATA区域):
        # | int16 | int16 | int16 | int16 | int16 | int16 | uint8 |
        # | ACC_X | ACC_Y | ACC_Z | GYR_X | GYR_Y | GYR_Z | SHOCK_STA |
        # 总长度:13字节
        """
        try:
        # 检查数据长度
            if len(data) != 13:
                self.get_logger().error(f"ID 0x01 数据长度错误,预期13字节,实际收到 {len(data)} 字节")
                return

            # 解析加速度(int16 x3,单位:mG)
            acc_x = int.from_bytes(data[0:2], 'little', signed=True)  # X轴加速度
            acc_y = int.from_bytes(data[2:4], 'little', signed=True)  # Y轴加速度
            acc_z = int.from_bytes(data[4:6], 'little', signed=True)  # Z轴加速度

            # 解析陀螺仪(int16 x3,单位:0.01°/s)
            gyr_x = int.from_bytes(data[6:8], 'little', signed=True)   # X轴角速度
            gyr_y = int.from_bytes(data[8:10], 'little', signed=True)  # Y轴角速度
            gyr_z = int.from_bytes(data[10:12], 'little', signed=True) # Z轴角速度

            # 解析震动状态(uint8,0-正常,1-震动警告)
            shock_sta = data[12]
            
            # 设置header
            self.imu_msg.header.stamp = self.get_clock().now().to_msg()
            self.imu_msg.header.frame_id = "imu_link"  # 根据你的实际frame_id设置
            
            self.imu_msg.linear_acceleration.x = float(acc_x * 0.01)
            self.imu_msg.linear_acceleration.y = float(acc_y * 0.01)
            self.imu_msg.linear_acceleration.z = float(acc_z * 0.01)
            
            self.imu_msg.angular_velocity.x = math.radians(gyr_x)
            self.imu_msg.angular_velocity.y = math.radians(gyr_y)
            self.imu_msg.angular_velocity.z = math.radians(gyr_z)
            
            self.imu_publisher.publish(self.imu_msg)
            # pub数值
            # self.data_list.acc_x = acc_x
            # self.inertial_pub.publish(self.data_list)
            # self.get_logger().info(f"已发布惯性传感器数据:{gyr_z}")

        except Exception as e:
            self.get_logger().error(f"解析ID 0x01数据失败: {str(e)}")
            # self.data_list.n0x01 = []  # 清空无效数据



    def parse_0x02(self, data):
        """
        解析罗盘、气压、温度传感器数据 (ID:0x02)

        数据定义(参考协议文档):
        | int16 | int16 | int16 | int32 | int16 | uint8 | uint8 |
        | MAG_X | MAG_Y | MAG_Z | ALT_BAR | TMP | BAR_STA | MAG_STA |
        总长度:14字节
        """
        try:
        # 检查数据长度
            if len(data) != 14:
                self.get_logger().error(f"ID 0x02 数据长度错误,预期14字节,实际收到 {len(data)} 字节")
                return
            # 解析磁罗盘数据(int16 x3)
            mag_x = int.from_bytes(data[0:2], 'little', signed=True)
            mag_y = int.from_bytes(data[2:4], 'little', signed=True)
            mag_z = int.from_bytes(data[4:6], 'little', signed=True)

            # 解析气压高度(int32)
            alt_bar = int.from_bytes(data[6:10], 'little', signed=True)  # 单位cm

            # 解析温度(int16,放大10倍)
            tmp_raw = int.from_bytes(data[10:12], 'little', signed=True)
            temperature = tmp_raw / 10.0  # 实际温度值,单位摄氏度

            # 解析状态标志(uint8 x2)
            bar_sta = data[12]
            mag_sta = data[13]

        # 存储数据
        # self.data_list.n0x02 = [mag_x, mag_y, mag_z, alt_bar, temperature, bar_sta, mag_sta]
        except Exception as e:
            self.get_logger().error(f"解析ID 0x02数据失败: {str(e)}")
            # self.data_list.n0x01 = []  # 清空无效数据

    def parse_0x03(self, data):
        """
        解析飞控姿态(欧拉角格式) (ID:0x03)

        # 协议定义(DATA区域):
        # | int16     | int16     | int16     | uint8    |
        # | ROL*100   | PIT*100   | YAW*100   | FUSION_STA |
        # 总长度:7字节
        """
        try:
            # 检查数据长度
            if len(data) != 7:
                self.get_logger().error(f"ID 0x03 数据长度错误,预期7字节,实际收到 {len(data)} 字节")
                return

            # 解析欧拉角(int16 x3,传输时放大100倍)
            rol_raw = int.from_bytes(data[0:2], 'little', signed=True)  # 横滚角(单位:0.01°)
            pit_raw = int.from_bytes(data[2:4], 'little', signed=True)  # 俯仰角(单位:0.01°)
            yaw_raw = int.from_bytes(data[4:6], 'little', signed=True)  # 航向角(单位:0.01°)

            # 单位转换(除以100还原实际值)
            rol_deg = rol_raw / 100.0  # 单位:°
            pit_deg = pit_raw / 100.0
            yaw_deg = yaw_raw / 100.0

            # 解析融合状态(uint8,0-未融合,1-融合正常)
            fusion_sta = data[6]
            # 欧拉角转四元数（XYZ顺序）
            roll_rad = math.radians(rol_deg)  # 角度 → 弧度
            pitch_rad = math.radians(pit_deg)
            yaw_rad = math.radians(yaw_deg)

            self.angles.roll = roll_rad
            self.angles.pitch = pitch_rad
            self.angles.yaw = yaw_rad
            # 这里imu数据不需要四元数
            # q = quaternion_from_euler(-yaw_rad, -pitch_rad, roll_rad, axes='rzyx')
            # self.imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            # 发布到协议列表
            self.MultiArray_pub.publish(self.angles)


        except Exception as e:
            self.get_logger().error(f"解析ID 0x03数据失败: {str(e)}")

    def parse_0x04(self, data):
        """解析飞控姿态(四元数格式) (ID:0x04)
        
        # 协议定义(DATA区域):
        # | int16       | int16       | int16       | int16       | uint8    |
        # | V0*10000    | V1*10000    | V2*10000    | V3*10000    | FUSION_STA |
        # 总长度:9字节
        """
        try:
            # 检查数据长度
            if len(data) != 9:
                self.get_logger().error(f"ID 0x04 数据长度错误,预期9字节,实际收到 {len(data)} 字节")
                return

            # 解析四元数分量(int16 x4,传输时放大10000倍)
            v0_raw = int.from_bytes(data[0:2], 'little', signed=True)   # 四元数V0
            v1_raw = int.from_bytes(data[2:4], 'little', signed=True)   # 四元数V1
            v2_raw = int.from_bytes(data[4:6], 'little', signed=True)   # 四元数V2
            v3_raw = int.from_bytes(data[6:8], 'little', signed=True)   # 四元数V3

            # 单位转换(除以10000还原实际值)
            v0 = v0_raw / 10000.0
            v1 = v1_raw / 10000.0
            v2 = v2_raw / 10000.0
            v3 = v3_raw / 10000.0

            # 解析融合状态(uint8,0-未融合,1-融合正常)
            fusion_sta = data[8]

        except Exception as e:
            self.get_logger().error(f"解析ID 0x04数据失败: {str(e)}")
            # self.data_list.n0x04 = []  # 清空无效数据

    def parse_0x05(self, data):
        """解析高度数据 (ID:0x05)
    
        # 协议定义(DATA区域):
        # | int32           | int32           | uint8      |
        # | ALT_FU (cm)     | ALT_ADD (cm)    | ALT_STA    |
        # 总长度:9字节
        """
        STEALT = 70
        try:
            # 检查数据长度
            if len(data) != 9:
                self.get_logger().error(f"ID 0x05 数据长度错误,预期9字节,实际收到 {len(data)} 字节")
                return

            # 解析融合高度(int32,单位厘米)
            alt_fu = int.from_bytes(data[0:4], 'little', signed=True)  # 融合后对地高度
        
            # 解析附加高度(int32,单位厘米,例如超声波/激光测距)
            alt_add = int.from_bytes(data[4:8], 'little', signed=True)  # 附加高度传感器数据
            # self.get_logger().info(f"高度数据 {alt_add} ")

            self.ano_data.z = float(alt_add/100)
            self.inertial_pub.publish(self.ano_data)
            # 解析测距状态(uint8,协议未明确状态码含义,需参考文档)
            alt_sta = data[8]

            self.position[2] = alt_fu

        except Exception as e:
            self.get_logger().error(f"解析ID 0x05数据失败: {str(e)}")
            # self.data_list.n0x05 = []  # 清空无效数据

    def parse_0x06(self, data):
        """解析飞控运行模式 (ID:0x06)
        
        # 协议定义(DATA区域):
        # | U8     | U8      | U8     | U8     | U8     |
        # | MODE   | SFLAG   | CID    | CMD0   | CMD1   |
        # 总长度:5字节

        字段说明:
        - MODE:   飞控模式 (0-姿态,1-自稳+定高,2-定点,3-程控)
        - SFLAG:  状态标志 (0-锁定,1-解锁,2-已起飞)
        - CID:    当前指令类别ID (协议文档功能触发类帧定义)
        - CMD0/1: 指令参数 (具体含义由CID决定)
        """

        try:
            # 检查数据长度
            if len(data) != 5:
                self.get_logger().error(f"ID 0x06 数据长度错误,预期5字节,实际收到 {len(data)} 字节")
                return

            # 解析原始字段
            mode = data[0]    # 直接获取uint8值
            sflag = data[1]
            cid = data[2]
            cmd0 = data[3]
            cmd1 = data[4]

            # --- 可选:字段值映射为可读文本 ---
            mode_map = {
                0: "姿态模式",
                1: "自稳+定高",
                2: "定点飞行",
                3: "程控模式"
            }
            status_map = {
                0: "锁定",
                1: "解锁",
                2: "已起飞"
            }

            # # --- 存储到数据列表 ---
            # self.data_list.n0x06 = [
            #     mode,          # 原始模式值
            #     sflag,         # 原始状态值
            #     cid,           # 指令类别ID
            #     cmd0, cmd1,    # 指令参数
            #     # 可选:添加解释性字段
            #     mode_map.get(mode, "未知模式"),
            #     status_map.get(sflag, "未知状态")
            # ]

            # # 调试输出(包含原始值和解释文本)
            # self.get_logger().info(
            #     f"飞控状态: 模式={mode}({self.data_list.n0x06[4]}), "
            #     f"状态={sflag}({self.data_list.n0x06[5]}), "
            #     f"指令CID=0x{cid:02X}, 参数CMD0=0x{cmd0:02X} CMD1=0x{cmd1:02X}",
            #     throttle_duration_sec=1  # 限速输出
            # )

        except IndexError as e:
            self.get_logger().error(f"数据索引错误: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"解析ID 0x06数据失败: {str(e)}")
            # self.data_list.n0x06 = []  # 清空无效数据
    

    def parse_0x07(self, data):
        """解析飞行速度数据 (ID:0x07)
        
        # 协议定义(DATA区域):
        # | int16     | int16     | int16     |
        # | SPEED_X   | SPEED_Y   | SPEED_Z   |
        # 总长度:6字节
        
        字段说明:
        - SPEED_X/Y/Z: XYZ轴速度,单位cm/s,有符号整数
        - 坐标系定义:
        - X:机头前进方向
        - Y:机身左侧方向  
        - Z:垂直向上方向
        """
        try:
            # 检查数据长度
            if len(data) != 6:
                self.get_logger().error(f"ID 0x07 数据长度错误,预期6字节,实际收到 {len(data)} 字节")
                return

            # 解析速度分量(小端模式,有符号int16)
            speed_x = int.from_bytes(data[0:2], 'little', signed=True)  # X轴速度
            speed_y = int.from_bytes(data[2:4], 'little', signed=True)  # Y轴速度
            speed_z = int.from_bytes(data[4:6], 'little', signed=True)  # Z轴速度

            # 调试输出(带单位转换示例)
            # self.get_logger().info(
            #     f"飞行速度: X={speed_x/100.0:.2f}m/s, "
            #     f"Y={speed_y/100.0:.2f}m/s, "
            #     f"Z={speed_z/100.0:.2f}m/s",
            #     throttle_duration_sec=1
            # )

            self.velocity[0] = speed_x/100
            self.velocity[1] = speed_y/100
            self.velocity[2] = speed_z/100

        except Exception as e:
            self.get_logger().error(f"解析ID 0x07数据失败: {str(e)}")
            # self.data_list.n0x07 = []  # 清空无效数据
    
    def parse_0x08(self, data):
        """解析位置偏移数据 (ID:0x08)
        
        # 协议定义(DATA区域):
        # | int32           | int32           |
        # | POS_X           | POS_Y           |
        # 总长度:8字节
        
        字段说明:
        - POS_X/Y: 相对于起飞点的水平位置偏移量,单位厘米(cm)
        - 坐标系定义:
          - X+:机头前进方向
          - Y+:机身左侧方向
        - 特殊值:0x80000000 表示数据无效
        """
        try:
            # 检查数据长度
            if len(data) != 8:
                self.get_logger().error(f"ID 0x08 数据长度错误,预期8字节,实际收到 {len(data)} 字节")
                return

            # 解析位置偏移量(小端模式,有符号int32)
            pos_x = int.from_bytes(data[0:4], 'little', signed=True)  # X轴偏移
            pos_y = int.from_bytes(data[4:8], 'little', signed=True)  # Y轴偏移

            # 检查无效值(0x80000000)
            INVALID_VALUE = -2147483648  # 0x80000000的十进制表示
            if pos_x == INVALID_VALUE or pos_y == INVALID_VALUE:
                self.get_logger().warning("位置偏移数据无效")
                return

            self.position[0] = pos_x/100
            self.position[1] = pos_y/100

            # 调试输出(带单位转换)
            # self.get_logger().info(
            #     f"位置偏移: X={pos_x/100.0:.2f}m, "
            #     f"Y={pos_y/100.0:.2f}m (相对起飞点)",
            #     throttle_duration_sec=0.5
            # )

        except Exception as e:
            self.get_logger().error(f"解析ID 0x08数据失败: {str(e)}")
            # self.data_list.n0x08 = []  # 清空无效数据
    
    def parse_0x0A(self, data):
        """解析目标姿态数据 (ID:0x0A)
        
        # 协议定义(DATA区域):
        # | int16     | int16     | int16     |
        # | TAR_ROL   | TAR_PIT   | TAR_YAW   |
        # 总长度:6字节
        
        字段说明:
        - TAR_ROL/PIT/YAW: 目标姿态角,单位0.01°,有符号整数
        - 范围:横滚/俯仰±90°,航向0-360°
        - 特殊值:0x8000 表示数据无效
        """
        try:
            if len(data) != 6:
                self.get_logger().error(f"ID 0x0A 数据长度错误,预期6字节,实际收到 {len(data)} 字节")
                return

            # 解析目标角度(小端模式,有符号int16)
            tar_rol = int.from_bytes(data[0:2], 'little', signed=True) / 100.0  # 横滚(°)
            tar_pit = int.from_bytes(data[2:4], 'little', signed=True) / 100.0  # 俯仰(°)
            tar_yaw = int.from_bytes(data[4:6], 'little', signed=True) / 100.0  # 航向(°)

            # 检查无效值
            if any(abs(v) > 327.67 for v in [tar_rol, tar_pit, tar_yaw]):
                self.get_logger().warning("目标姿态数据无效")
                return

            # # 存储数据(单位:度)
            # self.data_list.n0x0A = [tar_rol, tar_pit, tar_yaw]

            # # 调试输出
            # self.get_logger().info(
            #     f"目标姿态: ROL={tar_rol:.2f}°, PIT={tar_pit:.2f}°, YAW={tar_yaw:.2f}°",
            #     throttle_duration_sec=1
            # )

        except Exception as e:
            self.get_logger().error(f"解析ID 0x0A数据失败: {str(e)}")
            # self.data_list.n0x0A = []

    def parse_0x0B(self, data):
        """解析目标速度数据 (ID:0x0B)
        
        # 协议定义(DATA区域):
        # | int16     | int16     | int16     |
        # | TAR_SPD_X | TAR_SPD_Y | TAR_SPD_Z |
        # 总长度:6字节
        
        字段说明:
        - TAR_SPD_X/Y/Z: 目标速度,单位cm/s,有符号整数
        - 坐标系:
        - X:机头前进方向
        - Y:机身左侧方向
        - Z:垂直向上方向
        - 特殊值:0x8000 表示数据无效
        """
        try:
            if len(data) != 6:
                self.get_logger().error(f"ID 0x0B 数据长度错误,预期6字节,实际收到 {len(data)} 字节")
                return

            # 解析目标速度(小端模式,有符号int16)
            spd_x = int.from_bytes(data[0:2], 'little', signed=True)  # X轴(cm/s)
            spd_y = int.from_bytes(data[2:4], 'little', signed=True)  # Y轴(cm/s)
            spd_z = int.from_bytes(data[4:6], 'little', signed=True)  # Z轴(cm/s)

            # 检查无效值
            if any(v == -32768 for v in [spd_x, spd_y, spd_z]):
                self.get_logger().warning("目标速度数据无效")
                return

            # # 存储原始数据(单位:cm/s)
            # self.data_list.n0x0B = [spd_x, spd_y, spd_z]

            # # 调试输出(转换为m/s)
            # self.get_logger().info(
            #     f"目标速度: X={spd_x/100.0:.2f}m/s, Y={spd_y/100.0:.2f}m/s, Z={spd_z/100.0:.2f}m/s",
            #     throttle_duration_sec=1
            # )

        except Exception as e:
            self.get_logger().error(f"解析ID 0x0B数据失败: {str(e)}")
            # self.data_list.n0x0B = []

    def parse_0x0C(self, data):
        """解析回航信息 (ID:0x0C)
        
        # 协议定义(DATA区域):
        # | int16     | uint16    |
        # | R_A*10    | R_D       |
        # 总长度:4字节
        
        字段说明:
        - R_A: 回航角度(正负180°,放大10倍传输)
        - R_D: 回航距离(单位:米)
        - 特殊值:R_D=0xFFFF 表示数据无效
        """
        try:
            if len(data) != 4:
                self.get_logger().error(f"ID 0x0C 数据长度错误,预期4字节,实际收到 {len(data)} 字节")
                return

            # 解析回航信息
            r_a = int.from_bytes(data[0:2], 'little', signed=True) / 10.0  # 角度(°)
            r_d = int.from_bytes(data[2:4], 'little', signed=False)         # 距离(m)

            # 检查无效值
            if r_d == 0xFFFF or abs(r_a) > 180.0:
                self.get_logger().warning("回航信息无效")
                return

            # # 存储数据
            # self.data_list.n0x0C = [r_a, r_d]

            # # 调试输出
            # self.get_logger().info(
            #     f"回航信息: 角度={r_a:.1f}°, 距离={r_d}m",
            #     throttle_duration_sec=1
            # )

        except Exception as e:
            self.get_logger().error(f"解析ID 0x0C数据失败: {str(e)}")
            # self.data_list.n0x0C = []
        
    def parse_0x0D(self, data):
        """解析电压电流数据 (ID:0x0D)
        
        # 协议定义(DATA区域):
        # | uint16    | uint16    |
        # | VOTAGE*100 | CURRENT*100 |
        # 总长度:4字节
        
        字段说明:
        - VOTAGE: 电源电压,放大100倍传输(单位:V)
        - CURRENT: 电源电流,放大100倍传输(单位:A)
        - 示例:收到值 2500 → 实际值 25.00V
        """
        try:
            if len(data) != 4:
                self.get_logger().error(f"ID 0x0D 数据长度错误,预期4字节,实际收到 {len(data)} 字节")
                return

            # 解析电压电流(小端模式,无符号uint16)
            voltage = int.from_bytes(data[0:2], 'little', signed=False) / 100.0  # 单位:V
            current = int.from_bytes(data[2:4], 'little', signed=False) / 100.0  # 单位:A

            # # 存储数据
            # self.data_list.n0x0D = [voltage, current]

            # 计算功率(可选)
            power = voltage * current  # 单位:W

            # 调试输出
            # self.get_logger().info(
            #     f"电源状态: 电压={voltage:.2f}V, 电流={current:.2f}A, 功率={power:.2f}W",
            #     throttle_duration_sec=1
            # )

        except Exception as e:
            self.get_logger().error(f"解析ID 0x0D数据失败: {str(e)}")
            # self.data_list.n0x0D = []
    
    def parse_0x0E(self, data):
        """解析外接模块状态 (ID:0x0E)
        
        # 协议定义(DATA区域):
        # | uint8     | uint8     | uint8     | uint8     |
        # | STA_G_VEL | STA_G_POS | STA_GPS   | STA_ALT_ADD |
        # 总长度:4字节
        
        状态码定义:
        0=无数据, 1=有数据但不可用, 2=正常, 3=良好(GPS专用)
        """
        try:
            if len(data) != 4:
                self.get_logger().error(f"ID 0x0E 数据长度错误,预期4字节,实际收到 {len(data)} 字节")
                return

            # 解析各模块状态
            sta_g_vel = data[0]   # 通用速度传感器状态
            sta_g_pos = data[1]   # 通用位置传感器状态
            sta_gps = data[2]     # GPS状态
            sta_alt = data[3]     # 附加高度传感器状态

            # 状态文本映射
            STATUS_MAP = {0: "无数据", 1: "不可用", 2: "正常", 3: "良好"}

            # # 存储原始数据
            # self.data_list.n0x0E = [sta_g_vel, sta_g_pos, sta_gps, sta_alt]

            # # 调试输出
            # self.get_logger().info(
            #     f"外设状态: 速度传感器={STATUS_MAP.get(sta_g_vel, '未知')}, "
            #     f"位置传感器={STATUS_MAP.get(sta_g_pos, '未知')}, "
            #     f"GPS={STATUS_MAP.get(sta_gps, '未知')}, "
            #     f"高度传感器={STATUS_MAP.get(sta_alt, '未知')}",
            #     throttle_duration_sec=1
            # )

        except Exception as e:
            self.get_logger().error(f"解析ID 0x0E数据失败: {str(e)}")
            # self.data_list.n0x0E = []

    def parse_0x0F(self, data):
        """解析RGB亮度信息 (ID:0x0F)
        
        # 协议定义(DATA区域):
        # | uint8     | uint8     | uint8     | uint8     |
        # | BRL_R     | BRL_G     | BRL_B     | BRL_A     |
        # 总长度:4字节
        
        字段说明:
        - BRL_R/G/B: RGB三色亮度(0-20级)
        - BRL_A: 独立LED亮度(0-20级)
        - 0=最暗,20=最亮
        """
        try:
            if len(data) != 4:
                self.get_logger().error(f"ID 0x0F 数据长度错误,预期4字节,实际收到 {len(data)} 字节")
                return

            # 解析亮度值
            r = data[0]  # 红色亮度
            g = data[1]  # 绿色亮度
            b = data[2]  # 蓝色亮度
            a = data[3]  # 独立LED亮度

            # 亮度范围检查
            if any(not 0 <= v <= 20 for v in [r, g, b, a]):
                self.get_logger().warning("亮度值超出有效范围")

            # # 存储数据
            # self.data_list.n0x0F = [r, g, b, a]

            # # 调试输出(十六进制格式)
            # self.get_logger().info(
            #     f"RGB亮度: R={r}/20, G={g}/20, B={b}/20, A={a}/20 | "
            #     f"HEX=#{r:02X}{g:02X}{b:02X}",
            #     throttle_duration_sec=1
            # )

        except Exception as e:
            self.get_logger().error(f"解析ID 0x0F数据失败: {str(e)}")
            # self.data_list.n0x0F = []

    def parse_0xA0(self, data):
        """解析LOG字符串信息 (ID:0xA0)
        
        # 协议定义(DATA区域):
        # | uint8     | uint8[n-1] |
        # | COLOR     | STR        |
        # 总长度:n字节(可变长度)
        
        字段说明:
        - COLOR: 颜色标识(0=黑,1=红,2=绿)
        - STR: ASCII字符串(长度=LEN-1)
        """
        try:
            if len(data) < 1:
                self.get_logger().error("ID 0xA0 数据长度不足")
                return

            # 解析颜色和字符串
            color_code = data[0]
            log_str = data[1:].decode('ascii', errors='replace')  # 处理非ASCII字符

            # 颜色映射
            color_map = {
                0: "\033[30m",  # 黑色
                1: "\033[31m",  # 红色
                2: "\033[32m"   # 绿色
            }
            color = color_map.get(color_code, "")

            # # 存储数据(原始字节和字符串)
            # self.data_list.n0xA0 = {
            #     'color': color_code,
            #     'text': log_str,
            #     'raw': data
            # }

            # 终端彩色输出(可选)
            self.get_logger().info(
                f"{color}LOG_STR: {log_str}\033[0m",
                throttle_duration_sec=0.5
            )

        except Exception as e:
            self.get_logger().error(f"解析ID 0xA0数据失败: {str(e)}")

    def parse_0xA1(self, data):
        """解析LOG字符串+数字信息 (ID:0xA1)
        
        # 协议定义(DATA区域):
        # | int32     | uint8[n-4] |
        # | VAL       | STR        |
        # 总长度:n字节(可变长度)
        
        字段说明:
        - VAL: 32位有符号整数
        - STR: ASCII字符串(长度=LEN-4)
        """
        try:
            if len(data) < 4:
                self.get_logger().error("ID 0xA1 数据长度不足")
                return

            # 解析数字和字符串
            val = int.from_bytes(data[0:4], 'little', signed=True)
            log_str = data[4:].decode('ascii', errors='replace')

            # # 存储数据
            # self.data_list.n0xA1 = {
            #     'value': val,
            #     'text': log_str,
            #     'raw': data
            # }

            # 调试输出
            self.get_logger().info(
                f"LOG_VAL: {log_str} {val}",
                throttle_duration_sec=0.5
            )

        except Exception as e:
            self.get_logger().error(f"解析ID 0xA1数据失败: {str(e)}")

    def Pub_Odom(self):
        """
        发布Odom数据
        """
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        # 设置位置信息
        self.odom_msg.pose.pose.position = Point(
            x=self.position[0],
            y=self.position[1],
            z=self.position[2]
        )
        # 中性方向 (Cartographer主要使用位置和速度)
        self.odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # 位置协方差 (Cartographer对XY精度要求较高)
        # 索引顺序: [x, y, z, rot_x, rot_y, rot_z]
        self.odom_msg.pose.covariance = [
            0.5, 0.0,  0.0,  0.0, 0.0, 0.0,  # X (低精度)
            0.0,  0.5, 0.0,  0.0, 0.0, 0.0,  # Y (低精度)
            0.0,  0.0,  0.01, 0.0, 0.0, 0.0,  # Z (高精度)
            0.0,  0.0,  0.0,  1e6, 0.0, 0.0,  # 旋转X (未知)
            0.0,  0.0,  0.0,  0.0, 1e6, 0.0,  # 旋转Y (未知)
            0.0,  0.0,  0.0,  0.0, 0.0, 1e6   # 旋转Z (未知)
        ]
        
        # 设置速度信息 (机体坐标系)
        self.odom_msg.twist.twist.linear = Vector3(
            x=self.velocity[0],
            y=self.velocity[1],
            z=self.velocity[2]
        )
        
        # 角速度设为0 (Cartographer主要使用线速度)
        self.odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        
        # 速度协方差 (Cartographer对速度精度要求较高)
        # 索引顺序: [vx, vy, vz, wx, wy, wz]
        self.odom_msg.twist.covariance = [
            0.1, 0.0,  0.0,  0.0, 0.0, 0.0,  # VX (低精度)
            0.0,  0.1, 0.0,  0.0, 0.0, 0.0,  # VY (低精度)
            0.0,  0.0,  0.02,  0.0, 0.0, 0.0,  # VZ (高等精度)
            0.0,  0.0,  0.0,  1e6, 0.0, 0.0,  # 角速度X (未知)
            0.0,  0.0,  0.0,  0.0, 1e6, 0.0,  # 角速度Y (未知)
            0.0,  0.0,  0.0,  0.0, 0.0, 1e6   # 角速度Z (未知)
        ]
        
        self.odom_publisher.publish(self.odom_msg)


def build_pos_frame(x :int, y :int, hx_x:int, hx_y:int):
    """
    构建当前位置(POS)数据帧
    :param x: X坐标值 (整数)
    :param y: Y坐标值 (整数)
    :param hx_x: HX_X坐标值 (整数)
    :param hx_y: HX_Y坐标值 (整数)
    :return: 字节数组格式的数据帧
    """
    # 帧头 | 长度(8字节) | 类型(POS)
    frame = bytearray([0xAA, 0x08, 0x01])

    # 添加坐标数据 (小端字节序)
    # x坐标
    frame.append(x & 0xFF)        # x低字节
    frame.append((x >> 8) & 0xFF) # x高字节
    
    # y坐标
    frame.append(y & 0xFF)        # y低字节
    frame.append((y >> 8) & 0xFF) # y高字节
    
    # hx_x坐标
    frame.append(hx_x & 0xFF)     # hx_x低字节
    frame.append((hx_x >> 8) & 0xFF) # hx_x高字节
    
    # hx_y坐标
    frame.append(hx_y & 0xFF)     # hx_y低字节
    frame.append((hx_y >> 8) & 0xFF) # hx_y高字节

    # 计算校验和 (长度 + 类型 + 所有数据)
    checksum = frame[1] + frame[2]  # 长度 + 类型
    for byte in frame[3:]:
        checksum += byte
    frame.append(checksum & 0xFF)  # 取低8位

    return frame

def build_tar_frame(x, y, z):
    """
    构建目标位置(TAR)数据帧
    :param x: X坐标值 (整数)
    :param y: Y坐标值 (整数)
    :param z: Z坐标值 (整数)
    :return: 字节数组格式的数据帧
    """
    # 帧头 | 长度 | 类型
    frame = bytearray([0xAA, 0x06, 0x02])
    
    # 添加坐标数据 (小端字节序)
    frame.append(x & 0xFF)        # x低字节
    frame.append((x >> 8) & 0xFF)  # x高字节
    frame.append(y & 0xFF)        # y低字节
    frame.append((y >> 8) & 0xFF)  # y高字节
    frame.append(z & 0xFF)        # z低字节
    frame.append((z >> 8) & 0xFF)  # z高字节
    
    # 计算校验和 (长度 + 类型 + 所有数据)
    checksum = frame[1] + frame[2]  # 长度 + 类型
    for byte in frame[3:]:
        checksum += byte
    frame.append(checksum & 0xFF)  # 取低8位
    
    return frame

def frame_to_hex(frame):
    """将字节数组转换为十六进制字符串表示"""
    return ' '.join([f'0x{byte:02X}' for byte in frame])
def main(args=None):
    rclpy.init(args=args)
    node = ANO_Serial('ano_data')
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.process_buffer()  # 持续处理缓冲区
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

