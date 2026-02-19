import rclpy
from rclpy.node import Node
import time
import socket
import json
import threading

# 导入ROS2消息类型
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D


class DetectionReaderNode(Node):
    """
    ROS2节点，用于通过UDP接收YOLOv5检测结果并发布到ROS2话题
    """

    def __init__(self):
        super().__init__('detection_reader')
        
        # 类别定义
        CLASSES = ['0','1','2','3','4','5','6','7','8','9']
        # CLASSES = ["kongqu", "monkey", "tiger", "wolf", "elephant"]
        
        # UDP配置参数
        self.declare_parameter('udp_host', '127.0.0.1')
        self.declare_parameter('udp_port', 8888)
        
        self.udp_host = self.get_parameter('udp_host').value
        self.udp_port = self.get_parameter('udp_port').value
        
        # 类别列表 - 需要与发送端保持一致
        self.declare_parameter('classes', CLASSES)
        self.CLASSES = self.get_parameter('classes').value
        
        # ROS2发布器配置
        self.declare_parameter('detection_topic', '/yolov5/detections')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('publish_rate', 10.0)
        
        detection_topic = self.get_parameter('detection_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # 创建ROS2发布器
        self.detection_pub = self.create_publisher(
            Detection2DArray, 
            detection_topic, 
            10
        )
        
        # 创建UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(1.0)  # 设置超时时间为1秒
        
        # UDP接收线程控制
        self.udp_running = True
        self.udp_thread = None
        
        # 启动UDP接收线程
        self.start_udp_receiver()
        
        # 创建定时器，用于处理接收到的数据
        self.timer = self.create_timer(0.001, self.timer_callback)
        
        # 存储接收到的检测数据
        self.latest_detection_data = None
        self.data_lock = threading.Lock()
        
        # 用于跟踪上次的检测结果，避免重复处理相同结果
        self.last_detection_count = 0
        self.last_update_time = time.time()
        
        # 发布统计
        self.publish_count = 0
        self.last_publish_log_time = time.time()
        
        self.get_logger().info(f'检测结果读取节点已启动，监听UDP {self.udp_host}:{self.udp_port}')
        self.get_logger().info(f'类别列表: {self.CLASSES}')
        self.get_logger().info(f'发布话题: {detection_topic}')
        
    def start_udp_receiver(self):
        """启动UDP接收线程"""
        try:
            self.udp_socket.bind((self.udp_host, self.udp_port))
            self.udp_thread = threading.Thread(target=self.udp_receive_loop)
            self.udp_thread.daemon = True
            self.udp_thread.start()
            self.get_logger().info(f'UDP接收线程已启动')
        except Exception as e:
            self.get_logger().error(f'绑定UDP端口失败: {e}')
    
    def udp_receive_loop(self):
        """UDP接收循环"""
        while self.udp_running:
            try:
                # 接收数据
                data, addr = self.udp_socket.recvfrom(1024)  # 缓冲区大小1024字节
                
                try:
                    # 解析JSON数据
                    detection_data = json.loads(data.decode('utf-8'))
                    
                    # 更新最新数据
                    with self.data_lock:
                        self.latest_detection_data = {
                            'data': detection_data,
                            'address': addr,
                            'receive_time': time.time()
                        }
                    
                    # 在UDP线程中记录接收信息
                    self.get_logger().debug(f'收到来自 {addr} 的检测数据')
                    
                except json.JSONDecodeError:
                    self.get_logger().warn(f'数据解析失败，接收数据: {data[:100]}...')  # 只打印前100字符
                except KeyError as e:
                    self.get_logger().warn(f'数据格式错误，缺少字段: {e}')
                except Exception as e:
                    self.get_logger().warn(f'处理UDP数据时出错: {e}')
                    
            except socket.timeout:
                # 超时是正常的，继续循环
                continue
            except Exception as e:
                if self.udp_running:
                    self.get_logger().error(f'UDP接收错误: {e}')
                break
    
    def timer_callback(self):
        """
        定时器回调函数，处理接收到的检测结果
        """
        with self.data_lock:
            if self.latest_detection_data is None:
                return
            
            detection_data = self.latest_detection_data['data']
            address = self.latest_detection_data['address']
            receive_time = self.latest_detection_data['receive_time']
            
            # 清空数据，避免重复处理
            self.latest_detection_data = None
        
        try:
            # 处理检测结果
            self.process_detection_data(detection_data, address, receive_time)
            
        except Exception as e:
            self.get_logger().error(f'处理检测数据时出错: {e}')
    
    def process_detection_data(self, detection_data, address, receive_time):
        """
        处理接收到的检测数据
        """
        # 验证数据格式
        required_fields = ['timestamp', 'count', 'detections']
        for field in required_fields:
            if field not in detection_data:
                self.get_logger().warn(f'检测数据缺少必要字段: {field}')
                return
        
        timestamp = detection_data['timestamp']
        count = detection_data['count']
        detections = detection_data['detections']
        
        # 转换数据格式以匹配原有处理逻辑
        processed_detections = []
        for det in detections:
            try:
                # 直接使用发送端的类别ID，不需要转换
                processed_det = {
                    'class_id': det.get('class_id', -1),
                    'confidence': det.get('confidence', 0.0),
                    'bbox': det.get('bbox', [0, 0, 0, 0])
                }
                processed_detections.append(processed_det)
            except Exception as e:
                self.get_logger().warn(f'转换检测数据时出错: {e}')
                continue
        
        # 调用原有的处理逻辑
        self.process_detections(processed_detections, timestamp)
        
        # 记录接收统计信息
        current_time = time.time()
        if current_time - self.last_update_time > 5.0:  # 每5秒记录一次
            self.get_logger().info(f'持续接收检测数据，最新检测目标数: {count}')
            self.last_update_time = current_time
    
    def process_detections(self, detections, timestamp):
        """
        处理检测结果并发布到ROS2话题
        """
        if not detections:
            # 没有检测到目标
            if self.last_detection_count > 0:
                self.get_logger().info('未检测到目标')
                self.last_detection_count = 0
                
                # 即使没有检测到目标，也发布空消息以保持连续性
                self.publish_empty_detection()
            return
        
        # 检查是否有新的检测结果
        if len(detections) != self.last_detection_count:
            self.last_detection_count = len(detections)
            
            # # 打印检测结果到终端
            # self.print_detections(detections, timestamp)
            
        # 发布到ROS2话题
        self.publish_detections(detections, timestamp)
    
    def publish_detections(self, detections, timestamp):
        """
        将检测结果发布为ROS2消息
        """
        # 创建Detection2DArray消息
        detection_array_msg = Detection2DArray()
        
        # 设置消息头
        detection_array_msg.header = Header()
        detection_array_msg.header.stamp = self.get_clock().now().to_msg()
        detection_array_msg.header.frame_id = self.frame_id
        
        # 转换每个检测结果
        for det in detections:
            detection_msg = Detection2D()
            
            # 设置检测结果的ID和假设
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['class_id'])
            hypothesis.hypothesis.score = float(det['confidence'])
            detection_msg.results.append(hypothesis)
            
            # 设置边界框
            bbox = det['bbox']
            # 将 [x1, y1, x2, y2] 转换为中心点和尺寸
            center_x = (bbox[0] + bbox[2]) / 2.0
            center_y = (bbox[1] + bbox[3]) / 2.0
            width = bbox[2] - bbox[0]
            height = bbox[3] - bbox[1]
            
            detection_msg.bbox.center.position.x = float(center_x)
            detection_msg.bbox.center.position.y = float(center_y)
            detection_msg.bbox.size_x = float(width)
            detection_msg.bbox.size_y = float(height)
            
            detection_array_msg.detections.append(detection_msg)
        
        # 发布消息
        self.detection_pub.publish(detection_array_msg)
        self.publish_count += 1
        
        # 定期记录发布统计
        current_time = time.time()
        if current_time - self.last_publish_log_time > 10.0:
            self.get_logger().info(f'已发布 {self.publish_count} 条检测消息')
            self.publish_count = 0
            self.last_publish_log_time = current_time
    
    def publish_empty_detection(self):
        """
        发布空的检测消息
        """
        detection_array_msg = Detection2DArray()
        detection_array_msg.header = Header()
        detection_array_msg.header.stamp = self.get_clock().now().to_msg()
        detection_array_msg.header.frame_id = self.frame_id
        
        self.detection_pub.publish(detection_array_msg)
    
    def print_detections(self, detections, timestamp):
        """
        在终端打印检测结果
        """
        print(f"\n[{time.strftime('%H:%M:%S')}] 检测到 {len(detections)} 个目标:")
        
        for i, det in enumerate(detections):
            class_id = det['class_id']
            confidence = det['confidence']
            bbox = det['bbox']
            
            # 根据类别ID获取类别名称
            class_name = self.get_class_name(class_id)
            
            print(f"  目标 {i+1}: {class_name}({class_id}), "
                  f"置信度: {confidence:.3f}, "
                  f"位置: [{bbox[0]:.1f}, {bbox[1]:.1f}, {bbox[2]:.1f}, {bbox[3]:.1f}]")
    
    def get_class_name(self, class_id):
        """
        根据类别ID返回类别名称
        """
        if 0 <= class_id < len(self.CLASSES):
            return self.CLASSES[class_id]
        else:
            return f"unknown_{class_id}"
    
    def destroy_node(self):
        """
        节点销毁时的清理工作
        """
        self.get_logger().info('正在关闭检测结果读取节点...')
        
        # 停止UDP接收线程
        self.udp_running = False
        if self.udp_socket:
            self.udp_socket.close()
        
        if self.udp_thread and self.udp_thread.is_alive():
            self.udp_thread.join(timeout=2.0)
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DetectionReaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n节点被用户中断")
    except Exception as e:
        print(f"节点运行错误: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()