#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ano_msg.msg import MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import cv2 # type: ignore
import numpy as np # type: ignore
import math
import tf_transformations
import json
import os
from datetime import datetime

class SimpleIMUCameraCalibration(Node):
    def __init__(self):
        super().__init__('imu_camera_calibration')
        
        # 参数
        self.camera_id = 0  # 摄像头ID
        self.imu_topic = '/multiarray'  # IMU话题
        
        # 相机内参
        self.camera_matrix = np.array([
            [303.14839604, 0.0, 325.69011423],
            [0.0, 302.73501984, 246.41753334],
            [0.0, 0.0, 1.0]
        ])
        
        self.dist_coeffs = np.array([0.07238952, -0.12088445, 0.00071579, 0.00371255, 0.04629666])
        
        # 标定板参数
        self.pattern_size = (9, 6)  # 棋盘格内角点数量
        self.square_size = 0.025    # 方格大小(米)
        
        # 标定数据存储
        self.camera_poses = []      # 相机位姿
        self.imu_orientations = []  # IMU方向（旋转矩阵）
        
        # 标定结果
        self.R_imu2cam = None
        self.t_imu2cam = None
        self.is_calibrated = False
        
        # IMU数据（欧拉角，单位：弧度）
        self.latest_roll = 0.0
        self.latest_pitch = 0.0
        self.latest_yaw = 0.0
        self.has_imu_data = False
        
        # TF广播
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # IMU订阅
        self.imu_sub = self.create_subscription(
            MultiArray, 
            self.imu_topic, 
            self.imu_callback, 
            10
        )
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error('无法打开摄像头')
            # 尝试其他摄像头ID
            for i in range(1, 5):
                self.cap = cv2.VideoCapture(i)
                if self.cap.isOpened():
                    self.get_logger().info(f'使用摄像头 ID: {i}')
                    break
            if not self.cap.isOpened():
                self.get_logger().error('无法找到可用的摄像头')
                return
                
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 创建OpenCV窗口
        cv2.namedWindow('IMU-Camera Calibration', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('IMU-Camera Calibration', 800, 600)
        
        # 定时器 - 处理相机帧
        self.timer = self.create_timer(0.033, self.camera_processing)  # 30Hz
        
        self.get_logger().info('IMU-相机标定节点已启动')
        self.get_logger().info('按空格键捕获当前位姿对')
        self.get_logger().info('按 c 键执行标定')
        self.get_logger().info('按 s 键保存标定结果')
        self.get_logger().info('按 r 键重置数据')
        self.get_logger().info('按 ESC 退出')
    
    def imu_callback(self, msg):
        """IMU欧拉角数据回调"""  
        # 转换为弧度
        self.latest_roll = msg.roll
        self.latest_pitch = msg.pitch
        self.latest_yaw = msg.yaw
        self.has_imu_data = True
        
        # 调试信息
        self.get_logger().debug(f'收到IMU数据 - Roll: {math.degrees(self.latest_roll):.2f}°, Pitch: {math.degrees(self.latest_pitch):.2f}°, Yaw: {math.degrees(self.latest_yaw):.2f}°')
    
    def camera_processing(self):
        """处理相机帧"""
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            self.get_logger().error('摄像头未初始化')
            return
            
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('无法读取相机帧')
            # 尝试重新初始化摄像头
            self.cap.release()
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                self.get_logger().error('重新初始化摄像头失败')
            return
        
        # 处理图像
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, self.pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        # 显示状态信息
        status_text = f"Samples: {len(self.camera_poses)}"
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if self.has_imu_data:
            cv2.putText(frame, "IMU: Connected", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            # 显示当前欧拉角（转换为角度显示）
            euler_text = f"Roll: {math.degrees(self.latest_roll):.1f}, Pitch: {math.degrees(self.latest_pitch):.1f}, Yaw: {math.degrees(self.latest_yaw):.1f}"
            cv2.putText(frame, euler_text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            cv2.putText(frame, "IMU: Disconnected", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        if found:
            # 亚像素精确化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(frame, self.pattern_size, corners, found)
            cv2.putText(frame, "Checkerboard: Detected", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Checkerboard: Not Found", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        if self.is_calibrated:
            cv2.putText(frame, "Status: Calibrated", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Status: Not Calibrated", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 显示操作说明
        help_text = "SPACE:Capture  C:Calibrate  S:Save  R:Reset  ESC:Exit"
        cv2.putText(frame, help_text, (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 显示图像
        cv2.imshow('IMU-Camera Calibration', frame)
        
        # 处理键盘输入
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC退出
            self.get_logger().info('正在关闭节点...')
            self.cleanup()
            rclpy.shutdown()
        elif key == ord(' ') and found and self.has_imu_data:  # 空格键捕获
            self.capture_pose_pair(frame, corners)
        elif key == ord('c'):  # 执行标定
            self.calibrate()
        elif key == ord('s'):  # 保存标定结果
            self.save_calibration_result()
        elif key == ord('r'):  # 重置数据
            self.reset_calibration()
    
    def capture_pose_pair(self, image, corners):
        """捕获位姿对"""
        if not self.has_imu_data:
            self.get_logger().warn('没有IMU数据，无法捕获')
            return
        
        # 生成物体坐标系点
        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2) * self.square_size
        
        # 求解PNP问题，获取相机位姿
        success, rvec, tvec = cv2.solvePnP(objp, corners, self.camera_matrix, self.dist_coeffs)
        
        if success:
            # 将IMU欧拉角转换为旋转矩阵
            R_imu = self.euler_to_rotation_matrix(self.latest_roll, self.latest_pitch, self.latest_yaw)
            
            # 存储位姿对
            self.camera_poses.append((rvec.copy(), tvec.copy()))
            self.imu_orientations.append(R_imu)
            
            self.get_logger().info(f'成功捕获位姿对 {len(self.camera_poses)}')
            self.get_logger().info(f'当前欧拉角 - Roll: {math.degrees(self.latest_roll):.2f}°, '
                                 f'Pitch: {math.degrees(self.latest_pitch):.2f}°, '
                                 f'Yaw: {math.degrees(self.latest_yaw):.2f}°')
        else:
            self.get_logger().warn('求解PNP失败')
    
    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        """欧拉角转旋转矩阵"""
        # 绕X轴旋转 (roll)
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        
        # 绕Y轴旋转 (pitch)
        Ry = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        
        # 绕Z轴旋转 (yaw)
        Rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # 组合旋转矩阵 (ZYX顺序)
        return Rz @ Ry @ Rx
    
    def calibrate(self):
        """执行标定"""
        if len(self.camera_poses) < 3:
            self.get_logger().warn(f'需要至少3组位姿对，当前只有 {len(self.camera_poses)}')
            return
        
        # 手眼标定 AX = XB
        R_list = []
        t_list = []
        
        for i in range(len(self.camera_poses) - 1):
            # 获取相邻两帧的相机位姿变化
            rvec1, tvec1 = self.camera_poses[i]
            rvec2, tvec2 = self.camera_poses[i + 1]
            
            R1, _ = cv2.Rodrigues(rvec1)
            R2, _ = cv2.Rodrigues(rvec2)
            
            # 相机相对旋转和平移
            R_cam_rel = R2 @ R1.T
            t_cam_rel = tvec2 - R_cam_rel @ tvec1
            
            # IMU相对旋转
            R_imu1 = self.imu_orientations[i]
            R_imu2 = self.imu_orientations[i + 1]
            R_imu_rel = R_imu2 @ R_imu1.T
            
            R_list.append((R_cam_rel, R_imu_rel))
            t_list.append((t_cam_rel, R_imu_rel))
        
        # 使用SVD方法求解平均旋转
        M = np.zeros((3, 3))
        for R_cam, R_imu in R_list:
            M += R_cam @ R_imu.T
        
        U, S, Vt = np.linalg.svd(M)
        self.R_imu2cam = U @ Vt
        
        # 确保旋转矩阵的行列式为1
        if np.linalg.det(self.R_imu2cam) < 0:
            Vt[2, :] *= -1
            self.R_imu2cam = U @ Vt
        
        # 求解平移向量
        A = []
        b = []
        for (t_cam, R_imu), (R_cam, _) in zip(t_list, R_list):
            A.append(R_cam - np.eye(3))
            b.append(t_cam.reshape(-1) - self.R_imu2cam @ R_imu @ np.zeros(3))
        
        if len(A) > 0:
            A = np.vstack(A)
            b = np.hstack(b)
            self.t_imu2cam = np.linalg.lstsq(A, b, rcond=None)[0]
        
        self.is_calibrated = True
        self.get_logger().info('IMU到相机标定完成!')
        self.get_logger().info(f'旋转矩阵:\n{self.R_imu2cam}')
        self.get_logger().info(f'平移向量: {self.t_imu2cam}')
        
        # 发布TF变换
        self.publish_tf()
        
        # 自动保存标定结果
        self.save_calibration_result()
    
    def publish_tf(self):
        """发布TF变换"""
        if not self.is_calibrated:
            return
        
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'imu_link'
        transform.child_frame_id = 'camera_link'
        
        # 设置平移
        transform.transform.translation.x = float(self.t_imu2cam[0])
        transform.transform.translation.y = float(self.t_imu2cam[1])
        transform.transform.translation.z = float(self.t_imu2cam[2])
        
        # 将旋转矩阵转换为四元数（TF需要四元数格式）
        quaternion = tf_transformations.quaternion_from_matrix(
            np.vstack([np.hstack([self.R_imu2cam, [[0], [0], [0]]]), [0, 0, 0, 1]])
        )
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info('已发布IMU到相机的TF变换')
    
    def save_calibration_result(self):
        """保存标定结果到文件"""
        if not self.is_calibrated:
            self.get_logger().warn('尚未完成标定，无法保存结果')
            return
        
        # 创建结果字典
        result = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "rotation_matrix": self.R_imu2cam.tolist(),
            "translation_vector": self.t_imu2cam.tolist(),
            "sample_count": len(self.camera_poses),
            "camera_matrix": self.camera_matrix.tolist(),
            "distortion_coefficients": self.dist_coeffs.tolist()
        }
        
        # 生成文件名
        filename = f"imu_camera_calibration_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        try:
            # 保存为JSON文件
            with open(filename, 'w') as f:
                json.dump(result, f, indent=4)
            
            self.get_logger().info(f'标定结果已保存到: {filename}')
            
            # 同时打印到控制台
            self.get_logger().info('=== 标定结果 ===')
            self.get_logger().info(f'旋转矩阵 R_imu2cam:')
            self.get_logger().info(f'{self.R_imu2cam}')
            self.get_logger().info(f'平移向量 t_imu2cam: {self.t_imu2cam}')
            
            # 计算欧拉角形式
            euler_angles = tf_transformations.euler_from_matrix(
                np.vstack([np.hstack([self.R_imu2cam, [[0], [0], [0]]]), [0, 0, 0, 1]])
            )
            self.get_logger().info(f'欧拉角 (弧度): roll={euler_angles[0]:.4f}, pitch={euler_angles[1]:.4f}, yaw={euler_angles[2]:.4f}')
            self.get_logger().info(f'欧拉角 (角度): roll={math.degrees(euler_angles[0]):.2f}°, pitch={math.degrees(euler_angles[1]):.2f}°, yaw={math.degrees(euler_angles[2]):.2f}°')
            
        except Exception as e:
            self.get_logger().error(f'保存标定结果失败: {str(e)}')
    
    def get_calibration_data(self):
        """获取标定数据（供外部调用）"""
        if not self.is_calibrated:
            return None
        
        return {
            "rotation_matrix": self.R_imu2cam.copy(),
            "translation_vector": self.t_imu2cam.copy(),
            "is_calibrated": self.is_calibrated,
            "sample_count": len(self.camera_poses)
        }
    
    def reset_calibration(self):
        """重置标定数据"""
        self.camera_poses.clear()
        self.imu_orientations.clear()
        self.R_imu2cam = None
        self.t_imu2cam = None
        self.is_calibrated = False
        self.get_logger().info('标定数据已重置')
    
    def cleanup(self):
        """清理资源"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = SimpleIMUCameraCalibration()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()