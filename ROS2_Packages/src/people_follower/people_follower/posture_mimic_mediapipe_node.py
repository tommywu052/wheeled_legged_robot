#!/usr/bin/env python3
"""
姿態模仿節點 - 使用 MediaPipe Pose 檢測人體姿態並控制機器人模仿
v1.5 - MediaPipe 版本

功能：
- 使用 MediaPipe Pose 檢測 33 個 3D 關鍵點
- 結合 RealSense 深度信息獲取真實 3D 姿態
- EMA 低通濾波平滑關節點
- Hysteresis 避免狀態抖動
- 分析姿態：高度、橫滾、俯仰
- 舉手檢測（站高功能）
- 發布到 /cmd_posture

優勢：
- 更低延遲
- 更穩定的骨架追蹤
- 更少抖動
- 更適合實時跟隨

作者：AI Assistant
日期：2026-01-02
版本：v1.5
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, JointState, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
import math

# MediaPipe
import mediapipe as mp


class PostureMimicMediaPipeNode(Node):
    """姿態模仿節點 - MediaPipe 版本"""
    
    # MediaPipe Pose 關鍵點索引（33個點）
    MP_POSE_LANDMARKS = {
        'NOSE': 0,
        'LEFT_EYE_INNER': 1, 'LEFT_EYE': 2, 'LEFT_EYE_OUTER': 3,
        'RIGHT_EYE_INNER': 4, 'RIGHT_EYE': 5, 'RIGHT_EYE_OUTER': 6,
        'LEFT_EAR': 7, 'RIGHT_EAR': 8,
        'MOUTH_LEFT': 9, 'MOUTH_RIGHT': 10,
        'LEFT_SHOULDER': 11, 'RIGHT_SHOULDER': 12,
        'LEFT_ELBOW': 13, 'RIGHT_ELBOW': 14,
        'LEFT_WRIST': 15, 'RIGHT_WRIST': 16,
        'LEFT_PINKY': 17, 'RIGHT_PINKY': 18,
        'LEFT_INDEX': 19, 'RIGHT_INDEX': 20,
        'LEFT_THUMB': 21, 'RIGHT_THUMB': 22,
        'LEFT_HIP': 23, 'RIGHT_HIP': 24,
        'LEFT_KNEE': 25, 'RIGHT_KNEE': 26,
        'LEFT_ANKLE': 27, 'RIGHT_ANKLE': 28,
        'LEFT_HEEL': 29, 'RIGHT_HEEL': 30,
        'LEFT_FOOT_INDEX': 31, 'RIGHT_FOOT_INDEX': 32
    }
    
    def __init__(self):
        super().__init__('posture_mimic_mediapipe_node')
        
        # ========== 參數聲明 ==========
        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('enable_mimic', True)
        self.declare_parameter('publish_rate', 30)  # Hz (MediaPipe 可以跑更快)
        
        # MediaPipe 參數
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('model_complexity', 1)  # 0=Lite, 1=Full, 2=Heavy
        self.declare_parameter('smooth_landmarks', True)  # MediaPipe 內建平滑
        
        # 姿態參數範圍
        self.declare_parameter('height_stand', 0.25)      # 站立高度
        self.declare_parameter('height_squat', 0.16)      # 蹲下高度
        self.declare_parameter('height_tall', 0.35)       # 站高高度
        self.declare_parameter('height_max', 0.36)        # 最大高度
        self.declare_parameter('height_min', 0.14)        # 最小高度
        self.declare_parameter('roll_max', 20.0)          # 橫滾最大角度（度）
        self.declare_parameter('pitch_max', 35.0)         # 俯仰最大角度（度）
        
        # EMA 濾波參數（指數移動平均）
        self.declare_parameter('ema_alpha_fast', 0.3)     # 快速濾波（用於關節點）
        self.declare_parameter('ema_alpha_slow', 0.1)     # 慢速濾波（用於高度）
        
        # Hysteresis 參數（防止狀態抖動）
        self.declare_parameter('height_hysteresis', 0.03) # 高度切換滯後（米）
        self.declare_parameter('hands_up_hysteresis', 0.1)# 舉手切換滯後
        
        # 變化速率限制
        self.declare_parameter('max_height_change', 0.15) # 單次最大高度變化（米）
        self.declare_parameter('max_roll_change', 5.0)    # 單次最大橫滾變化（度）
        self.declare_parameter('max_pitch_change', 15.0)  # 單次最大俯仰變化（度）
        
        # 功能開關
        self.declare_parameter('enable_height', True)
        self.declare_parameter('enable_roll', False)      # 可以開啟，MediaPipe 更穩
        self.declare_parameter('enable_pitch', True)
        self.declare_parameter('enable_hands_up', True)
        
        # 深度測量參數
        self.declare_parameter('camera_height', 0.3)      # 相機離地高度（米）
        self.declare_parameter('camera_pitch', 15.0)      # 相機俯仰角度（度）
        self.declare_parameter('depth_roi_size', 5)       # 深度 ROI 大小（像素）
        self.declare_parameter('use_depth_median', True)  # 使用中位數（更穩定）
        
        # 檢測超時
        self.declare_parameter('detection_timeout', 3.0)  # 秒
        
        # 獲取參數
        self.camera_topic = self.get_parameter('camera_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.enable_mimic = self.get_parameter('enable_mimic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        self.min_detection_confidence = self.get_parameter('min_detection_confidence').value
        self.min_tracking_confidence = self.get_parameter('min_tracking_confidence').value
        self.model_complexity = self.get_parameter('model_complexity').value
        self.smooth_landmarks = self.get_parameter('smooth_landmarks').value
        
        self.height_stand = self.get_parameter('height_stand').value
        self.height_squat = self.get_parameter('height_squat').value
        self.height_tall = self.get_parameter('height_tall').value
        self.height_max = self.get_parameter('height_max').value
        self.height_min = self.get_parameter('height_min').value
        self.roll_max = self.get_parameter('roll_max').value
        self.pitch_max = self.get_parameter('pitch_max').value
        
        self.ema_alpha_fast = self.get_parameter('ema_alpha_fast').value
        self.ema_alpha_slow = self.get_parameter('ema_alpha_slow').value
        self.height_hysteresis = self.get_parameter('height_hysteresis').value
        self.hands_up_hysteresis = self.get_parameter('hands_up_hysteresis').value
        
        self.max_height_change = self.get_parameter('max_height_change').value
        self.max_roll_change = self.get_parameter('max_roll_change').value
        self.max_pitch_change = self.get_parameter('max_pitch_change').value
        
        self.enable_height = self.get_parameter('enable_height').value
        self.enable_roll = self.get_parameter('enable_roll').value
        self.enable_pitch = self.get_parameter('enable_pitch').value
        self.enable_hands_up = self.get_parameter('enable_hands_up').value
        
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_pitch_deg = self.get_parameter('camera_pitch').value
        self.camera_pitch_rad = math.radians(self.camera_pitch_deg)
        self.depth_roi_size = self.get_parameter('depth_roi_size').value
        self.use_depth_median = self.get_parameter('use_depth_median').value
        
        self.detection_timeout = self.get_parameter('detection_timeout').value
        
        # ========== 初始化 MediaPipe ==========
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=self.model_complexity,
            smooth_landmarks=self.smooth_landmarks,
            enable_segmentation=False,
            smooth_segmentation=False,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence
        )
        
        self.get_logger().info(f'✅ MediaPipe Pose 已初始化 (complexity={self.model_complexity})')
        
        # ========== CV Bridge ==========
        self.bridge = CvBridge()
        
        # ========== 狀態變量 ==========
        self.current_posture = {
            'height': self.height_stand,
            'roll': 0.0,
            'pitch': 0.0
        }
        
        # EMA 濾波狀態（用於 3D 關鍵點）
        self.ema_landmarks_3d = None  # 33x3 array
        
        # 高度狀態（用於 hysteresis）
        self.height_state = 'NORMAL'  # 'SQUAT', 'NORMAL', 'TALL'
        self.height_raw = self.height_stand
        self.height_filtered = self.height_stand
        
        # 舉手狀態
        self.hands_up_state = False
        self.hands_up_raw_score = 0.0  # 0.0-1.0
        self.last_normal_height = self.height_stand
        
        # 檢測狀態
        self.last_detection_time = None
        self.detection_active = False
        
        # 相機內參
        self.camera_intrinsics = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # 深度圖
        self.latest_depth_image = None
        
        # ========== QoS 設置 ==========
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # ========== 訂閱者 ==========
        self.color_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile
        )
        
        # ========== 發布者 ==========
        self.posture_pub = self.create_publisher(JointState, '/cmd_posture', 10)
        self.image_pub = self.create_publisher(Image, '/posture_mimic/image', 10)
        self.status_pub = self.create_publisher(String, '/posture_mimic/status', 10)
        
        # ========== 定時器 ==========
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_posture)
        
        self.get_logger().info('🚀 姿態模仿節點已啟動 (MediaPipe Pose)')
        self.get_logger().info(f'模仿啟用: {self.enable_mimic}')
        self.get_logger().info(f'發布頻率: {self.publish_rate} Hz')
        self.get_logger().info(f'訂閱話題: {self.camera_topic}')
        self.get_logger().info(f'訂閱深度: {self.depth_topic}')
        self.get_logger().info(f'相機傾斜角度: {self.camera_pitch_deg}°')
        self.get_logger().info(f'發布話題: /cmd_posture')
        self.get_logger().info(f'EMA 濾波: fast={self.ema_alpha_fast}, slow={self.ema_alpha_slow}')
        self.get_logger().info(f'Hysteresis: height={self.height_hysteresis}m, hands_up={self.hands_up_hysteresis}')
    
    def camera_info_callback(self, msg):
        """接收相機內參"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = msg
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f'✅ 相機內參已接收: {msg.width}x{msg.height}')
            self.get_logger().info(f'   fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}')
    
    def depth_callback(self, msg):
        """接收深度圖"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'深度圖轉換錯誤: {e}')
    
    def image_callback(self, msg):
        """處理彩色圖像"""
        try:
            # 轉換為 OpenCV 格式
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 轉換為 RGB（MediaPipe 需要）
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # MediaPipe 處理
            results = self.pose.process(frame_rgb)
            
            # 檢查是否檢測到人
            if results.pose_landmarks:
                self.detection_active = True
                self.last_detection_time = self.get_clock().now()
                
                # 提取 3D 關鍵點（已經是歸一化坐標）
                landmarks_3d = self.extract_landmarks_3d(results.pose_landmarks, frame.shape)
                
                # EMA 濾波
                landmarks_3d_filtered = self.apply_ema_filter(landmarks_3d, self.ema_alpha_fast)
                
                # 分析姿態
                posture = self.analyze_posture(landmarks_3d_filtered, frame.shape)
                
                # 更新當前姿態
                if posture is not None:
                    self.current_posture = posture
                
                # 繪製骨架
                self.draw_skeleton(frame, results.pose_landmarks, frame.shape)
                
                # 繪製姿態信息
                self.draw_posture_info(frame, posture, 'ACTIVE')
                
            else:
                # 未檢測到人
                if self.last_detection_time is not None:
                    elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
                    if elapsed > self.detection_timeout:
                        self.detection_active = False
                        # 保持最後姿態（不重置）
                        if self.hands_up_state:
                            # 如果之前是舉手狀態，恢復到正常高度
                            self.hands_up_state = False
                            self.current_posture['height'] = self.last_normal_height
                
                self.draw_posture_info(frame, self.current_posture, 'WAITING')
            
            # 發布圖像
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(image_msg)
            
        except Exception as e:
            self.get_logger().error(f'處理錯誤: {e}')
    
    def extract_landmarks_3d(self, pose_landmarks, image_shape):
        """
        提取 3D 關鍵點並結合深度信息
        
        MediaPipe 提供：
        - x, y: 歸一化坐標 [0, 1]
        - z: 相對深度（相對於髖部中點）
        - visibility: 可見性 [0, 1]
        
        我們結合 RealSense 深度獲取真實 Z
        """
        h, w = image_shape[:2]
        landmarks = []
        
        for idx, lm in enumerate(pose_landmarks.landmark):
            # 像素坐標
            px = int(lm.x * w)
            py = int(lm.y * h)
            
            # 從深度圖獲取真實深度
            depth_z = self.get_depth_at_point(px, py)
            
            # 如果深度無效，使用 MediaPipe 的相對深度
            if depth_z is None or depth_z <= 0:
                depth_z = lm.z  # 相對深度（單位不確定）
            
            landmarks.append({
                'x': lm.x,  # 歸一化 [0, 1]
                'y': lm.y,  # 歸一化 [0, 1]
                'z': depth_z,  # 真實深度（米）或相對深度
                'visibility': lm.visibility,
                'px': px,  # 像素坐標
                'py': py
            })
        
        return landmarks
    
    def get_depth_at_point(self, px, py):
        """
        獲取指定點的深度值（使用 ROI 中位數或均值）
        """
        if self.latest_depth_image is None:
            return None
        
        h, w = self.latest_depth_image.shape[:2]
        
        # 邊界檢查
        if px < 0 or px >= w or py < 0 or py >= h:
            return None
        
        # ROI 範圍
        roi_size = self.depth_roi_size
        x1 = max(0, px - roi_size)
        x2 = min(w, px + roi_size + 1)
        y1 = max(0, py - roi_size)
        y2 = min(h, py + roi_size + 1)
        
        # 提取 ROI
        roi = self.latest_depth_image[y1:y2, x1:x2]
        
        # 過濾無效深度
        valid_depths = roi[roi > 0]
        
        if len(valid_depths) == 0:
            return None
        
        # 使用中位數（更穩定）或均值
        if self.use_depth_median:
            depth_mm = np.median(valid_depths)
        else:
            depth_mm = np.mean(valid_depths)
        
        # 轉換為米
        depth_m = depth_mm / 1000.0
        
        return depth_m
    
    def apply_ema_filter(self, landmarks, alpha):
        """
        對關鍵點應用 EMA（指數移動平均）濾波
        
        EMA 公式：
        filtered[t] = alpha * raw[t] + (1 - alpha) * filtered[t-1]
        
        alpha 越大，響應越快，但越抖
        alpha 越小，越平滑，但延遲越大
        """
        # 轉換為 numpy 數組
        current = np.array([[lm['x'], lm['y'], lm['z']] for lm in landmarks])
        
        # 初始化
        if self.ema_landmarks_3d is None:
            self.ema_landmarks_3d = current
            filtered_landmarks = landmarks.copy()
        else:
            # EMA 濾波
            self.ema_landmarks_3d = alpha * current + (1 - alpha) * self.ema_landmarks_3d
            
            # 更新 landmarks
            filtered_landmarks = []
            for idx, lm in enumerate(landmarks):
                filtered_lm = lm.copy()
                filtered_lm['x'] = self.ema_landmarks_3d[idx, 0]
                filtered_lm['y'] = self.ema_landmarks_3d[idx, 1]
                filtered_lm['z'] = self.ema_landmarks_3d[idx, 2]
                filtered_landmarks.append(filtered_lm)
        
        return filtered_landmarks
    
    def analyze_posture(self, landmarks, image_shape):
        """
        分析姿態
        
        返回：
        {
            'height': float (米),
            'roll': float (度),
            'pitch': float (度)
        }
        """
        h, w = image_shape[:2]
        
        # 提取關鍵點索引
        idx = self.MP_POSE_LANDMARKS
        
        # ========== 1. 計算高度 ==========
        height = None
        if self.enable_height:
            height = self.calculate_height_from_3d(landmarks)
            
            # 應用 hysteresis（防止狀態抖動）
            height = self.apply_height_hysteresis(height)
            
            # 檢測舉手
            if self.enable_hands_up:
                hands_up = self.detect_hands_up(landmarks)
                if hands_up and not self.hands_up_state:
                    # 進入舉手狀態
                    self.hands_up_state = True
                    self.last_normal_height = self.height_filtered
                    height = self.height_tall
                    self.get_logger().info('🙌 偵測到舉手！機器人站高')
                elif not hands_up and self.hands_up_state:
                    # 退出舉手狀態
                    self.hands_up_state = False
                    height = self.last_normal_height
                    self.get_logger().info('👋 手放下，機器人恢復正常高度')
                elif self.hands_up_state:
                    # 保持舉手狀態
                    height = self.height_tall
        
        # ========== 2. 計算橫滾（Roll）==========
        roll = 0.0
        if self.enable_roll:
            roll = self.calculate_roll(landmarks)
        
        # ========== 3. 計算俯仰（Pitch）==========
        pitch = 0.0
        if self.enable_pitch:
            pitch = self.calculate_pitch(landmarks)
        
        return {
            'height': height if height is not None else self.height_stand,
            'roll': roll,
            'pitch': pitch
        }
    
    def calculate_height_from_3d(self, landmarks):
        """
        使用 3D 關鍵點計算高度
        
        方法：
        1. 使用鼻子或髖部的 Y 坐標（像素）+ 深度
        2. 轉換為世界坐標系的高度
        3. 映射到機器人高度
        """
        idx = self.MP_POSE_LANDMARKS
        
        # 優先使用鼻子
        nose = landmarks[idx['NOSE']]
        left_hip = landmarks[idx['LEFT_HIP']]
        right_hip = landmarks[idx['RIGHT_HIP']]
        
        # 選擇可見性最高的點
        if nose['visibility'] > 0.5 and nose['z'] > 0:
            # 使用鼻子
            py = nose['py']
            depth = nose['z']
            
            # 像素坐標轉相機坐標
            if self.cy is not None:
                y_cam = (py - self.cy) * depth / self.fy
            else:
                y_cam = (py - 240) * depth / 525.0  # 默認值
            
            # 相機坐標轉世界坐標（考慮相機傾斜）
            y_world = y_cam * math.cos(self.camera_pitch_rad) - depth * math.sin(self.camera_pitch_rad)
            
            # 計算頭部高度（離地）
            head_height = self.camera_height - y_world
            
            # 映射到機器人高度
            # 人站立時頭部約 1.6-1.8m，蹲下約 0.8-1.2m
            # 映射到機器人 0.16-0.30m
            if head_height > 1.4:
                # 站立
                robot_height = np.interp(head_height, [1.4, 1.8], [self.height_stand, self.height_stand + 0.05])
            elif head_height < 1.0:
                # 蹲下
                robot_height = np.interp(head_height, [0.6, 1.0], [self.height_squat, self.height_stand])
            else:
                # 中間狀態
                robot_height = np.interp(head_height, [1.0, 1.4], [self.height_stand, self.height_stand])
            
            # 限制範圍
            robot_height = np.clip(robot_height, self.height_min, self.height_max)
            
            return robot_height
        
        elif (left_hip['visibility'] > 0.5 or right_hip['visibility'] > 0.5):
            # 使用髖部
            if left_hip['visibility'] > right_hip['visibility']:
                hip = left_hip
            else:
                hip = right_hip
            
            py = hip['py']
            depth = hip['z']
            
            if depth <= 0:
                return None
            
            # 像素坐標轉相機坐標
            if self.cy is not None:
                y_cam = (py - self.cy) * depth / self.fy
            else:
                y_cam = (py - 240) * depth / 525.0
            
            # 相機坐標轉世界坐標
            y_world = y_cam * math.cos(self.camera_pitch_rad) - depth * math.sin(self.camera_pitch_rad)
            
            # 計算髖部高度
            hip_height = self.camera_height - y_world
            
            # 映射到機器人高度
            # 人站立時髖部約 0.9-1.1m，蹲下約 0.3-0.6m
            if hip_height > 0.8:
                # 站立
                robot_height = np.interp(hip_height, [0.8, 1.2], [self.height_stand, self.height_stand + 0.05])
            else:
                # 蹲下
                robot_height = np.interp(hip_height, [0.2, 0.8], [self.height_squat, self.height_stand])
            
            # 限制範圍
            robot_height = np.clip(robot_height, self.height_min, self.height_max)
            
            return robot_height
        
        return None
    
    def apply_height_hysteresis(self, height_raw):
        """
        對高度應用 Hysteresis（滯後）避免狀態抖動
        
        原理：
        - 狀態切換需要跨過閾值 + hysteresis
        - 避免在臨界點來回切換
        """
        if height_raw is None:
            return self.height_filtered
        
        # 保存原始高度
        self.height_raw = height_raw
        
        # 確定當前應該處於的狀態
        squat_threshold = self.height_squat + self.height_hysteresis
        normal_threshold = self.height_stand - self.height_hysteresis
        
        if self.height_state == 'SQUAT':
            # 當前是蹲下狀態，需要超過閾值才切換到正常
            if height_raw > squat_threshold:
                self.height_state = 'NORMAL'
                self.get_logger().info(f'🔄 狀態切換: SQUAT → NORMAL (高度={height_raw:.3f}m)')
        
        elif self.height_state == 'NORMAL':
            # 當前是正常狀態
            if height_raw < normal_threshold:
                self.height_state = 'SQUAT'
                self.get_logger().info(f'🔄 狀態切換: NORMAL → SQUAT (高度={height_raw:.3f}m)')
            elif height_raw > self.height_stand + self.height_hysteresis:
                # 可以擴展為 TALL 狀態（如果需要）
                pass
        
        # 應用慢速 EMA 濾波到高度
        self.height_filtered = self.ema_alpha_slow * height_raw + (1 - self.ema_alpha_slow) * self.height_filtered
        
        # 速率限制
        delta = self.height_filtered - self.current_posture['height']
        if abs(delta) > self.max_height_change:
            delta = np.sign(delta) * self.max_height_change
            self.height_filtered = self.current_posture['height'] + delta
        
        return self.height_filtered
    
    def detect_hands_up(self, landmarks):
        """
        檢測舉手姿態
        
        判斷條件：
        1. 雙手腕高於肩膀
        2. 雙手腕接近頭部
        3. 站立姿態（不是蹲下）
        
        返回：
        - True: 舉手
        - False: 沒有舉手
        """
        idx = self.MP_POSE_LANDMARKS
        
        # 提取關鍵點
        nose = landmarks[idx['NOSE']]
        left_wrist = landmarks[idx['LEFT_WRIST']]
        right_wrist = landmarks[idx['RIGHT_WRIST']]
        left_shoulder = landmarks[idx['LEFT_SHOULDER']]
        right_shoulder = landmarks[idx['RIGHT_SHOULDER']]
        left_hip = landmarks[idx['LEFT_HIP']]
        right_hip = landmarks[idx['RIGHT_HIP']]
        
        # 檢查可見性
        if (nose['visibility'] < 0.5 or
            left_wrist['visibility'] < 0.3 or right_wrist['visibility'] < 0.3 or
            left_shoulder['visibility'] < 0.5 or right_shoulder['visibility'] < 0.5):
            return False
        
        # 1. 計算手腕相對於肩膀的高度（Y 坐標，越小越高）
        avg_shoulder_y = (left_shoulder['y'] + right_shoulder['y']) / 2
        left_wrist_above = (avg_shoulder_y - left_wrist['y'])
        right_wrist_above = (avg_shoulder_y - right_wrist['y'])
        
        # 2. 計算手腕相對於頭部的距離
        nose_y = nose['y']
        left_wrist_to_head = abs(left_wrist['y'] - nose_y)
        right_wrist_to_head = abs(right_wrist['y'] - nose_y)
        
        # 3. 檢查是否站立（排除蹲下時的誤判）
        # 計算軀幹長度
        avg_hip_y = (left_hip['y'] + right_hip['y']) / 2
        torso_length = avg_hip_y - avg_shoulder_y
        
        # 舉手判斷
        hands_up_score = 0.0
        
        # 條件 1：手腕高於肩膀
        if left_wrist_above > 0.05 and right_wrist_above > 0.05:
            hands_up_score += 0.4
        
        # 條件 2：手腕接近頭部
        if left_wrist_to_head < 0.15 and right_wrist_to_head < 0.15:
            hands_up_score += 0.4
        
        # 條件 3：站立姿態（軀幹長度合理）
        if torso_length > 0.15 and torso_length < 0.4:
            hands_up_score += 0.2
        
        # 保存原始分數
        self.hands_up_raw_score = hands_up_score
        
        # Hysteresis
        if self.hands_up_state:
            # 當前是舉手狀態，需要分數低於 (0.5 - hysteresis) 才退出
            threshold = 0.5 - self.hands_up_hysteresis
            return hands_up_score > threshold
        else:
            # 當前不是舉手狀態，需要分數高於 (0.5 + hysteresis) 才進入
            threshold = 0.5 + self.hands_up_hysteresis
            return hands_up_score > threshold
    
    def calculate_roll(self, landmarks):
        """
        計算橫滾（Roll）- 左右搖擺
        
        使用肩膀連線的傾斜角度
        """
        idx = self.MP_POSE_LANDMARKS
        
        left_shoulder = landmarks[idx['LEFT_SHOULDER']]
        right_shoulder = landmarks[idx['RIGHT_SHOULDER']]
        
        if left_shoulder['visibility'] < 0.5 or right_shoulder['visibility'] < 0.5:
            return 0.0
        
        # 計算肩膀連線的角度
        dx = right_shoulder['x'] - left_shoulder['x']
        dy = right_shoulder['y'] - left_shoulder['y']
        
        # 角度（度）
        roll_rad = math.atan2(dy, dx)
        roll_deg = math.degrees(roll_rad)
        
        # 限制範圍
        roll_deg = np.clip(roll_deg, -self.roll_max, self.roll_max)
        
        return roll_deg
    
    def calculate_pitch(self, landmarks):
        """
        計算俯仰（Pitch）- 抬頭/低頭
        
        方法 A：耳朵-鼻子角度（優先）
        方法 B：鼻子-肩膀位置
        """
        idx = self.MP_POSE_LANDMARKS
        
        nose = landmarks[idx['NOSE']]
        left_ear = landmarks[idx['LEFT_EAR']]
        right_ear = landmarks[idx['RIGHT_EAR']]
        left_shoulder = landmarks[idx['LEFT_SHOULDER']]
        right_shoulder = landmarks[idx['RIGHT_SHOULDER']]
        
        # 方法 A：耳朵-鼻子角度
        if (left_ear['visibility'] > 0.5 or right_ear['visibility'] > 0.5) and nose['visibility'] > 0.5:
            # 計算平均耳朵位置
            if left_ear['visibility'] > 0.5 and right_ear['visibility'] > 0.5:
                avg_ear_y = (left_ear['y'] + right_ear['y']) / 2
            elif left_ear['visibility'] > 0.5:
                avg_ear_y = left_ear['y']
            else:
                avg_ear_y = right_ear['y']
            
            # 計算俯仰比例
            pitch_ratio = (nose['y'] - avg_ear_y)
            
            # 離散化為 3 個狀態
            if pitch_ratio > 0.03:
                pitch_deg = -30.0  # 低頭
            elif pitch_ratio < -0.03:
                pitch_deg = 30.0   # 抬頭
            else:
                pitch_deg = 0.0    # 正常
            
            return pitch_deg
        
        # 方法 B：鼻子-肩膀位置
        if nose['visibility'] > 0.5 and (left_shoulder['visibility'] > 0.5 or right_shoulder['visibility'] > 0.5):
            avg_shoulder_y = (left_shoulder['y'] + right_shoulder['y']) / 2
            pitch_ratio = (nose['y'] - avg_shoulder_y)
            
            # 離散化
            if pitch_ratio < -0.25:
                pitch_deg = 30.0   # 抬頭
            elif pitch_ratio > -0.15:
                pitch_deg = -30.0  # 低頭
            else:
                pitch_deg = 0.0    # 正常
            
            return pitch_deg
        
        return 0.0
    
    def draw_skeleton(self, frame, pose_landmarks, image_shape):
        """
        繪製骨架
        
        使用 MediaPipe 內建的繪圖工具
        """
        # 使用 MediaPipe 的標準繪製
        self.mp_drawing.draw_landmarks(
            frame,
            pose_landmarks,
            self.mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
        )
    
    def draw_posture_info(self, frame, posture, status):
        """繪製姿態信息"""
        h, w = frame.shape[:2]
        
        # 背景框
        cv2.rectangle(frame, (10, 10), (350, 180), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (350, 180), (0, 255, 0), 2)
        
        # 標題
        cv2.putText(frame, f'MediaPipe Pose v1.5', (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 狀態
        status_color = (0, 255, 0) if status == 'ACTIVE' else (0, 165, 255)
        cv2.putText(frame, f'Status: {status}', (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        # 姿態信息
        if posture:
            height = posture.get('height', 0.0)
            roll = posture.get('roll', 0.0)
            pitch = posture.get('pitch', 0.0)
            
            cv2.putText(frame, f'Height: {height:.3f}m', (20, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f'Roll:   {roll:.1f} deg', (20, 125),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f'Pitch:  {pitch:.1f} deg', (20, 155),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 舉手狀態
        if self.hands_up_state:
            cv2.putText(frame, '🙌 HANDS UP!', (w - 200, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # 高度狀態（用於調試）
        state_text = f'State: {self.height_state}'
        cv2.putText(frame, state_text, (w - 200, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    def publish_posture(self):
        """定時發布姿態命令"""
        if not self.enable_mimic:
            return
        
        # 創建 JointState 消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['body_height', 'body_roll', 'body_pitch']
        msg.position = [
            self.current_posture['height'],
            math.radians(self.current_posture['roll']),
            math.radians(self.current_posture['pitch'])
        ]
        
        # 發布
        self.posture_pub.publish(msg)
        
        # 發布狀態
        status_msg = String()
        if self.detection_active:
            status_msg.data = 'ACTIVE'
        else:
            status_msg.data = 'WAITING'
        self.status_pub.publish(status_msg)
    
    def destroy_node(self):
        """清理資源"""
        self.pose.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PostureMimicMediaPipeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

