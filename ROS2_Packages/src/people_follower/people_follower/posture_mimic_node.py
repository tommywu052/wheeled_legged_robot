#!/usr/bin/env python3
"""
姿態模仿節點 - 使用 YOLOv8-Pose 檢測人體姿態並控制機器人模仿

功能：
- 檢測人體關鍵點（17個關鍵點）
- 分析姿態：高度（站/蹲）、橫滾（左右搖擺）、俯仰（抬頭/低頭）
- 映射到機器人控制命令
- 發布到 /cmd_posture

作者：AI Assistant
日期：2026-01-01
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, JointState, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import torch
from collections import deque
import math


class PostureMimicNode(Node):
    """姿態模仿節點"""
    
    def __init__(self):
        super().__init__('posture_mimic_node')
        
        # ========== 參數聲明 ==========
        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('yolo_model', 'yolov8n-pose.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('enable_mimic', True)
        self.declare_parameter('publish_rate', 20)  # Hz
        
        # 姿態參數範圍
        self.declare_parameter('height_stand', 0.30)      # 站立高度
        self.declare_parameter('height_squat', 0.16)      # 蹲下高度
        self.declare_parameter('height_max', 0.36)        # 最大高度
        self.declare_parameter('height_min', 0.14)        # 最小高度
        self.declare_parameter('roll_max', 20.0)          # 橫滾最大角度（度）
        self.declare_parameter('pitch_max', 20.0)         # 俯仰最大角度（度）
        
        # 平滑濾波參數
        self.declare_parameter('smoothing_window', 10)    # 滑動平均窗口大小（增加到10幀）
        self.declare_parameter('min_confidence', 0.3)     # 關鍵點最小置信度
        
        # 變化閾值參數（防止抖動）
        self.declare_parameter('height_deadzone', 0.05)   # 高度變化死區（米）
        self.declare_parameter('roll_deadzone', 3.0)      # 橫滾變化死區（度）
        self.declare_parameter('pitch_deadzone', 3.0)     # 俯仰變化死區（度）
        
        # 變化速率限制（防止突變）
        self.declare_parameter('max_height_change', 0.02) # 單次最大高度變化（米）
        self.declare_parameter('max_roll_change', 2.0)    # 單次最大橫滾變化（度）
        self.declare_parameter('max_pitch_change', 2.0)   # 單次最大俯仰變化（度）
        
        # 功能開關
        self.declare_parameter('enable_height', True)     # 啟用高度模仿
        self.declare_parameter('enable_roll', False)      # 啟用橫滾模仿（暫時關閉）
        self.declare_parameter('enable_pitch', False)     # 啟用俯仰模仿（暫時關閉）
        
        # 獲取參數
        self.camera_topic = self.get_parameter('camera_topic').value
        self.yolo_model_name = self.get_parameter('yolo_model').value
        self.confidence_threshold = self.get_parameter('confidence').value
        self.device = self.get_parameter('device').value
        self.enable_mimic = self.get_parameter('enable_mimic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        self.height_stand = self.get_parameter('height_stand').value
        self.height_squat = self.get_parameter('height_squat').value
        self.height_max = self.get_parameter('height_max').value
        self.height_min = self.get_parameter('height_min').value
        self.roll_max = self.get_parameter('roll_max').value
        self.pitch_max = self.get_parameter('pitch_max').value
        
        self.smoothing_window = self.get_parameter('smoothing_window').value
        self.min_confidence = self.get_parameter('min_confidence').value
        
        self.height_deadzone = self.get_parameter('height_deadzone').value
        self.roll_deadzone = self.get_parameter('roll_deadzone').value
        self.pitch_deadzone = self.get_parameter('pitch_deadzone').value
        
        self.max_height_change = self.get_parameter('max_height_change').value
        self.max_roll_change = self.get_parameter('max_roll_change').value
        self.max_pitch_change = self.get_parameter('max_pitch_change').value
        
        self.enable_height = self.get_parameter('enable_height').value
        self.enable_roll = self.get_parameter('enable_roll').value
        self.enable_pitch = self.get_parameter('enable_pitch').value
        
        # 舉手姿態檢測參數
        self.declare_parameter('hands_up_threshold', 0.5)
        self.hands_up_threshold = self.get_parameter('hands_up_threshold').value
        
        # ========== 初始化 YOLOv8-Pose ==========
        self.get_logger().info(f"📦 載入 YOLOv8-Pose 模型: {self.yolo_model_name}...")
        try:
            # 檢查 CUDA 可用性
            if 'cuda' in self.device and not torch.cuda.is_available():
                self.get_logger().warn("⚠️  CUDA不可用，將使用CPU（會較慢）")
                self.device = 'cpu'
            
            # TensorRT 優化：優先使用 .engine 文件
            import os
            model_path = self.yolo_model_name
            
            # 如果是 .pt 模型，檢查是否有對應的 .engine 文件
            if model_path.endswith('.pt'):
                engine_path = model_path.replace('.pt', '.engine')
                
                if os.path.exists(engine_path):
                    # 使用已存在的 TensorRT 引擎
                    self.get_logger().info(f"🚀 發現 TensorRT 引擎: {engine_path}")
                    self.model = YOLO(engine_path)
                    self.get_logger().info("✅ 已載入 TensorRT 引擎（高性能模式）")
                    self.get_logger().info(f"   預期性能提升: 2-4倍（相比 PyTorch）")
                else:
                    # 載入 PyTorch 模型並提示轉換
                    self.get_logger().info(f"📥 載入 PyTorch 模型: {model_path}")
                    self.model = YOLO(model_path)
                    self.get_logger().info("✅ PyTorch 模型已載入")
                    
                    if 'cuda' in self.device:
                        self.get_logger().warn("⚠️  建議轉換為 TensorRT 以提升性能！")
                        self.get_logger().warn(f"   執行命令: yolo export model={model_path} format=engine")
                        self.get_logger().warn(f"   或在 Python 中執行:")
                        self.get_logger().warn(f"   >>> from ultralytics import YOLO")
                        self.get_logger().warn(f"   >>> model = YOLO('{model_path}')")
                        self.get_logger().warn(f"   >>> model.export(format='engine')")
            else:
                # 直接載入指定的模型（可能是 .engine 或其他格式）
                self.model = YOLO(model_path)
                model_type = "TensorRT" if model_path.endswith('.engine') else "PyTorch"
                self.get_logger().info(f"✅ {model_type} 模型已載入")
            
            self.get_logger().info(f"   裝置: {self.device}")
                
        except Exception as e:
            self.get_logger().error(f"❌ 載入 YOLOv8-Pose 模型失敗: {e}")
            self.get_logger().error("   請確保已安裝: pip3 install ultralytics")
            rclpy.shutdown()
            return
        
        # ========== CvBridge ==========
        self.bridge = CvBridge()
        
        # ========== QoS 設置 ==========
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # ========== 訂閱器 ==========
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile
        )
        
        # 訂閱深度圖像（用於準確的高度測量）
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            qos_profile
        )
        
        # 訂閱相機內參
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            qos_profile
        )
        
        # ========== 發布器 ==========
        self.posture_pub = self.create_publisher(JointState, '/cmd_posture', 10)
        self.debug_image_pub = self.create_publisher(Image, '/posture_mimic/debug_image', 10)
        self.status_pub = self.create_publisher(String, '/posture_mimic/status', 10)
        
        # ========== 定時發布器 ==========
        # 以固定頻率發布姿態命令（避免超時）
        self.posture_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_posture_command
        )
        
        # ========== 狀態變量 ==========
        self.current_posture = {
            'height': self.height_stand,
            'roll': 0.0,
            'pitch': 0.0
        }
        self.last_detection_time = self.get_clock().now()
        self.posture_reset = False  # 標記是否已經重置姿態（用於超時處理）
        
        # 檢測超時參數
        self.declare_parameter('detection_timeout', 3.0)
        self.detection_timeout = self.get_parameter('detection_timeout').value
        
        # Depth 圖像和相機參數
        self.latest_depth = None
        self.camera_intrinsics = None
        
        # 相機安裝參數（可通過配置文件調整）
        self.declare_parameter('camera_pitch_deg', 15.0)
        self.declare_parameter('camera_height', 0.3)  # 實測值：30cm
        
        camera_pitch_deg = self.get_parameter('camera_pitch_deg').value
        self.camera_pitch = math.radians(camera_pitch_deg)  # 轉換為弧度
        self.camera_height_from_ground = self.get_parameter('camera_height').value
        
        # 平滑濾波緩衝區
        self.height_buffer = deque(maxlen=self.smoothing_window)
        self.roll_buffer = deque(maxlen=self.smoothing_window)
        self.pitch_buffer = deque(maxlen=self.smoothing_window)
        
        # 統計信息
        self.frame_count = 0
        self.last_fps_time = self.get_clock().now()
        self.fps = 0.0
        
        # 舉手姿態狀態（用於減少日誌輸出）
        self.hands_up_state = False
        
        # 記錄最後的正常高度（非舉手狀態的高度）
        self.last_normal_height = self.height_stand
        
        # ========== YOLO 關鍵點索引 ==========
        # YOLOv8-Pose 檢測 17 個關鍵點
        self.KEYPOINT_NAMES = [
            'nose',           # 0
            'left_eye',       # 1
            'right_eye',      # 2
            'left_ear',       # 3
            'right_ear',      # 4
            'left_shoulder',  # 5
            'right_shoulder', # 6
            'left_elbow',     # 7
            'right_elbow',    # 8
            'left_wrist',     # 9
            'right_wrist',    # 10
            'left_hip',       # 11
            'right_hip',      # 12
            'left_knee',      # 13
            'right_knee',     # 14
            'left_ankle',     # 15
            'right_ankle'     # 16
        ]
        
        self.get_logger().info("🚀 姿態模仿節點已啟動")
        self.get_logger().info(f"   模仿啟用: {self.enable_mimic}")
        self.get_logger().info(f"   發布頻率: {self.publish_rate} Hz")
        self.get_logger().info(f"   訂閱話題: {self.camera_topic}")
        self.get_logger().info(f"   訂閱深度: /camera/camera/aligned_depth_to_color/image_raw")
        self.get_logger().info(f"   相機傾斜角度: {math.degrees(self.camera_pitch):.1f}°")
        self.get_logger().info(f"   發布話題: /cmd_posture")
    
    
    def depth_callback(self, msg):
        """接收深度圖像"""
        try:
            # 轉換為 NumPy 數組（單位：毫米）
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"深度圖像轉換錯誤: {e}")
    
    
    def camera_info_callback(self, msg):
        """接收相機內參"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = {
                'fx': msg.k[0],  # 焦距 x
                'fy': msg.k[4],  # 焦距 y
                'cx': msg.k[2],  # 光心 x
                'cy': msg.k[5],  # 光心 y
                'width': msg.width,
                'height': msg.height
            }
            self.get_logger().info(f"✅ 相機內參已接收: {msg.width}x{msg.height}")
    
    
    def image_callback(self, msg):
        """處理影像並檢測人體姿態"""
        try:
            # 轉換為 OpenCV 格式
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # YOLOv8-Pose 推理
            results = self.model(
                frame,
                conf=self.confidence_threshold,
                verbose=False,
                device=self.device
            )
            
            # 處理檢測結果
            person_detected = False
            for r in results:
                # ===== 繪製檢測框 ⭐ 新增 =====
                if r.boxes is not None and len(r.boxes) > 0:
                    for box in r.boxes:
                        # 獲取邊界框坐標
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = box.conf[0].cpu().numpy()
                        cls = int(box.cls[0].cpu().numpy())
                        
                        # 只顯示人（class 0）
                        if cls == 0:  # person class
                            # 繪製矩形框
                            cv2.rectangle(frame, 
                                         (int(x1), int(y1)), 
                                         (int(x2), int(y2)), 
                                         (0, 0, 255), 2)  # 紅色框
                            
                            # 顯示標籤
                            label = f"person {conf:.2f}"
                            cv2.putText(frame, label, 
                                       (int(x1), int(y1) - 10),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, 
                                       (0, 0, 255), 2)  # 紅色文字
                
                # ===== 繪製骨架和姿態分析 =====
                if r.keypoints is not None and len(r.keypoints) > 0:
                    # 獲取第一個人的關鍵點（最大的檢測框）
                    keypoints = r.keypoints[0]
                    
                    # 檢查 keypoints.data 是否有效
                    if keypoints.data is not None and len(keypoints.data) > 0:
                        # 分析姿態（傳遞 depth 圖像）
                        posture = self.analyze_posture(
                            keypoints, 
                            frame.shape,
                            depth_image=self.latest_depth
                        )
                        
                        if posture is not None:
                            # 更新當前姿態（平滑濾波）
                            self.update_posture_smoothed(posture)
                            person_detected = True
                            self.last_detection_time = self.get_clock().now()
                            self.posture_reset = False  # 重新檢測到人，重置標誌
                        
                        # 繪製骨架（只繪製第一個人）
                        self.draw_skeleton(frame, keypoints)
                    
                    break
            
            # 如果沒有檢測到人，檢查超時
            if not person_detected:
                elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
                
                # 每隔1秒輸出一次警告
                if int(elapsed) % 1 == 0 and elapsed < self.detection_timeout:
                    self.get_logger().warn(
                        f"未檢測到人！已經 {elapsed:.1f}s（超時 {self.detection_timeout}s）",
                        throttle_duration_sec=1.0
                    )
                
                if elapsed > self.detection_timeout:
                    # 超時後重置到默認姿態（只執行一次）
                    if not self.posture_reset:
                        self.get_logger().warn(
                            f"⚠️ 檢測超時 {self.detection_timeout}s！重置姿態到初始狀態 "
                            f"(高度: {self.height_stand}m, 橫滾: 0°, 俯仰: 0°)"
                        )
                        self.reset_to_default_posture()
                        self.hands_up_state = False
                        self.posture_reset = True  # 標記已重置
            
            # 繪製姿態信息
            self.draw_posture_info(frame, person_detected)
            
            # 計算 FPS
            self.update_fps()
            
            # 發布調試圖像
            if self.debug_image_pub.get_subscription_count() > 0:
                debug_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f"處理錯誤: {e}")
    
    
    def detect_hands_up(self, kpts):
        """檢測雙手是否舉高（投降姿態）
        
        判斷標準（更嚴格，避免蹲下時誤判）：
        1. 左右手腕都可見（置信度足夠）
        2. 左右肩膀都可見
        3. 鼻子（頭部）可見
        4. 雙手腕的 y 坐標都顯著低於對應肩膀（手在上方）
        5. 頭部位置必須相對較高（站立狀態，排除蹲下）⭐
        6. 手腕必須在頭部附近或上方（真正的舉手）⭐
        
        參數：
            kpts: 關鍵點數組 [17, 3]，格式 [x, y, confidence]
        
        返回：
            True 如果檢測到舉手姿態，否則 False
        """
        NOSE = 0
        LEFT_SHOULDER = 5
        RIGHT_SHOULDER = 6
        LEFT_WRIST = 9
        RIGHT_WRIST = 10
        LEFT_HIP = 11
        RIGHT_HIP = 12
        
        # 檢查關鍵點置信度
        if (kpts[LEFT_SHOULDER][2] < self.min_confidence or 
            kpts[RIGHT_SHOULDER][2] < self.min_confidence or
            kpts[LEFT_WRIST][2] < self.min_confidence or
            kpts[RIGHT_WRIST][2] < self.min_confidence or
            kpts[NOSE][2] < self.min_confidence):
            return False
        
        # 獲取肩膀和手腕的 y 坐標（注意：y 軸向下為正）
        left_shoulder_y = kpts[LEFT_SHOULDER][1]
        right_shoulder_y = kpts[RIGHT_SHOULDER][1]
        left_wrist_y = kpts[LEFT_WRIST][1]
        right_wrist_y = kpts[RIGHT_WRIST][1]
        nose_y = kpts[NOSE][1]
        
        # 計算平均肩膀高度
        shoulder_y_avg = (left_shoulder_y + right_shoulder_y) / 2
        
        # ========== 新增：檢查是否為蹲下狀態 ==========
        # 如果髖部可見，檢查身體是否彎曲（蹲下）
        if (kpts[LEFT_HIP][2] > self.min_confidence and 
            kpts[RIGHT_HIP][2] > self.min_confidence):
            left_hip_y = kpts[LEFT_HIP][1]
            right_hip_y = kpts[RIGHT_HIP][1]
            hip_y_avg = (left_hip_y + right_hip_y) / 2
            
            # 計算軀幹長度（肩膀到髖部的垂直距離）
            torso_length = abs(hip_y_avg - shoulder_y_avg)
            
            # 如果軀幹長度太短，可能是蹲下狀態（身體壓縮）
            # 正常站立時軀幹長度應該較大
            shoulder_width = abs(kpts[RIGHT_SHOULDER][0] - kpts[LEFT_SHOULDER][0])
            if torso_length < shoulder_width * 0.8:  # 軀幹長度小於肩寬的 80%
                # 可能是蹲下，禁用舉手檢測
                return False
        
        # ========== 新增：檢查手腕是否在頭部附近 ==========
        # 真正的舉手姿態，手腕應該在頭部附近或上方
        left_wrist_to_head = abs(nose_y - left_wrist_y)
        right_wrist_to_head = abs(nose_y - right_wrist_y)
        head_to_shoulder = abs(nose_y - shoulder_y_avg)
        
        # 手腕必須接近頭部（在頭部上下一定範圍內）
        # 如果手腕離頭部太遠，可能不是真正的舉手
        if left_wrist_to_head > head_to_shoulder * 2 or right_wrist_to_head > head_to_shoulder * 2:
            return False
        
        # ========== 原有檢測邏輯 ==========
        # 檢測閾值：手腕需要高於肩膀一定距離（像素）
        # 使用肩膀之間的距離作為參考
        shoulder_width = abs(kpts[RIGHT_SHOULDER][0] - kpts[LEFT_SHOULDER][0])
        threshold = shoulder_width * self.hands_up_threshold  # 使用配置的閾值比例
        
        # 最小閾值（避免太小）
        threshold = max(threshold, 40)  # 至少 40 像素
        
        # 判斷：雙手都在肩膀上方且超過閾值
        left_hand_up = (left_shoulder_y - left_wrist_y) > threshold
        right_hand_up = (right_shoulder_y - right_wrist_y) > threshold
        
        # 所有條件都滿足才返回 True
        return left_hand_up and right_hand_up
    
    
    def detect_left_hand_up(self, kpts):
        """檢測左手舉高（單手）
        
        用於橫滾控制：左手舉高 → 機器人向左橫滾
        
        要求：手腕必須明顯高於肩膀（不是平舉），接近頭部
        
        參數：
            kpts: 關鍵點數組 [17, 3]
        
        返回：
            bool: True 表示左手舉高
        """
        # 關鍵點索引
        LEFT_WRIST = 9
        LEFT_SHOULDER = 5
        RIGHT_WRIST = 10
        RIGHT_SHOULDER = 6
        NOSE = 0
        
        # 檢查必要關鍵點的置信度
        if (kpts[LEFT_WRIST][2] < self.min_confidence or 
            kpts[LEFT_SHOULDER][2] < self.min_confidence or
            kpts[NOSE][2] < self.min_confidence):
            return False
        
        # 獲取關鍵點坐標
        left_wrist_y = kpts[LEFT_WRIST][1]
        left_shoulder_y = kpts[LEFT_SHOULDER][1]
        nose_y = kpts[NOSE][1]
        
        # 計算參考距離：肩膀到鼻子的垂直距離（頭頸長度）
        shoulder_to_nose_dist = abs(left_shoulder_y - nose_y)
        
        # 條件 1：左手腕明顯高於左肩（不是平舉）
        # 使用相對閾值：至少高於肩膀 0.4 倍的頭頸長度
        # 或絕對閾值 60 像素（取較大值，避免平舉誤觸發）
        min_raise_dist = max(60, shoulder_to_nose_dist * 0.4)
        hand_raised_high = (left_shoulder_y - left_wrist_y) > min_raise_dist
        
        # 條件 2：左手腕接近頭部高度（在鼻子上下範圍內）
        wrist_near_head = abs(left_wrist_y - nose_y) < 100  # 100 像素範圍內
        
        # 條件 3：右手沒有舉起（排除雙手舉高）
        right_hand_down = True
        if kpts[RIGHT_WRIST][2] > self.min_confidence and kpts[RIGHT_SHOULDER][2] > self.min_confidence:
            right_wrist_y = kpts[RIGHT_WRIST][1]
            right_shoulder_y = kpts[RIGHT_SHOULDER][1]
            # 使用相同的嚴格閾值檢測右手
            right_min_raise = max(60, shoulder_to_nose_dist * 0.4)
            if (right_shoulder_y - right_wrist_y) > right_min_raise:
                right_hand_down = False
        
        return hand_raised_high and wrist_near_head and right_hand_down
    
    
    def detect_right_hand_up(self, kpts):
        """檢測右手舉高（單手）
        
        用於橫滾控制：右手舉高（手腕接近頭部）→ 機器人向右橫滾
        與指向控制區分：手腕接近頭部 = 舉高，手腕遠離頭部 = 指向
        
        要求：手腕必須明顯高於肩膀（不是平舉），接近頭部
        
        參數：
            kpts: 關鍵點數組 [17, 3]
        
        返回：
            bool: True 表示右手舉高（用於橫滾）
        """
        # 關鍵點索引
        RIGHT_WRIST = 10
        RIGHT_SHOULDER = 6
        LEFT_WRIST = 9
        LEFT_SHOULDER = 5
        NOSE = 0
        
        # 檢查必要關鍵點的置信度
        if (kpts[RIGHT_WRIST][2] < self.min_confidence or 
            kpts[RIGHT_SHOULDER][2] < self.min_confidence or
            kpts[NOSE][2] < self.min_confidence):
            return False
        
        # 獲取關鍵點坐標
        right_wrist_y = kpts[RIGHT_WRIST][1]
        right_shoulder_y = kpts[RIGHT_SHOULDER][1]
        nose_y = kpts[NOSE][1]
        
        # 計算參考距離：肩膀到鼻子的垂直距離（頭頸長度）
        shoulder_to_nose_dist = abs(right_shoulder_y - nose_y)
        
        # 條件 1：右手腕明顯高於右肩（不是平舉）
        # 使用相對閾值：至少高於肩膀 0.4 倍的頭頸長度
        # 或絕對閾值 60 像素（取較大值，避免平舉誤觸發）
        min_raise_dist = max(60, shoulder_to_nose_dist * 0.4)
        hand_raised_high = (right_shoulder_y - right_wrist_y) > min_raise_dist
        
        # 條件 2：右手腕接近頭部高度（在鼻子上下範圍內）
        # 這是與指向控制的關鍵區分：舉手時手腕靠近頭部
        wrist_near_head = abs(right_wrist_y - nose_y) < 100  # 100 像素範圍內
        
        # 條件 3：左手沒有舉起（排除雙手舉高）
        left_hand_down = True
        if kpts[LEFT_WRIST][2] > self.min_confidence and kpts[LEFT_SHOULDER][2] > self.min_confidence:
            left_wrist_y = kpts[LEFT_WRIST][1]
            left_shoulder_y = kpts[LEFT_SHOULDER][1]
            # 使用相同的嚴格閾值檢測左手
            left_min_raise = max(60, shoulder_to_nose_dist * 0.4)
            if (left_shoulder_y - left_wrist_y) > left_min_raise:
                left_hand_down = False
        
        return hand_raised_high and wrist_near_head and left_hand_down
    
    
    def detect_right_hand_pointing(self, kpts):
        """檢測右手指向方向（用於俯仰控制）
        
        方案 2：單手指向控制
        - 👆 右手向上指（手臂伸直）→ 機器人抬頭 (+30°)
        - 👉 右手向前指（水平）→ 機器人水平 (0°)
        - 👇 右手向下指（手臂伸直）→ 機器人低頭 (-30°)
        
        與橫滾控制區分：
        - 右手舉高（手腕接近頭部）→ 橫滾控制（優先級更高）
        - 右手指向（手腕遠離頭部，手臂伸直）→ 俯仰控制
        
        參數：
            kpts: 關鍵點數組 [17, 3]
        
        返回：
            float: 俯仰角度（-30, 0, +30）
        """
        # 關鍵點索引
        RIGHT_WRIST = 10
        RIGHT_SHOULDER = 6
        RIGHT_ELBOW = 8
        NOSE = 0
        
        # 檢查必要關鍵點的置信度
        if (kpts[RIGHT_WRIST][2] < self.min_confidence or 
            kpts[RIGHT_SHOULDER][2] < self.min_confidence):
            return 0.0  # 無法檢測，返回水平
        
        # ===== 關鍵：與橫滾控制區分 =====
        # 如果右手是舉高姿態（手腕接近頭部），優先用於橫滾控制
        # 不應該同時觸發俯仰控制
        if kpts[NOSE][2] > self.min_confidence:
            right_wrist_y = kpts[RIGHT_WRIST][1]
            right_shoulder_y = kpts[RIGHT_SHOULDER][1]
            nose_y = kpts[NOSE][1]
            
            # 檢查是否為舉手姿態（手腕高於肩膀且接近頭部）
            hand_raised = (right_shoulder_y - right_wrist_y) > 30  # 手腕高於肩膀
            wrist_near_head = abs(right_wrist_y - nose_y) < 100   # 手腕接近頭部
            
            if hand_raised and wrist_near_head:
                # 這是舉手姿態，用於橫滾控制，不檢測指向
                return 0.0
        
        # 獲取關鍵點坐標
        right_wrist = kpts[RIGHT_WRIST][:2]
        right_shoulder = kpts[RIGHT_SHOULDER][:2]
        
        # 計算手腕相對於肩膀的向量
        wrist_shoulder_vector = right_wrist - right_shoulder
        
        # 計算角度（使用 atan2，返回值範圍 [-π, π]）
        # atan2(y, x): y 向下為正，y 向上為負（圖像坐標系）
        # x 向右為正
        angle_rad = np.arctan2(wrist_shoulder_vector[1], wrist_shoulder_vector[0])
        angle_deg = np.degrees(angle_rad)
        
        # 檢查手臂是否伸直（可選，提高準確性）
        arm_extended = True
        if kpts[RIGHT_ELBOW][2] > self.min_confidence:
            right_elbow = kpts[RIGHT_ELBOW][:2]
            # 計算肩-肘-腕的角度，接近 180° 表示伸直
            shoulder_to_elbow = right_elbow - right_shoulder
            elbow_to_wrist = right_wrist - right_elbow
            
            # 計算兩個向量的夾角
            dot_product = np.dot(shoulder_to_elbow, elbow_to_wrist)
            norm_product = np.linalg.norm(shoulder_to_elbow) * np.linalg.norm(elbow_to_wrist)
            
            if norm_product > 0:
                cos_angle = dot_product / norm_product
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                elbow_angle = np.degrees(np.arccos(cos_angle))
                
                # 如果手臂彎曲超過 30°，認為不是指向姿態
                if elbow_angle < 150:  # 允許一定彎曲（150° = 30° 彎曲）
                    arm_extended = False
        
        # 如果手臂沒有伸直，返回水平（避免誤檢測）
        if not arm_extended:
            return 0.0
        
        # 根據角度判斷指向方向
        # angle_deg 範圍：
        #   -90° = 向上（12點方向）
        #     0° = 向右（3點方向）
        #   +90° = 向下（6點方向）
        #  ±180° = 向左（9點方向）
        
        if angle_deg < -45:
            # 向上指（-90° ± 45°範圍）
            pitch = -30.0  # 機器人抬頭（負角度）⭐ 修正方向
            self.get_logger().info(
                f"👆 右手向上指！機器人抬頭 ({pitch:.1f}°) [角度: {angle_deg:.1f}°]",
                throttle_duration_sec=1.0
            )
        elif angle_deg > 45:
            # 向下指（+90° ± 45°範圍）
            pitch = 30.0  # 機器人低頭（正角度）⭐ 修正方向
            self.get_logger().info(
                f"👇 右手向下指！機器人低頭 (+{pitch:.1f}°) [角度: {angle_deg:.1f}°]",
                throttle_duration_sec=1.0
            )
        else:
            # 水平指（-45° ~ +45°範圍）或無明確指向動作
            pitch = 0.0  # 機器人水平（默認狀態）
            # 不記錄水平狀態，避免日誌過多
        
        return pitch
    
    
    def analyze_posture(self, keypoints, frame_shape, depth_image=None):
        """分析人體姿態（使用 Depth 圖像提高準確性）
        
        參數：
            keypoints: YOLOv8-Pose 檢測的關鍵點
            frame_shape: 圖像形狀
            depth_image: RealSense 深度圖像（可選）
        
        返回：
            dict: {'height': float, 'roll': float, 'pitch': float}
            或 None（如果關鍵點質量不足）
        """
        # 檢查 keypoints.data 是否為空
        if keypoints.data is None or len(keypoints.data) == 0:
            self.get_logger().warn("關鍵點數據為空")
            return None
        
        # 獲取關鍵點數據 [x, y, confidence]
        kpts = keypoints.data[0].cpu().numpy()  # shape: (17, 3)
        
        # 檢查關鍵點數組形狀
        if kpts.shape[0] < 17:
            self.get_logger().warn(f"關鍵點數量不足：{kpts.shape[0]} < 17")
            return None
        
        # 定義需要的關鍵點索引
        NOSE = 0
        LEFT_SHOULDER = 5
        RIGHT_SHOULDER = 6
        LEFT_HIP = 11
        RIGHT_HIP = 12
        LEFT_ANKLE = 15
        RIGHT_ANKLE = 16
        
        # 檢查關鍵點置信度
        required_points = [LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP]
        point_names = ["左肩", "右肩", "左髖", "右髖"]
        for i, idx in enumerate(required_points):
            conf = kpts[idx][2]
            if conf < self.min_confidence:
                self.get_logger().warn(
                    f"{point_names[i]}置信度不足：{conf:.2f} < {self.min_confidence}"
                )
                return None
        
        # ========== 0. 檢測舉手姿態（投降狀）==========
        # 如果雙手舉高，直接設置為站高姿態
        hands_up = self.detect_hands_up(kpts)
        if hands_up:
            # 只在狀態變化時輸出日誌
            if not self.hands_up_state:
                # 在進入舉手狀態前，記錄當前的正常高度
                self.last_normal_height = self.current_posture['height']
                self.get_logger().info(
                    f"✋ 檢測到舉手姿態！機器人站高 (0.35m) "
                    f"[記錄正常高度: {self.last_normal_height:.2f}m] "
                    f"[當前高度: {self.current_posture['height']:.2f}m]"
                )
                self.hands_up_state = True
            
            return {
                'height': 0.35,  # 站高姿態
                'roll': 0.0,
                'pitch': 0.0
            }
        else:
            # 放下手臂，恢復正常模式
            if self.hands_up_state:
                self.get_logger().info(
                    f"👋 手臂放下，恢復正常高度 "
                    f"(目標: {self.last_normal_height:.2f}m, 當前: {self.current_posture['height']:.2f}m)"
                )
                self.hands_up_state = False
        
        # ========== 1. 計算高度（站立/蹲下）==========
        
        # 先計算肩部中心（橫滾和俯仰計算需要）
        left_shoulder = kpts[LEFT_SHOULDER][:2]
        right_shoulder = kpts[RIGHT_SHOULDER][:2]
        shoulder_center = (left_shoulder + right_shoulder) / 2
        
        # ===== 方法 A：使用 Depth 圖像（優先，更準確）=====
        height = None
        if depth_image is not None and self.camera_intrinsics is not None:
            height = self.calculate_height_from_depth(kpts, depth_image, frame_shape)
            if height is not None:
                # Depth 方法成功，更新最後正常高度
                self.last_normal_height = height
        
        # ===== 方法 B：使用像素距離（備用）=====
        if height is None:
            left_hip = kpts[LEFT_HIP][:2]
            right_hip = kpts[RIGHT_HIP][:2]
            hip_center = (left_hip + right_hip) / 2
            
            # 如果腳踝可見，使用髖部到腳踝距離
            if kpts[LEFT_ANKLE][2] > self.min_confidence or kpts[RIGHT_ANKLE][2] > self.min_confidence:
                ankles = []
                if kpts[LEFT_ANKLE][2] > self.min_confidence:
                    ankles.append(kpts[LEFT_ANKLE][:2])
                if kpts[RIGHT_ANKLE][2] > self.min_confidence:
                    ankles.append(kpts[RIGHT_ANKLE][:2])
                ankle_center = np.mean(ankles, axis=0)
                
                # 計算髖部到腳踝的垂直距離（像素）
                body_height_px = abs(hip_center[1] - ankle_center[1])
            else:
                # 如果腳踝不可見，使用肩部到髖部距離的估算
                torso_height_px = abs(shoulder_center[1] - hip_center[1])
                body_height_px = torso_height_px * 2.0  # 估算全身高度
            
            # 歸一化（相對於圖像高度）
            normalized_height = body_height_px / frame_shape[0]
            
            # 映射到機器人高度
            # normalized_height 範圍約 0.3-0.6（蹲下到站立）
            # 線性映射到機器人高度範圍
            if normalized_height < 0.35:
                # 蹲下
                height = self.height_min + (normalized_height - 0.25) * (self.height_squat - self.height_min) / 0.1
            elif normalized_height > 0.50:
                # 站立/站高
                height = self.height_stand + (normalized_height - 0.50) * (self.height_max - self.height_stand) / 0.15
            else:
                # 中間範圍
                height = self.height_squat + (normalized_height - 0.35) * (self.height_stand - self.height_squat) / 0.15
            
            # 限制範圍
            height = np.clip(height, self.height_min, self.height_max)
            
            # 更新最後正常高度（像素方法）
            self.last_normal_height = height
        
        # 如果兩種方法都失敗，使用默認高度
        if height is None:
            height = self.height_stand
        
        # ========== 2. 計算橫滾（左右傾斜）==========
        # v1.4.5.1：橫滾使用雙手控制（與指向控制區分）
        # 區分原則：手腕接近頭部 = 舉手（橫滾），手腕遠離頭部 = 指向（俯仰）
        roll_deg = 0.0
        
        # 檢測雙手舉高
        left_hand_up = self.detect_left_hand_up(kpts)
        right_hand_up = self.detect_right_hand_up(kpts)
        
        if left_hand_up:
            # 左手舉高 → 機器人向左橫滾（負值）
            roll_deg = -self.roll_max
            self.get_logger().info(f"👈 偵測到左手舉高！機器人向左橫滾 ({roll_deg:.1f}°)", 
                                   throttle_duration_sec=1.0)
        elif right_hand_up:
            # 右手舉高 → 機器人向右橫滾（正值）
            roll_deg = self.roll_max
            self.get_logger().info(f"👉 偵測到右手舉高！機器人向右橫滾 (+{roll_deg:.1f}°)", 
                                   throttle_duration_sec=1.0)
        else:
            # 都不舉 → 橫滾角為 0（正常）
            roll_deg = 0.0
        
        # ========== 3. 計算俯仰（抬頭/低頭）==========
        # v1.4.5：方案 2 - 單手指向控制
        # 使用右手指向方向控制俯仰角（-30°, 0°, +30°）
        pitch_deg = self.detect_right_hand_pointing(kpts)
        
        return {
            'height': height,
            'roll': roll_deg,
            'pitch': pitch_deg
        }
    
    
    def calculate_height_from_depth(self, kpts, depth_image, frame_shape):
        """使用 Depth 圖像計算人體實際高度
        
        方法：測量鼻子（頭部）或髖部的實際 3D 位置，計算離地高度
        
        參數：
            kpts: 關鍵點數組 [17, 3]
            depth_image: 深度圖像（毫米）
            frame_shape: 圖像形狀
        
        返回：
            機器人高度（米）或 None
        """
        try:
            NOSE = 0
            LEFT_HIP = 11
            RIGHT_HIP = 12
            
            # 嘗試方法 1：使用鼻子（頭部）
            if kpts[NOSE][2] > self.min_confidence:
                nose_x, nose_y = int(kpts[NOSE][0]), int(kpts[NOSE][1])
                
                # 確保在圖像範圍內
                if 0 <= nose_x < depth_image.shape[1] and 0 <= nose_y < depth_image.shape[0]:
                    # 獲取深度值（毫米 → 米）
                    depth_mm = depth_image[nose_y, nose_x]
                    
                    if depth_mm > 0:  # 有效深度
                        depth_m = depth_mm / 1000.0
                        
                        # 計算頭部在相機坐標系中的 3D 位置
                        # 考慮相機向上傾斜 15 度
                        fx = self.camera_intrinsics['fx']
                        fy = self.camera_intrinsics['fy']
                        cx = self.camera_intrinsics['cx']
                        cy = self.camera_intrinsics['cy']
                        
                        # 像素坐標轉相機坐標
                        x_cam = (nose_x - cx) * depth_m / fx
                        y_cam = (nose_y - cy) * depth_m / fy
                        z_cam = depth_m
                        
                        # 考慮相機傾斜角度（向上 15 度）
                        # 旋轉矩陣修正
                        y_world = y_cam * math.cos(self.camera_pitch) - z_cam * math.sin(self.camera_pitch)
                        z_world = y_cam * math.sin(self.camera_pitch) + z_cam * math.cos(self.camera_pitch)
                        
                        # 頭部實際離地高度（米）
                        # y_world 是向下為正，所以要反轉
                        head_height = self.camera_height_from_ground - y_world
                        
                        # 調試輸出
                        self.get_logger().info(
                            f"[HEAD] pixel=({nose_x},{nose_y}), depth={depth_m:.2f}m, "
                            f"y_cam={y_cam:.3f}, y_world={y_world:.3f}, head={head_height:.3f}m"
                        )
                        
                        # 映射到機器人高度
                        # 相機高度 0.3m，實測站立時頭部約 0.7-1.1m
                        # 擴大映射範圍，讓踮腳尖能達到站高
                        
                        if head_height > 0.95:
                            # 站立/站高（頭部高於 0.95m）
                            # 0.95m → 0.25m, 1.2m → 0.36m
                            ratio = (head_height - 0.95) / 0.25
                            robot_height = 0.25 + ratio * (self.height_max - 0.25)
                            robot_height = min(robot_height, self.height_max)
                        elif head_height < 0.55:
                            # 深蹲（頭部低於 0.55m）
                            robot_height = self.height_min + (head_height - 0.4) / 0.15 * (self.height_squat - self.height_min)
                        else:
                            # 線性插值（0.55m - 0.95m）
                            ratio = (head_height - 0.55) / (0.95 - 0.55)
                            robot_height = self.height_squat + ratio * (0.25 - self.height_squat)
                        
                        # 限制範圍
                        robot_height = np.clip(robot_height, self.height_min, self.height_max)
                        
                        # 調試輸出
                        self.get_logger().info(
                            f"[MAP_HEAD] head={head_height:.3f}m → robot={robot_height:.3f}m"
                        )
                        
                        return robot_height
            
            # 嘗試方法 2：使用髖部中心
            if kpts[LEFT_HIP][2] > self.min_confidence and kpts[RIGHT_HIP][2] > self.min_confidence:
                left_hip = kpts[LEFT_HIP][:2]
                right_hip = kpts[RIGHT_HIP][:2]
                hip_center = ((left_hip + right_hip) / 2).astype(int)
                hip_x, hip_y = hip_center[0], hip_center[1]
                
                # 確保在圖像範圍內
                if 0 <= hip_x < depth_image.shape[1] and 0 <= hip_y < depth_image.shape[0]:
                    depth_mm = depth_image[hip_y, hip_x]
                    
                    if depth_mm > 0:
                        depth_m = depth_mm / 1000.0
                        
                        # 計算髖部 3D 位置
                        fx = self.camera_intrinsics['fx']
                        fy = self.camera_intrinsics['fy']
                        cx = self.camera_intrinsics['cx']
                        cy = self.camera_intrinsics['cy']
                        
                        x_cam = (hip_x - cx) * depth_m / fx
                        y_cam = (hip_y - cy) * depth_m / fy
                        z_cam = depth_m
                        
                        # 考慮相機傾斜
                        y_world = y_cam * math.cos(self.camera_pitch) - z_cam * math.sin(self.camera_pitch)
                        
                        hip_height = self.camera_height_from_ground - y_world
                        
                        # 調試輸出
                        self.get_logger().info(
                            f"[HIP] pixel=({hip_x},{hip_y}), depth={depth_m:.2f}m, "
                            f"y_cam={y_cam:.3f}, y_world={y_world:.3f}, hip={hip_height:.3f}m"
                        )
                        
                        # 映射到機器人高度
                        # 相機高度 0.3m，向上 15°
                        # 站立時髖部約 0.7-0.8m → robot 0.30-0.36m
                        # 蹲下時髖部約 0.2-0.3m → robot 0.16m
                        
                        if hip_height > 0.65:
                            # 站立（髖部高於 0.65m）
                            robot_height = self.height_stand + (hip_height - 0.65) / 0.15 * (self.height_max - self.height_stand)
                        elif hip_height < 0.35:
                            # 深蹲（髖部低於 0.35m）
                            robot_height = self.height_min + (hip_height - 0.15) / 0.2 * (self.height_squat - self.height_min)
                        else:
                            # 線性插值（0.35m - 0.65m）
                            ratio = (hip_height - 0.35) / (0.65 - 0.35)
                            robot_height = self.height_squat + ratio * (self.height_stand - self.height_squat)
                        
                        robot_height = np.clip(robot_height, self.height_min, self.height_max)
                        
                        # 調試輸出
                        self.get_logger().info(
                            f"[MAP_HIP] hip={hip_height:.3f}m → robot={robot_height:.3f}m"
                        )
                        
                        return robot_height
            
            # 兩種方法都失敗
            return None
            
        except Exception as e:
            self.get_logger().warn(f"Depth 高度計算失敗: {e}")
            return None
    
    
    def update_posture_smoothed(self, posture):
        """使用滑動平均更新姿態（平滑濾波 + 死區 + 速率限制）"""
        # ========== 1. 添加到緩衝區（滑動平均）==========
        # 高度和俯仰使用平滑濾波
        self.height_buffer.append(posture['height'])
        self.pitch_buffer.append(posture['pitch'])
        
        # 橫滾不使用平滑濾波（手勢控制需要立即響應）
        # self.roll_buffer.append(posture['roll'])  # 不再添加
        
        # 計算平滑後的值
        smoothed_height = np.mean(self.height_buffer)
        smoothed_pitch = np.mean(self.pitch_buffer)
        # smoothed_roll 不再需要，橫滾直接使用 posture['roll']
        
        # ========== 2. 應用死區（Deadzone）==========
        # 只有當變化超過閾值時才更新
        
        # 高度死區
        if self.enable_height:
            height_diff = abs(smoothed_height - self.current_posture['height'])
            if height_diff > self.height_deadzone:
                # 超過死區，應用速率限制
                max_change = self.max_height_change
                height_change = np.clip(
                    smoothed_height - self.current_posture['height'],
                    -max_change,
                    max_change
                )
                self.current_posture['height'] += height_change
        
        # 橫滾控制（手勢控制，直接設置）
        if self.enable_roll:
            # 手勢控制的橫滾應該立即響應，不使用平滑濾波和速率限制
            # 因為手勢檢測本身已經是離散的（-20°, 0°, +20°）
            # 直接設置目標值，實現"一次到位"的效果
            self.current_posture['roll'] = posture['roll']
        else:
            # 橫滾禁用時保持 0
            self.current_posture['roll'] = 0.0
        
        # 俯仰死區（暫時禁用）
        if self.enable_pitch:
            pitch_diff = abs(smoothed_pitch - self.current_posture['pitch'])
            if pitch_diff > self.pitch_deadzone:
                # 超過死區，應用速率限制
                max_change = self.max_pitch_change
                pitch_change = np.clip(
                    smoothed_pitch - self.current_posture['pitch'],
                    -max_change,
                    max_change
                )
                self.current_posture['pitch'] += pitch_change
        else:
            # 俯仰禁用時保持 0
            self.current_posture['pitch'] = 0.0
    
    
    def reset_to_default_posture(self):
        """重置到默認姿態（站立）"""
        self.current_posture['height'] = self.height_stand
        self.current_posture['roll'] = 0.0
        self.current_posture['pitch'] = 0.0
        
        # 清空緩衝區
        self.height_buffer.clear()
        self.roll_buffer.clear()
        self.pitch_buffer.clear()
    
    
    def publish_posture_command(self):
        """以固定頻率發布姿態命令"""
        if not self.enable_mimic:
            return
        
        # 創建 JointState 消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_height', 'joint_roll', 'joint_pitching']
        msg.position = [
            float(self.current_posture['height']),
            float(self.current_posture['roll']),
            float(self.current_posture['pitch'])
        ]
        
        self.posture_pub.publish(msg)
        
        # 發布狀態信息
        status_msg = String()
        status_msg.data = (
            f"Height: {self.current_posture['height']:.2f}m, "
            f"Roll: {self.current_posture['roll']:.1f}°, "
            f"Pitch: {self.current_posture['pitch']:.1f}°"
        )
        self.status_pub.publish(status_msg)
    
    
    def draw_skeleton(self, frame, keypoints):
        """繪製人體骨架"""
        # 檢查 keypoints.data 是否為空
        if keypoints.data is None or len(keypoints.data) == 0:
            return
        
        kpts = keypoints.data[0].cpu().numpy()
        
        # 檢查關鍵點數組形狀
        if kpts.shape[0] < 17:
            return
        
        # 定義骨架連接（OpenPose 格式）
        skeleton = [
            [0, 1], [0, 2],  # 頭部
            [1, 3], [2, 4],  # 耳朵
            [0, 5], [0, 6],  # 肩膀
            [5, 6],          # 肩部連線
            [5, 7], [7, 9],  # 左臂
            [6, 8], [8, 10], # 右臂
            [5, 11], [6, 12],# 軀幹
            [11, 12],        # 髖部連線
            [11, 13], [13, 15], # 左腿
            [12, 14], [14, 16]  # 右腿
        ]
        
        # 繪製骨架線
        for connection in skeleton:
            pt1_idx, pt2_idx = connection
            if kpts[pt1_idx][2] > self.min_confidence and kpts[pt2_idx][2] > self.min_confidence:
                pt1 = tuple(kpts[pt1_idx][:2].astype(int))
                pt2 = tuple(kpts[pt2_idx][:2].astype(int))
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
        
        # 繪製關鍵點
        for i, kpt in enumerate(kpts):
            if kpt[2] > self.min_confidence:
                x, y = int(kpt[0]), int(kpt[1])
                cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)
    
    
    def draw_posture_info(self, frame, person_detected):
        """繪製姿態信息"""
        h, w = frame.shape[:2]
        
        # 背景框
        cv2.rectangle(frame, (10, 10), (350, 160), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (350, 160), (255, 255, 255), 2)
        
        # 標題
        cv2.putText(frame, "Posture Mimic", (20, 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # 狀態
        status = "ACTIVE" if person_detected else "WAITING"
        color = (0, 255, 0) if person_detected else (0, 165, 255)
        cv2.putText(frame, f"Status: {status}", (20, 65),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # 姿態參數
        cv2.putText(frame, f"Height: {self.current_posture['height']:.2f}m",
                   (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Roll:   {self.current_posture['roll']:+.1f} deg",
                   (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Pitch:  {self.current_posture['pitch']:+.1f} deg",
                   (20, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # FPS
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (w - 120, 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    
    def update_fps(self):
        """計算 FPS"""
        self.frame_count += 1
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_fps_time).nanoseconds / 1e9
        
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_fps_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = PostureMimicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("🛑 姿態模仿節點已停止")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

