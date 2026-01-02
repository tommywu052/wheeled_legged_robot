#!/usr/bin/env python3
"""
RealSense人體追蹤節點 - YOLOv8版本
使用YOLOv8進行高準確度人體偵測，支援Jetson GPU加速

訂閱話題：
  - /camera/camera/color/image_raw (RGB影像)
  - /camera/camera/aligned_depth_to_color/image_raw (對齊的深度影像)

發佈話題：
  - /cmd_vel (控制機器人運動)
  - /people_follower/debug_image (除錯視覺化影像)
  - /people_follower/status (追蹤狀態)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

# YOLOv8相關導入
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️  YOLOv8未安裝，請執行: pip3 install ultralytics")


class PeopleFollowerYOLO(Node):
    """基於YOLOv8的人體追蹤節點"""
    
    def __init__(self):
        super().__init__('people_follower_yolo')
        
        if not YOLO_AVAILABLE:
            self.get_logger().error('❌ YOLOv8未安裝！請執行: pip3 install ultralytics')
            return
        
        # ========== 參數宣告 ==========
        self.declare_parameter('yolo_model', 'yolov8n.pt')  # yolov8n(快) / yolov8s(準)
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('min_distance', 0.5)
        self.declare_parameter('max_distance', 3.0)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 2.0)
        self.declare_parameter('enable_follower', True)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('confidence_threshold', 0.5)  # YOLO信心閾值
        
        # 獲取參數
        yolo_model = self.get_parameter('yolo_model').value
        self.target_distance = self.get_parameter('target_distance').value
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.enable_follower = self.get_parameter('enable_follower').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # ========== 初始化變數 ==========
        self.bridge = CvBridge()
        self.latest_color_image = None
        self.latest_depth_image = None
        self.camera_info = None
        self.image_width = 640
        self.image_height = 480
        self.last_detection_time = time.time()
        self.no_person_timeout = 2.0
        
        # ========== 初始化YOLOv8模型 ==========
        self.get_logger().info(f'📦 載入YOLO模型: {yolo_model}')
        try:
            self.model = YOLO(yolo_model)
            # Jetson優化：使用FP16
            self.model.fuse()  # 融合層以提升速度
            self.get_logger().info('✅ YOLOv8模型已載入')
            self.get_logger().info(f'   使用裝置: {"CUDA" if self.model.device.type == "cuda" else "CPU"}')
        except Exception as e:
            self.get_logger().error(f'❌ YOLO模型載入失敗: {e}')
            return
        
        # ========== QoS設置 ==========
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # ========== 訂閱者 ==========
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            qos_profile
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            qos_profile
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # ========== 發佈者 ==========
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_image_pub = self.create_publisher(Image, '/people_follower/debug_image', 10)
        self.status_pub = self.create_publisher(String, '/people_follower/status', 10)
        
        # ========== 定時器 ==========
        self.timer = self.create_timer(0.1, self.process_callback)  # 10Hz
        
        self.get_logger().info('🚀 YOLOv8人體追蹤節點已啟動')
        self.get_logger().info(f'   模型: {yolo_model}')
        self.get_logger().info(f'   目標距離: {self.target_distance}m')
        self.get_logger().info(f'   追蹤啟用: {self.enable_follower}')
    
    def color_callback(self, msg):
        """RGB影像回調"""
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_height, self.image_width = self.latest_color_image.shape[:2]
        except Exception as e:
            self.get_logger().error(f'RGB影像轉換失敗: {e}')
    
    def depth_callback(self, msg):
        """深度影像回調"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'深度影像轉換失敗: {e}')
    
    def camera_info_callback(self, msg):
        """相機參數回調"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('✅ 相機參數已接收')
    
    def detect_people_yolo(self, image):
        """使用YOLOv8偵測人體"""
        # YOLO推理
        results = self.model(image, verbose=False, conf=self.confidence_threshold)
        
        detections = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # 只保留person類別 (class_id = 0 in COCO)
                if int(box.cls) == 0:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = float(box.conf)
                    
                    detections.append({
                        'bbox': (int(x1), int(y1), int(x2-x1), int(y2-y1)),
                        'confidence': confidence,
                        'center_x': int((x1 + x2) / 2),
                        'center_y': int((y1 + y2) / 2)
                    })
        
        return detections
    
    def get_depth_at_point(self, x, y, depth_image, window_size=9):
        """獲取指定點的深度值（改進版：更大窗口，更穩定）"""
        if depth_image is None:
            return None
        
        h, w = depth_image.shape
        
        # 確保座標在有效範圍內
        x = int(np.clip(x, 0, w - 1))
        y = int(np.clip(y, 0, h - 1))
        
        # 使用更大的窗口（9x9）提高穩定性
        half_window = window_size // 2
        y_min = max(0, y - half_window)
        y_max = min(h, y + half_window + 1)
        x_min = max(0, x - half_window)
        x_max = min(w, x + half_window + 1)
        
        # 提取窗口
        window = depth_image[y_min:y_max, x_min:x_max]
        
        # 過濾無效值（0表示無深度）
        valid_depths = window[window > 0]
        
        # 如果有效深度太少，擴大搜索範圍
        if len(valid_depths) < 3:
            # 嘗試更大的窗口（15x15）
            larger_half = 7
            y_min = max(0, y - larger_half)
            y_max = min(h, y + larger_half + 1)
            x_min = max(0, x - larger_half)
            x_max = min(w, x + larger_half + 1)
            
            window = depth_image[y_min:y_max, x_min:x_max]
            valid_depths = window[window > 0]
        
        if len(valid_depths) == 0:
            return None
        
        # 使用中位數（比平均值更穩定）
        return np.median(valid_depths) / 1000.0
    
    def select_target_person(self, detections, depth_image):
        """選擇目標人物（改進版：更好的邊緣處理）"""
        if len(detections) == 0:
            return None
        
        valid_targets = []
        
        for i, detection in enumerate(detections):
            center_x = detection['center_x']
            center_y = detection['center_y']
            bbox = detection['bbox']
            
            # 檢查檢測框是否在畫面內
            x, y, w, h = bbox
            if x < 0 or y < 0 or (x + w) > self.image_width or (y + h) > self.image_height:
                # 邊界框部分超出，調整中心點到框內有效區域
                center_x = int(np.clip(center_x, 10, self.image_width - 10))
                center_y = int(np.clip(center_y, 10, self.image_height - 10))
                self.get_logger().debug(f'檢測框{i}部分超出畫面，調整中心點: ({center_x}, {center_y})')
            
            # 獲取深度
            depth = self.get_depth_at_point(center_x, center_y, depth_image)
            
            if depth is None or depth <= 0:
                self.get_logger().debug(f'檢測框{i}無法獲取有效深度')
                continue
            
            # 過濾超出範圍的偵測（只過濾太遠的，太近的仍要追蹤以便後退）
            if depth > self.max_distance:
                self.get_logger().debug(f'檢測框{i}距離太遠: {depth:.2f}m > {self.max_distance}m')
                continue
            
            # 太近時發出警告但仍然追蹤（這樣才能後退）
            if depth < self.min_distance:
                self.get_logger().warn(f'⚠️ 目標非常接近: {depth:.2f}m，機器人應後退')
            
            # 計算與影像中心的距離
            center_offset = abs(center_x - self.image_width / 2)
            
            valid_targets.append({
                'detection': detection,
                'depth': depth,
                'center_offset': center_offset
            })
            
            self.get_logger().debug(f'有效目標{i}: 深度={depth:.2f}m, 偏移={center_offset:.0f}px')
        
        if len(valid_targets) == 0:
            self.get_logger().debug(f'檢測到{len(detections)}人，但無有效目標')
            return None
        
        # 選擇最近的目標
        valid_targets.sort(key=lambda x: (x['depth'], x['center_offset']))
        
        selected = valid_targets[0]
        self.get_logger().debug(f'選擇最近目標: 深度={selected["depth"]:.2f}m')
        
        return selected
    
    def calculate_control_command(self, target):
        """計算控制命令（三級自適應增益系統）"""
        cmd = Twist()
        
        if target is None:
            return cmd
        
        detection = target['detection']
        depth = target['depth']
        center_x = detection['center_x']
        
        # ========== 三級自適應增益系統 ==========
        # 根據距離調整增益和速度限制
        
        # 定義距離區間
        EMERGENCY_THRESHOLD = self.min_distance  # 0.4m - 緊急區域
        NEAR_THRESHOLD = 0.7                      # 0.7m - 近距離閾值
        
        if depth < EMERGENCY_THRESHOLD:
            # 🚨 緊急後退模式 (<0.4m)：快速後退，允許更高速度
            adaptive_linear_gain = 0.6        # 高增益快速後退
            adaptive_angular_gain = 1.2       # 超高轉向增益
            max_linear = 0.8                  # 提高後退速度限制（0.5→0.8）
            max_angular = 0.6                 # 提高轉向速度限制
            mode = "🚨緊急後退"
            
        elif depth <= NEAR_THRESHOLD:
            # 近距離模式 (0.4-0.7m)：慢速前後，快速轉向
            adaptive_linear_gain = 0.25
            adaptive_angular_gain = 0.98
            max_linear = self.max_linear_speed   # 0.3
            max_angular = 0.7                    # 提高到 0.7 rad/s（更靈活的轉向）
            mode = "近距離"
            
        else:
            # 遠距離模式 (>0.7m)：快速接近
            adaptive_linear_gain = self.linear_gain   # 0.35
            adaptive_angular_gain = self.angular_gain  # 0.85
            max_linear = 0.5                  # 提高速度限制（0.3→0.5）讓機器人能跟上
            max_angular = self.max_angular_speed
            mode = "遠距離"
        
        # 記錄模式切換（僅在改變時）
        if not hasattr(self, '_last_mode') or self._last_mode != mode:
            self.get_logger().info(
                f'🔄 切換到{mode}模式：linear={adaptive_linear_gain}, angular={adaptive_angular_gain}, '
                f'max_linear={max_linear:.1f}, max_angular={max_angular:.1f}'
            )
            self._last_mode = mode
        
        # ========== 線速度控制（前後移動）==========
        distance_error = depth - self.target_distance
        linear_velocity = adaptive_linear_gain * distance_error
        
        # 使用動態速度限制
        linear_velocity = np.clip(linear_velocity, -max_linear, max_linear)
        
        # 死區：±5cm 內不動（但緊急模式下縮小死區）
        deadzone = 0.02 if depth < EMERGENCY_THRESHOLD else 0.05
        if abs(distance_error) < deadzone:
            linear_velocity = 0.0
        
        # ========== 角速度控制（左右轉向）==========
        center_offset = (center_x - self.image_width / 2) / (self.image_width / 2)
        angular_velocity = -adaptive_angular_gain * center_offset
        
        # 使用動態速度限制
        angular_velocity = np.clip(angular_velocity, -max_angular, max_angular)
        
        # 死區：±10% 畫面寬度內不轉（避免擺動）
        if abs(center_offset) < 0.1:
            angular_velocity = 0.0
        
        # ========== 輸出控制命令 ==========
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        
        # 調試信息（緊急模式下強制顯示）
        if depth < EMERGENCY_THRESHOLD:
            self.get_logger().warn(
                f'{mode}: 距離={depth:.2f}m❗ 誤差={distance_error:.2f}m, '
                f'後退速度={linear_velocity:.3f} m/s, 角速度={angular_velocity:.2f}'
            )
        else:
            self.get_logger().debug(
                f'{mode}: 距離={depth:.2f}m, 誤差={distance_error:.2f}m, '
                f'線速度={linear_velocity:.2f}, 角速度={angular_velocity:.2f}'
            )
        
        return cmd
    
    def draw_debug_image(self, image, detections, target):
        """繪製除錯影像（改進版：顯示所有人的距離）"""
        debug_image = image.copy()
        
        # 先獲取所有檢測結果的深度信息
        detection_depths = {}
        for i, detection in enumerate(detections):
            center_x = detection['center_x']
            center_y = detection['center_y']
            depth = self.get_depth_at_point(center_x, center_y, self.latest_depth_image)
            detection_depths[i] = depth
        
        # 繪製所有偵測結果（非目標）
        target_detection = target['detection'] if target is not None else None
        
        for i, detection in enumerate(detections):
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            depth = detection_depths[i]
            
            # 判斷是否為目標
            is_target = (target_detection is not None and 
                        detection['center_x'] == target_detection['center_x'] and
                        detection['center_y'] == target_detection['center_y'])
            
            if not is_target:
                # 其他人：灰色框 + 顯示距離
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (128, 128, 128), 2)
                
                # 顯示信心分數和距離
                label = f'Person {confidence:.2f}'
                if depth is not None and depth > 0:
                    label += f' | {depth:.2f}m'
                
                cv2.putText(debug_image, label, (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 2)
        
        # 繪製選中的目標（最近的人）
        if target is not None:
            detection = target['detection']
            depth = target['depth']
            x, y, w, h = detection['bbox']
            center_x = detection['center_x']
            center_y = detection['center_y']
            confidence = detection['confidence']
            
            # 綠色粗框標示目標
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
            
            # 顯示「目標」標籤和距離
            label = f'TARGET {confidence:.2f} | {depth:.2f}m'
            cv2.putText(debug_image, label, (x, y - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # 標示「最近」
            cv2.putText(debug_image, 'CLOSEST', (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # 標示目標中心點
            cv2.circle(debug_image, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # 繪製指向目標的箭頭（從畫面中心）
            cv2.arrowedLine(debug_image, 
                          (self.image_width // 2, self.image_height - 50),
                          (center_x, center_y),
                          (0, 255, 0), 2, tipLength=0.2)
        
        # 繪製影像中心線
        cv2.line(debug_image, (self.image_width // 2, 0),
                (self.image_width // 2, self.image_height), (255, 0, 0), 2)
        
        # 顯示狀態和統計信息
        status_text = 'TRACKING (YOLO)' if target is not None else 'SEARCHING (YOLO)'
        color = (0, 255, 0) if target is not None else (0, 0, 255)
        cv2.putText(debug_image, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        
        # 顯示偵測到的人數
        people_count = len(detections)
        cv2.putText(debug_image, f'Detected: {people_count} people', (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 如果有目標，顯示目標距離（大字）
        if target is not None:
            distance_text = f'Distance: {target["depth"]:.2f}m'
            cv2.putText(debug_image, distance_text, (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        return debug_image
    
    def process_callback(self):
        """主處理回調"""
        if self.latest_color_image is None or self.latest_depth_image is None:
            return
        
        try:
            # YOLOv8人體偵測
            detections = self.detect_people_yolo(self.latest_color_image)
            
            # 選擇目標
            target = self.select_target_person(detections, self.latest_depth_image)
            
            # 計算控制命令
            if target is not None:
                self.last_detection_time = time.time()
                
                if self.enable_follower:
                    cmd = self.calculate_control_command(target)
                    self.cmd_vel_pub.publish(cmd)
                    
                    status_msg = String()
                    status_msg.data = f'TRACKING: {target["depth"]:.2f}m'
                    self.status_pub.publish(status_msg)
                    
                    self.get_logger().info(
                        f'🎯 追蹤目標 | 距離: {target["depth"]:.2f}m | '
                        f'線速度: {cmd.linear.x:.2f} | 角速度: {cmd.angular.z:.2f}',
                        throttle_duration_sec=1.0
                    )
            else:
                time_since_detection = time.time() - self.last_detection_time
                
                if time_since_detection > self.no_person_timeout:
                    cmd = Twist()
                    self.cmd_vel_pub.publish(cmd)
                    
                    status_msg = String()
                    status_msg.data = 'SEARCHING'
                    self.status_pub.publish(status_msg)
                    
                    self.get_logger().info('🔍 未偵測到目標，機器人已停止',
                                         throttle_duration_sec=2.0)
            
            # 發佈除錯影像
            if self.publish_debug_image:
                debug_image = self.draw_debug_image(
                    self.latest_color_image, detections, target
                )
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                self.debug_image_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'處理錯誤: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PeopleFollowerYOLO()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.get_logger().info('🛑 YOLOv8人體追蹤節點已停止')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

