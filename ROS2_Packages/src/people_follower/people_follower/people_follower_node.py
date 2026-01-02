#!/usr/bin/env python3
"""
RealSense人體追蹤節點
功能：使用RealSense相機偵測人並控制機器人追蹤

訂閱話題：
  - /camera/color/image_raw (RGB影像)
  - /camera/aligned_depth_to_color/image_raw (對齊的深度影像)
  - /camera/color/camera_info (相機參數)

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


class PeopleFollowerNode(Node):
    """人體追蹤節點"""
    
    def __init__(self):
        super().__init__('people_follower_node')
        
        # ========== 參數宣告 ==========
        self.declare_parameter('detection_method', 'hog')  # 'hog', 'dnn', 'mediapipe'
        self.declare_parameter('target_distance', 1.0)     # 目標追蹤距離(公尺)
        self.declare_parameter('min_distance', 0.5)        # 最小安全距離(公尺)
        self.declare_parameter('max_distance', 3.0)        # 最大偵測距離(公尺)
        self.declare_parameter('max_linear_speed', 0.3)    # 最大前進速度(m/s)
        self.declare_parameter('max_angular_speed', 0.8)   # 最大旋轉速度(rad/s)
        self.declare_parameter('linear_gain', 0.5)         # 線速度增益
        self.declare_parameter('angular_gain', 2.0)        # 角速度增益
        self.declare_parameter('enable_follower', True)    # 是否啟用追蹤
        self.declare_parameter('publish_debug_image', True) # 是否發佈除錯影像
        
        # 獲取參數
        self.detection_method = self.get_parameter('detection_method').value
        self.target_distance = self.get_parameter('target_distance').value
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.enable_follower = self.get_parameter('enable_follower').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        
        # ========== 初始化變數 ==========
        self.bridge = CvBridge()
        self.latest_color_image = None
        self.latest_depth_image = None
        self.camera_info = None
        self.image_width = 640
        self.image_height = 480
        self.last_detection_time = time.time()
        self.no_person_timeout = 2.0  # 未偵測到人的超時時間(秒)
        
        # ========== 初始化人體偵測器 ==========
        self.init_detector()
        
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
        
        self.get_logger().info('🚀 人體追蹤節點已啟動')
        self.get_logger().info(f'   偵測方法: {self.detection_method}')
        self.get_logger().info(f'   目標距離: {self.target_distance}m')
        self.get_logger().info(f'   追蹤啟用: {self.enable_follower}')
    
    def init_detector(self):
        """初始化人體偵測器"""
        if self.detection_method == 'hog':
            # HOG偵測器（OpenCV內建，簡單快速）
            self.hog_detector = cv2.HOGDescriptor()
            self.hog_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.get_logger().info('✅ HOG人體偵測器已初始化')
            
        elif self.detection_method == 'dnn':
            # DNN偵測器（MobileNet-SSD）
            try:
                # 使用COCO預訓練的MobileNet-SSD模型
                # 需要下載模型檔案（如果沒有，會使用HOG作為後備）
                model_path = '/home/robotester1/legged_robot/models/MobileNetSSD_deploy.caffemodel'
                config_path = '/home/robotester1/legged_robot/models/MobileNetSSD_deploy.prototxt'
                
                self.net = cv2.dnn.readNetFromCaffe(config_path, model_path)
                self.get_logger().info('✅ DNN人體偵測器已初始化')
            except Exception as e:
                self.get_logger().warn(f'⚠️ DNN模型載入失敗，切換到HOG: {e}')
                self.detection_method = 'hog'
                self.init_detector()
        else:
            self.get_logger().warn(f'⚠️ 未知偵測方法 {self.detection_method}，使用HOG')
            self.detection_method = 'hog'
            self.init_detector()
    
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
            # RealSense深度影像通常是16位元，單位為毫米
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'深度影像轉換失敗: {e}')
    
    def camera_info_callback(self, msg):
        """相機參數回調"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('✅ 相機參數已接收')
    
    def detect_people_hog(self, image):
        """使用HOG偵測人體"""
        # 影像預處理：直方圖均衡化改善對比度
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
        # HOG偵測器參數（調整以減少誤檢）
        # 注意：OpenCV 4.10+ 已移除 finalThreshold 參數
        detections, weights = self.hog_detector.detectMultiScale(
            gray_bgr,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
            useMeanshiftGrouping=False
        )
        
        results = []
        for i, (x, y, w, h) in enumerate(detections):
            # 置信度
            try:
                confidence = float(weights[i]) if len(weights) > 0 and i < len(weights) else 1.0
            except:
                confidence = 1.0
            
            # 多重過濾條件減少誤檢
            aspect_ratio = h / w if w > 0 else 0
            is_reasonable_size = (h > 100) and (w > 40)
            is_person_like = (1.5 <= aspect_ratio <= 4.0)
            
            # 只保留高置信度且符合人體特徵的偵測
            if confidence > 1.0 and is_reasonable_size and is_person_like:
                results.append({
                    'bbox': (x, y, w, h),
                    'confidence': confidence,
                    'center_x': x + w // 2,
                    'center_y': y + h // 2
                })
        
        return results
    
    def detect_people_dnn(self, image):
        """使用DNN偵測人體（MobileNet-SSD）"""
        blob = cv2.dnn.blobFromImage(
            cv2.resize(image, (300, 300)),
            0.007843,
            (300, 300),
            127.5
        )
        
        self.net.setInput(blob)
        detections = self.net.forward()
        
        results = []
        h, w = image.shape[:2]
        
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            class_id = int(detections[0, 0, i, 1])
            
            # Class 15 是人類（COCO資料集）
            if confidence > 0.5 and class_id == 15:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (x1, y1, x2, y2) = box.astype('int')
                
                results.append({
                    'bbox': (x1, y1, x2 - x1, y2 - y1),
                    'confidence': float(confidence),
                    'center_x': (x1 + x2) // 2,
                    'center_y': (y1 + y2) // 2
                })
        
        return results
    
    def get_depth_at_point(self, x, y, depth_image, window_size=5):
        """獲取指定點的深度值（使用周圍視窗的中值以提高穩健性）"""
        if depth_image is None:
            return None
        
        h, w = depth_image.shape
        x = np.clip(x, 0, w - 1)
        y = np.clip(y, 0, h - 1)
        
        # 獲取視窗區域
        half_window = window_size // 2
        y_min = max(0, y - half_window)
        y_max = min(h, y + half_window + 1)
        x_min = max(0, x - half_window)
        x_max = min(w, x + half_window + 1)
        
        window = depth_image[y_min:y_max, x_min:x_max]
        
        # 過濾無效深度值（0）
        valid_depths = window[window > 0]
        
        if len(valid_depths) == 0:
            return None
        
        # 返回中值深度（毫米轉公尺）
        return np.median(valid_depths) / 1000.0
    
    def select_target_person(self, detections, depth_image):
        """選擇目標人物（最近且在影像中央的人，使用深度過濾誤檢）"""
        if len(detections) == 0:
            return None
        
        valid_targets = []
        
        for detection in detections:
            center_x = detection['center_x']
            center_y = detection['center_y']
            bbox = detection['bbox']
            x, y, w, h = bbox
            
            # 獲取深度（使用檢測框內多個點的平均深度）
            depth_samples = []
            for dy in [h//4, h//2, 3*h//4]:
                for dx in [w//3, w//2, 2*w//3]:
                    d = self.get_depth_at_point(x + dx, y + dy, depth_image)
                    if d is not None and d > 0:
                        depth_samples.append(d)
            
            if len(depth_samples) == 0:
                continue
            
            # 使用中值深度（更穩健）
            depth = np.median(depth_samples)
            
            # 過濾超出範圍的偵測
            if depth < self.min_distance or depth > self.max_distance:
                continue
            
            # 深度一致性檢查（同一個人的深度應該相近）
            depth_std = np.std(depth_samples)
            if depth_std > 0.3:  # 深度變化太大，可能是誤檢
                continue
            
            # 計算與影像中心的距離（用於選擇中央的人）
            center_offset = abs(center_x - self.image_width / 2)
            
            valid_targets.append({
                'detection': detection,
                'depth': depth,
                'center_offset': center_offset,
                'depth_consistency': depth_std
            })
        
        if len(valid_targets) == 0:
            return None
        
        # 選擇最近的目標（如果多個目標距離相近，選擇更靠近中央的）
        valid_targets.sort(key=lambda x: (x['depth'], x['center_offset']))
        
        return valid_targets[0]
    
    def calculate_control_command(self, target):
        """計算控制命令"""
        cmd = Twist()
        
        if target is None:
            # 沒有目標，停止
            return cmd
        
        detection = target['detection']
        depth = target['depth']
        center_x = detection['center_x']
        
        # ========== 線速度控制 ==========
        # 距離誤差
        distance_error = depth - self.target_distance
        
        # 比例控制
        linear_velocity = self.linear_gain * distance_error
        
        # 限制速度
        linear_velocity = np.clip(
            linear_velocity,
            -self.max_linear_speed,
            self.max_linear_speed
        )
        
        # 死區控制（避免抖動）
        if abs(distance_error) < 0.1:
            linear_velocity = 0.0
        
        # ========== 角速度控制 ==========
        # 計算目標在影像中的偏移（正規化到[-1, 1]）
        center_offset = (center_x - self.image_width / 2) / (self.image_width / 2)
        
        # 比例控制
        angular_velocity = -self.angular_gain * center_offset
        
        # 限制速度
        angular_velocity = np.clip(
            angular_velocity,
            -self.max_angular_speed,
            self.max_angular_speed
        )
        
        # 死區控制
        if abs(center_offset) < 0.1:
            angular_velocity = 0.0
        
        # ========== 設定命令 ==========
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        
        return cmd
    
    def draw_debug_image(self, image, detections, target):
        """繪製除錯影像"""
        debug_image = image.copy()
        
        # 繪製所有偵測結果
        for detection in detections:
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            
            # 灰色框表示偵測到但未選中的人
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), (128, 128, 128), 2)
            cv2.putText(
                debug_image,
                f'{confidence:.2f}',
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (128, 128, 128),
                2
            )
        
        # 繪製選中的目標
        if target is not None:
            detection = target['detection']
            depth = target['depth']
            x, y, w, h = detection['bbox']
            center_x = detection['center_x']
            center_y = detection['center_y']
            
            # 綠色框表示追蹤目標
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
            
            # 顯示距離資訊
            text = f'Target: {depth:.2f}m'
            cv2.putText(
                debug_image,
                text,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )
            
            # 繪製中心點
            cv2.circle(debug_image, (center_x, center_y), 5, (0, 0, 255), -1)
        
        # 繪製影像中心線
        cv2.line(
            debug_image,
            (self.image_width // 2, 0),
            (self.image_width // 2, self.image_height),
            (255, 0, 0),
            2
        )
        
        # 顯示狀態資訊
        status_text = 'TRACKING' if target is not None else 'SEARCHING'
        color = (0, 255, 0) if target is not None else (0, 0, 255)
        cv2.putText(
            debug_image,
            status_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            color,
            2
        )
        
        return debug_image
    
    def process_callback(self):
        """主處理回調"""
        # 檢查是否有影像資料
        if self.latest_color_image is None or self.latest_depth_image is None:
            return
        
        try:
            # ========== 人體偵測 ==========
            if self.detection_method == 'hog':
                detections = self.detect_people_hog(self.latest_color_image)
            elif self.detection_method == 'dnn':
                detections = self.detect_people_dnn(self.latest_color_image)
            else:
                detections = []
            
            # ========== 選擇目標 ==========
            target = self.select_target_person(detections, self.latest_depth_image)
            
            # ========== 計算控制命令 ==========
            if target is not None:
                self.last_detection_time = time.time()
                
                if self.enable_follower:
                    cmd = self.calculate_control_command(target)
                    self.cmd_vel_pub.publish(cmd)
                    
                    # 發佈狀態
                    status_msg = String()
                    status_msg.data = f'TRACKING: {target["depth"]:.2f}m'
                    self.status_pub.publish(status_msg)
                    
                    self.get_logger().info(
                        f'🎯 追蹤目標 | 距離: {target["depth"]:.2f}m | '
                        f'線速度: {cmd.linear.x:.2f} | 角速度: {cmd.angular.z:.2f}',
                        throttle_duration_sec=1.0
                    )
            else:
                # 檢查超時
                time_since_detection = time.time() - self.last_detection_time
                
                if time_since_detection > self.no_person_timeout:
                    # 超時，停止機器人
                    cmd = Twist()
                    self.cmd_vel_pub.publish(cmd)
                    
                    # 發佈狀態
                    status_msg = String()
                    status_msg.data = 'SEARCHING'
                    self.status_pub.publish(status_msg)
                    
                    self.get_logger().info(
                        '🔍 未偵測到目標，機器人已停止',
                        throttle_duration_sec=2.0
                    )
            
            # ========== 發佈除錯影像 ==========
            if self.publish_debug_image:
                debug_image = self.draw_debug_image(
                    self.latest_color_image,
                    detections,
                    target
                )
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                self.debug_image_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'處理錯誤: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PeopleFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 發送停止命令
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.get_logger().info('🛑 人體追蹤節點已停止')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
