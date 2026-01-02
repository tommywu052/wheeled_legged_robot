#!/usr/bin/env python3
"""
ROS2版本的偵測測試腳本
用於測試HOG偵測器在RealSense影像上的效果
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectionTestNode(Node):
    """簡單的偵測測試節點"""
    
    def __init__(self):
        super().__init__('detection_test_node')
        
        # 初始化
        self.bridge = CvBridge()
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        # 訂閱RealSense RGB影像（使用正確的話題名稱）
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.frame_count = 0
        
        self.get_logger().info('🚀 偵測測試節點已啟動')
        self.get_logger().info('   訂閱話題: /camera/camera/color/image_raw')
        self.get_logger().info('   按 Ctrl+C 停止測試')
        
    def image_callback(self, msg):
        """處理接收到的影像"""
        try:
            # 計數接收到的幀
            self.frame_count += 1
            if self.frame_count == 1:
                self.get_logger().info('✅ 成功接收到影像！開始偵測...')
            
            # 轉換ROS影像為OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 改善對比度（有助於偵測）
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)  # 直方圖均衡化
            gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            
            # HOG人體偵測（平衡準確度和召回率）
            detections, weights = self.hog.detectMultiScale(
                gray_bgr,
                winStride=(8, 8),      # 步長
                padding=(8, 8),        # 邊界
                scale=1.05,            # 尺度步進
                finalThreshold=2.0,    # 提高閾值減少誤檢（原本1.5）
                useMeanshiftGrouping=False
            )
            
            # 過濾和繪製偵測結果
            for i, (x, y, w, h) in enumerate(detections):
                # 修正：weights是numpy陣列，不是list of list
                try:
                    confidence = float(weights[i]) if len(weights) > 0 and i < len(weights) else 1.0
                except:
                    confidence = 1.0
                
                # 過濾條件：
                # 1. 置信度要高於閾值
                # 2. 檢測框要有合理的寬高比（人體通常是直立的）
                aspect_ratio = h / w if w > 0 else 0
                is_reasonable_size = (h > 100) and (w > 40)  # 最小尺寸過濾
                is_person_like = (1.5 <= aspect_ratio <= 4.0)  # 人體寬高比
                    
                if confidence > 1.0 and is_reasonable_size and is_person_like:
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(
                        frame,
                        f'Person {i+1}: {confidence:.2f}',
                        (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )
            
            # 顯示偵測數量和影像資訊
            h, w = frame.shape[:2]
            cv2.putText(
                frame,
                f'Detected: {len(detections)} people',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0) if len(detections) > 0 else (0, 0, 255),
                2
            )
            
            # 顯示解析度
            cv2.putText(
                frame,
                f'Size: {w}x{h}',
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1
            )
            
            # 顯示提示
            cv2.putText(
                frame,
                "Tips: Stand 1-3m away, upright posture",
                (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                1
            )
            
            # 顯示影像
            cv2.imshow('RealSense Detection Test', frame)
            
            # 按'q'退出，按's'儲存截圖
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('使用者請求退出')
                rclpy.shutdown()
            elif key == ord('s'):
                filename = f'realsense_test_{np.random.randint(1000)}.jpg'
                cv2.imwrite(filename, frame)
                self.get_logger().info(f'📸 截圖已儲存: {filename}')
            
            # 定期輸出偵測資訊
            if len(detections) > 0:
                self.get_logger().info(
                    f'✅ 偵測到 {len(detections)} 人',
                    throttle_duration_sec=2.0
                )
                
        except Exception as e:
            self.get_logger().error(f'處理影像時發生錯誤: {e}')


def main(args=None):
    print("=" * 60)
    print("  RealSense HOG Detection Test")
    print("=" * 60)
    print("")
    print("使用說明:")
    print("  - 站在相機前測試偵測")
    print("  - 按 'q' 退出")
    print("  - 按 's' 儲存截圖")
    print("")
    print("確保RealSense節點已啟動:")
    print("  ros2 launch realsense2_camera rs_launch.py")
    print("=" * 60)
    print("")
    
    rclpy.init(args=args)
    
    try:
        node = DetectionTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n測試結束")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

