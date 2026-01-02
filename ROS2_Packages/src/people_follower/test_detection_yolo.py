#!/usr/bin/env python3
"""
YOLOv8人體偵測測試腳本
用於驗證YOLOv8在RealSense影像上的偵測效果
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
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
    print("❌ YOLOv8未安裝！")
    print("請執行: pip3 install ultralytics")
    exit(1)


class YOLODetectionTest(Node):
    """YOLOv8偵測測試節點"""
    
    def __init__(self):
        super().__init__('yolo_detection_test')
        
        # 初始化
        self.bridge = CvBridge()
        self.frame_count = 0
        self.detection_count = 0
        self.start_time = time.time()
        
        # 載入YOLOv8模型
        print("=" * 60)
        print("  YOLOv8 Detection Test")
        print("=" * 60)
        print("")
        print("📦 載入YOLOv8n模型...")
        
        try:
            self.model = YOLO('yolov8n.pt')  # 自動下載如果不存在
            print(f"✅ 模型已載入")
            print(f"   裝置: {'CUDA' if self.model.device.type == 'cuda' else 'CPU'}")
            
            # 測試CUDA
            import torch
            if torch.cuda.is_available():
                print(f"   GPU: {torch.cuda.get_device_name(0)}")
                print(f"   CUDA版本: {torch.version.cuda}")
            else:
                print("   ⚠️  CUDA不可用，使用CPU（會較慢）")
        except Exception as e:
            print(f"❌ 模型載入失敗: {e}")
            exit(1)
        
        print("")
        print("使用說明:")
        print("  - 站在相機前測試偵測")
        print("  - 按 'q' 退出")
        print("  - 按 's' 儲存截圖")
        print("  - 按 'c' 切換信心閾值 (0.3/0.5/0.7)")
        print("")
        print("=" * 60)
        print("")
        
        # 偵測參數
        self.confidence_threshold = 0.5
        self.confidence_levels = [0.3, 0.5, 0.7]
        self.confidence_index = 1
        
        # 訂閱RealSense RGB影像
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('🚀 YOLOv8測試節點已啟動')
        self.get_logger().info(f'   信心閾值: {self.confidence_threshold}')
        self.get_logger().info('   訂閱話題: /camera/camera/color/image_raw')
        
    def image_callback(self, msg):
        """處理接收到的影像"""
        try:
            self.frame_count += 1
            
            # 轉換ROS影像為OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = frame.shape[:2]
            
            # 每30幀顯示一次接收訊息
            if self.frame_count == 1:
                self.get_logger().info('✅ 成功接收到影像！開始偵測...')
            
            # YOLOv8推理
            start_inference = time.time()
            results = self.model(frame, verbose=False, conf=self.confidence_threshold)
            inference_time = (time.time() - start_inference) * 1000  # 毫秒
            
            # 處理偵測結果
            detections = []
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # 只保留person類別 (class_id = 0)
                    if int(box.cls) == 0:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        confidence = float(box.conf)
                        
                        detections.append({
                            'bbox': (int(x1), int(y1), int(x2), int(y2)),
                            'confidence': confidence
                        })
            
            # 統計
            if len(detections) > 0:
                self.detection_count += 1
            
            # 繪製結果
            display_frame = frame.copy()
            
            # 繪製所有偵測框
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                conf = det['confidence']
                
                # 綠色框
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # 標籤
                label = f'Person {conf:.2f}'
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(display_frame, (x1, y1 - label_size[1] - 10),
                            (x1 + label_size[0], y1), (0, 255, 0), -1)
                cv2.putText(display_frame, label, (x1, y1 - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            
            # 顯示資訊
            info_y = 30
            line_height = 35
            
            # 偵測數量（大字）
            status_color = (0, 255, 0) if len(detections) > 0 else (0, 0, 255)
            cv2.putText(display_frame, f'Detected: {len(detections)} people',
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
            info_y += line_height
            
            # 解析度
            cv2.putText(display_frame, f'Size: {w}x{h}',
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            info_y += line_height - 5
            
            # 推理時間和FPS
            fps = self.frame_count / (time.time() - self.start_time)
            cv2.putText(display_frame, f'Inference: {inference_time:.1f}ms | FPS: {fps:.1f}',
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            info_y += line_height - 5
            
            # 信心閾值
            cv2.putText(display_frame, f'Confidence: {self.confidence_threshold:.1f}',
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            
            # 檢測率
            detection_rate = (self.detection_count / self.frame_count * 100) if self.frame_count > 0 else 0
            cv2.putText(display_frame, f'Detection Rate: {detection_rate:.1f}%',
                       (10, h - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # 操作提示
            cv2.putText(display_frame, "Q:Quit | S:Save | C:Change Confidence",
                       (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # 畫面中心線
            cv2.line(display_frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
            
            # 顯示影像
            cv2.imshow('YOLOv8 Detection Test', display_frame)
            
            # 處理按鍵
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('使用者請求退出')
                
                # 顯示統計
                runtime = time.time() - self.start_time
                avg_fps = self.frame_count / runtime
                detection_rate = (self.detection_count / self.frame_count * 100)
                
                print("")
                print("=" * 60)
                print("  測試統計")
                print("=" * 60)
                print(f"  總幀數: {self.frame_count}")
                print(f"  偵測到人的幀數: {self.detection_count}")
                print(f"  偵測率: {detection_rate:.1f}%")
                print(f"  平均FPS: {avg_fps:.1f}")
                print(f"  運行時間: {runtime:.1f}秒")
                print("=" * 60)
                
                rclpy.shutdown()
                
            elif key == ord('s'):
                filename = f'yolo_test_{int(time.time())}.jpg'
                cv2.imwrite(filename, display_frame)
                self.get_logger().info(f'📸 截圖已儲存: {filename}')
                
            elif key == ord('c'):
                # 切換信心閾值
                self.confidence_index = (self.confidence_index + 1) % len(self.confidence_levels)
                self.confidence_threshold = self.confidence_levels[self.confidence_index]
                self.get_logger().info(f'信心閾值已切換至: {self.confidence_threshold}')
            
            # 定期輸出偵測資訊
            if len(detections) > 0 and self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'✅ 偵測到 {len(detections)} 人 | '
                    f'FPS: {fps:.1f} | '
                    f'推理: {inference_time:.1f}ms',
                    throttle_duration_sec=1.0
                )
                
        except Exception as e:
            self.get_logger().error(f'處理影像時發生錯誤: {e}')
            import traceback
            traceback.print_exc()


def main(args=None):
    if not YOLO_AVAILABLE:
        print("")
        print("=" * 60)
        print("  錯誤：YOLOv8未安裝")
        print("=" * 60)
        print("")
        print("請執行以下命令安裝:")
        print("  pip3 install ultralytics")
        print("")
        print("或")
        print("  sudo apt install python3-pip")
        print("  pip3 install ultralytics")
        print("")
        return
    
    rclpy.init(args=args)
    
    try:
        node = YOLODetectionTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n測試中斷")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


