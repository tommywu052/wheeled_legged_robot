#!/usr/bin/env python3
"""
簡單的偵測測試腳本
用於驗證人體偵測是否正常工作（不移動機器人）
"""

import cv2
import numpy as np

def test_hog_detector():
    """測試HOG偵測器"""
    print("初始化HOG偵測器...")
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    print("✅ HOG偵測器初始化成功")
    
    # 打開攝影機測試
    print("\n嘗試打開攝影機...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ 無法打開攝影機")
        return
    
    print("✅ 攝影機已打開")
    print("\n按 'q' 退出, 按 's' 儲存截圖")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 無法讀取幀")
            break
        
        # 偵測人體
        detections, weights = hog.detectMultiScale(
            frame,
            winStride=(8, 8),
            padding=(4, 4),
            scale=1.05
        )
        
        # 繪製偵測結果
        for i, (x, y, w, h) in enumerate(detections):
            confidence = weights[i][0] if len(weights) > 0 else 0.0
            if confidence > 0.5:
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
        
        # 顯示偵測數量
        cv2.putText(
            frame,
            f'Detected: {len(detections)} people',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )
        
        cv2.imshow('People Detection Test', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f'detection_test_{np.random.randint(1000)}.jpg'
            cv2.imwrite(filename, frame)
            print(f"📸 截圖已儲存: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    print("\n測試結束")


if __name__ == '__main__':
    print("=" * 50)
    print("People Detection Test Script")
    print("=" * 50)
    test_hog_detector()
