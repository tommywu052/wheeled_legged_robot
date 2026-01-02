# People Follower - 人體追蹤與姿態模仿套件

## 📋 功能總覽

這個 ROS2 套件提供了兩大核心功能，讓輪足機器人能夠智能地與人互動：

### 🎯 功能一：人體追蹤 (People Following)
- ✅ 實時人體偵測（HOG/DNN/YOLO方法）
- ✅ 深度資訊融合，準確測距
- ✅ 自適應速度控制
- ✅ 安全距離保持
- ✅ 視覺化除錯輸出

### 🤸 功能二：姿態模仿 (Posture Mimic) ⭐ 最新
- ✅ **橫滾控制**：雙手舉高控制機器人左右傾斜（±20°）
- ✅ **高度控制**：蹲下/站立/站高（0.14m - 0.35m）
- ✅ **俯仰控制**：右手指向控制機器人視角（±30°，可選）
- ✅ **超時重置**：3秒無檢測自動恢復初始姿態
- ✅ **智能檢測**：避免誤觸發（平舉不會觸發橫滾）

---

## 目錄

- [安裝依賴](#-安裝依賴)
- [編譯安裝](#-編譯安裝)
- [人體追蹤功能](#-人體追蹤功能)
- [姿態模仿功能](#-姿態模仿功能-最新)
- [故障排查](#-故障排查)
- [安全建議](#-安全建議)

---

## 🔧 安裝依賴

### 1. ROS2 依賴

```bash
sudo apt update
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-realsense2-camera \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs
```

### 2. Python 依賴

```bash
# 基礎套件
sudo apt install -y python3-pip python3-opencv python3-numpy

# YOLO 支援（用於姿態模仿）
pip3 install ultralytics opencv-python

# 驗證安裝
python3 -c "import cv2; print('OpenCV:', cv2.__version__)"
python3 -c "from ultralytics import YOLO; print('YOLO: OK')"
```

### 3. MediaPipe 支援（可選，用於高級姿態檢測）

```bash
pip3 install mediapipe
```

---

## 📦 編譯安裝

```bash
# 進入工作空間
cd ~/ros2_ws

# 編譯套件
colcon build --packages-select people_follower

# 更新環境變數
source install/setup.bash
```

---

## 🎯 人體追蹤功能

### 快速開始

#### 方法1：單獨啟動（相機已運行）

```bash
# 終端1: 啟動 RealSense 相機
ros2 launch realsense2_camera rs_launch.py \
  config_file:=/home/robotester1/legged_robot/realsense/realsense_params.yaml

# 終端2: 啟動人體追蹤
ros2 launch people_follower people_follower.launch.py
```

#### 方法2：一鍵啟動

```bash
ros2 launch people_follower people_follower_with_camera.launch.py
```

#### 方法3：測試模式（不移動機器人）

```bash
ros2 launch people_follower people_follower.launch.py enable_follower:=false
```

### 話題說明

#### 訂閱話題

| 話題 | 類型 | 說明 |
|------|------|------|
| `/camera/camera/color/image_raw` | sensor_msgs/Image | RGB 影像 |
| `/camera/camera/aligned_depth_to_color/image_raw` | sensor_msgs/Image | 對齊的深度影像 |
| `/camera/camera/color/camera_info` | sensor_msgs/CameraInfo | 相機參數 |

#### 發佈話題

| 話題 | 類型 | 說明 |
|------|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 機器人速度控制命令 |
| `/people_follower/debug_image` | sensor_msgs/Image | 視覺化除錯影像 |
| `/people_follower/status` | std_msgs/String | 追蹤狀態資訊 |

### 參數配置

編輯配置檔：`config/people_follower_params.yaml`

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `detection_method` | `hog` | 偵測方法：`hog`、`dnn` 或 `yolo` |
| `target_distance` | `1.0` | 目標追蹤距離（公尺）|
| `min_distance` | `0.5` | 最小安全距離（公尺）|
| `max_distance` | `3.0` | 最大偵測距離（公尺）|
| `max_linear_speed` | `0.3` | 最大前進速度（m/s）|
| `max_angular_speed` | `0.8` | 最大旋轉速度（rad/s）|

---

## 🤸 姿態模仿功能 ⭐ 最新

### 功能介紹

姿態模仿功能讓機器人能夠實時模仿人體動作，包括：

#### 1. 橫滾控制（Roll Control）
- **左手舉高** → 機器人向左傾斜 **-20°**
- **右手舉高** → 機器人向右傾斜 **+20°**
- **手放下** → 機器人恢復水平 **0°**

#### 2. 高度控制（Height Control）
- **站立** → 標準高度 **0.25m**
- **蹲下** → 降低至 **0.16m**
- **雙手舉高** → 站高至 **0.35m**
- **深蹲** → 最低 **0.14m**

#### 3. 俯仰控制（Pitch Control，可選）
- **右手向上指** → 機器人抬頭 **-30°**
- **右手水平指** → 機器人水平 **0°**
- **右手向下指** → 機器人低頭 **+30°**
- **目前狀態**：暫時關閉，可在配置中啟用

#### 4. 超時重置
- 未檢測到人超過 **3秒** 後，自動恢復到初始狀態
- 初始狀態：高度 0.25m，橫滾 0°，俯仰 0°

### 快速開始

#### 基本啟動

```bash
# 終端1：啟動 RealSense 相機
ros2 launch realsense2_camera rs_launch.py \
  config_file:=/home/robotester1/legged_robot/realsense/realsense_params.yaml

# 終端2：啟動姿態模仿
ros2 launch people_follower posture_mimic.launch.py
```

#### MediaPipe 版本（推薦，更穩定）

```bash
ros2 launch people_follower posture_mimic_mediapipe.launch.py
```

#### 監控姿態命令（可選）

```bash
# 終端3：監控完整姿態
ros2 topic echo /cmd_posture

# 或分別監控
ros2 topic echo /cmd_posture --field position[0]  # 高度
ros2 topic echo /cmd_posture --field position[1]  # 橫滾
ros2 topic echo /cmd_posture --field position[2]  # 俯仰
```

**注意：** 如果使用的是標準 RealSense 啟動（無配置文件），也可以使用：
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### 手勢控制表

| 手勢 | 左手 | 右手 | 高度 | 橫滾 | 俯仰 | 說明 |
|------|------|------|------|------|------|------|
| 正常站立 | 放下 | 放下 | 0.25m | 0° | 0° | 初始狀態 |
| **左手舉高** | **舉高** | 放下 | 0.25m | **-20°** | 0° | 向左傾斜 |
| **右手舉高** | 放下 | **舉高** | 0.25m | **+20°** | 0° | 向右傾斜 |
| 雙手舉高 | 舉高 | 舉高 | **0.35m** | 0° | 0° | 站高 |
| 蹲下 | 放下 | 放下 | **0.16m** | 0° | 0° | 降低高度 |
| 平舉 | 平舉 | - | 0.25m | **0°** | 0° | **不觸發** ✅ |
| 超時3秒 | - | - | **0.25m** | **0°** | **0°** | **自動重置** |

**重要說明：**
- ✅ 舉手需要手腕**明顯高於肩膀**且**接近頭部**才會觸發
- ✅ 手臂**平舉**（水平伸出）**不會觸發**橫滾（已優化）
- ✅ 所有動作都是**立即響應**，無延遲
- ✅ 離開視野3秒後自動恢復安全姿態

### 話題說明

#### 訂閱話題

| 話題 | 類型 | 說明 |
|------|------|------|
| `/camera/camera/color/image_raw` | sensor_msgs/Image | RGB 影像 |
| `/camera/camera/aligned_depth_to_color/image_raw` | sensor_msgs/Image | 深度影像（用於高度計算）|
| `/camera/camera/color/camera_info` | sensor_msgs/CameraInfo | 相機內參 |

#### 發佈話題

| 話題 | 類型 | 說明 |
|------|------|------|
| `/cmd_posture` | sensor_msgs/JointState | 姿態控制命令 ⭐ |
| `/posture_mimic/debug_image` | sensor_msgs/Image | 視覺化除錯影像 |
| `/posture_mimic/status` | std_msgs/String | 模仿狀態資訊 |

**`/cmd_posture` 格式：**
```yaml
name: ['joint_height', 'joint_roll', 'joint_pitching']
position: [0.25, 0.0, 0.0]  # [高度(m), 橫滾(rad), 俯仰(rad)]
```

### 參數配置

編輯配置檔：`config/posture_mimic_params.yaml`

#### 姿態範圍

```yaml
# 高度參數（米）
height_stand: 0.25      # 標準站立高度（初始高度）⭐
height_squat: 0.16      # 蹲下高度
height_max: 0.35        # 最大高度（站高）
height_min: 0.14        # 最小高度（深蹲）

# 角度參數（度）
roll_max: 20.0          # 橫滾最大角度（雙手舉高控制）⭐
pitch_max: 30.0         # 俯仰最大角度（右手指向控制）⭐
```

#### 功能開關

```yaml
enable_height: true     # 啟用高度模仿 ✅
enable_roll: true       # 啟用橫滾模仿 ✅
enable_pitch: false     # 俯仰模仿（暫時關閉，可啟用）
```

#### 檢測參數

```yaml
min_confidence: 0.15              # 關鍵點最小置信度
detection_timeout: 3.0            # 檢測超時（秒）⭐
smoothing_window: 3               # 平滑濾波窗口大小
```

#### 變化閾值（防止抖動）

```yaml
height_deadzone: 0.01             # 高度變化死區（米）
roll_deadzone: 3.0                # 橫滾變化死區（度）
pitch_deadzone: 10.0              # 俯仰變化死區（度）
```

### 視覺化除錯

查看實時骨架和姿態資訊：

```bash
# 方法1: 使用 rqt_image_view
ros2 run rqt_image_view rqt_image_view /posture_mimic/debug_image

# 方法2: 使用 RViz2
rviz2
# 添加 Image display，選擇話題 /posture_mimic/debug_image
```

**除錯影像包含：**
- 🟢 綠色骨架：檢測到的人體關鍵點
- 📊 左上角資訊：高度、橫滾、俯仰角度
- 🎯 檢測狀態：舉手、蹲下等
- 📈 FPS 顯示

### 測試流程

#### 1. 基本測試（推薦順序）

```bash
# Step 1: 啟動系統
ros2 launch people_follower posture_mimic.launch.py

# Step 2: 站在相機前方 1-2 米，測試各種動作

# 測試橫滾
✓ 左手舉高 → 機器人向左傾斜
✓ 右手舉高 → 機器人向右傾斜
✓ 手放下 → 機器人恢復水平

# 測試高度
✓ 蹲下 → 機器人降低
✓ 站起 → 機器人升高
✓ 雙手舉高 → 機器人站高

# 測試超時重置
✓ 做動作後離開視野 3 秒
✓ 觀察機器人是否自動恢復站立姿態
```

#### 2. 驗證平舉不觸發

```bash
# 重要測試：確保手臂平舉不會誤觸發

✓ 左手平舉（水平伸出）→ 橫滾應該保持 0°
✓ 右手平舉（水平伸出）→ 橫滾應該保持 0°

# 如果平舉會觸發，說明檢測閾值需要調整
```

#### 3. 監控姿態值

```bash
# 實時查看姿態數值
ros2 topic echo /cmd_posture --field position

# 預期輸出範例
[0.25, 0.0, 0.0]        # 正常站立
[0.25, -0.349, 0.0]     # 左手舉高（-20° = -0.349 rad）
[0.35, 0.0, 0.0]        # 雙手舉高（站高）
[0.16, 0.0, 0.0]        # 蹲下
```

### 啟用俯仰控制（可選）

如果想要啟用右手指向控制俯仰角：

```bash
# 1. 編輯配置檔
nano ~/ros2_ws/src/people_follower/config/posture_mimic_params.yaml

# 2. 修改參數
enable_pitch: true  # 從 false 改為 true

# 3. 重新編譯
cd ~/ros2_ws
colcon build --packages-select people_follower
source install/setup.bash

# 4. 重新啟動
ros2 launch people_follower posture_mimic.launch.py
```

**俯仰控制手勢：**
- 👆 右手向上指（手臂伸直）→ 機器人抬頭 -30°
- 👉 右手水平指 → 機器人水平 0°
- 👇 右手向下指（手臂伸直）→ 機器人低頭 +30°

**注意：** 俯仰控制與橫滾控制智能區分：
- 右手**舉高**（手腕靠近頭部）→ 橫滾控制
- 右手**指向**（手臂伸直，手腕遠離頭部）→ 俯仰控制

---

## 🛠️ 故障排查

### 人體追蹤問題

#### 問題1：偵測不到人

**可能原因：**
- 光照條件不好
- 人距離太遠（超過 `max_distance`）
- 相機被遮擋

**解決方法：**
```bash
# 調整偵測距離範圍
ros2 param set /people_follower_node max_distance 5.0
ros2 param set /people_follower_node min_distance 0.3

# 檢查相機話題
ros2 topic list | grep camera
ros2 topic hz /camera/camera/color/image_raw
```

#### 問題2：機器人不移動

```bash
# 檢查參數
ros2 param get /people_follower_node enable_follower

# 查看速度命令
ros2 topic echo /cmd_vel

# 檢查日誌
ros2 node info /people_follower_node
```

### 姿態模仿問題

#### 問題1：手臂平舉會觸發橫滾

**症狀：** 手臂水平伸出時，機器人傾斜

**解決：**
```bash
# 確認版本
grep "v1.4.5" ~/ros2_ws/src/people_follower/people_follower/posture_mimic_node.py

# 如果不是最新版本，重新編譯
cd ~/ros2_ws
rm -rf build/people_follower install/people_follower
colcon build --packages-select people_follower
source install/setup.bash
```

#### 問題2：超時後沒有重置

**症狀：** 離開視野3秒後，機器人保持原來的姿態

**檢查：**
```bash
# 查看日誌，應該看到
# "⚠️ 檢測超時 3.0s！重置姿態到初始狀態"

# 如果沒有，檢查超時參數
ros2 param get /posture_mimic_node detection_timeout

# 應該是 3.0
```

#### 問題3：初始高度不對

**症狀：** 啟動後高度不是 0.25m

**解決：**
```bash
# 檢查配置
grep "height_stand" ~/ros2_ws/src/people_follower/config/posture_mimic_params.yaml

# 應該顯示：height_stand: 0.25

# 檢查實時參數
ros2 param get /posture_mimic_node height_stand

# 臨時修改（不需重啟）
ros2 param set /posture_mimic_node height_stand 0.25
```

#### 問題4：橫滾歸零不精確

**症狀：** 手放下後，橫滾角不是 0.0° 而是 ±2-3°

**原因：** 這是舊版本問題，v1.4.5.4+ 已修復

**解決：** 確保使用最新版本
```bash
cd ~/ros2_ws
colcon build --packages-select people_follower
source install/setup.bash
```

#### 問題5：檢測不穩定

**症狀：** 骨架檢測跳動或消失

**解決方案：**
```bash
# 方法1：改善光線條件
# - 增加環境光
# - 避免背光

# 方法2：調整置信度閾值
ros2 param set /posture_mimic_node min_confidence 0.10
# 降低閾值（從0.15到0.10）

# 方法3：使用 MediaPipe 版本（更穩定）
ros2 launch people_follower posture_mimic_mediapipe.launch.py
```

### 相機問題

#### 問題：相機話題找不到

```bash
# 列出所有話題
ros2 topic list | grep camera

# 檢查相機是否正常
ros2 topic hz /camera/camera/color/image_raw

# 如果話題名稱不同，需要修改 launch 檔案中的 remappings
```

---

## 🔐 安全建議

### 使用前必讀 ⚠️

#### 人體追蹤安全

1. **首次測試時請有人準備緊急停止**
2. **從低速開始測試**（max_linear_speed: 0.15）
3. **確保測試區域開闊**，無障礙物
4. **保持遙控器隨時可用**，以便緊急接管
5. **注意電池電量**，低電量時性能下降

#### 姿態模仿安全

1. **測試時確保機器人周圍無人**
2. **橫滾角度最大 ±20°**，注意穩定性
3. **超時重置功能確保安全**（3秒自動恢復）
4. **初次測試建議在地面平坦處**
5. **避免在斜坡或不穩定表面使用**

### 緊急停止方法

```bash
# 方法1: Ctrl+C 停止節點

# 方法2: 發送零速度命令
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"

# 方法3: 發送零姿態命令
ros2 topic pub --once /cmd_posture sensor_msgs/JointState \
  "{name: ['joint_height', 'joint_roll', 'joint_pitching'], \
    position: [0.25, 0.0, 0.0]}"
```

---

## 📈 性能優化建議

### 不同場景的參數推薦

#### 人體追蹤

**室內狹窄空間：**
```yaml
max_linear_speed: 0.2
max_angular_speed: 0.6
target_distance: 0.8
```

**室外開闊空間：**
```yaml
max_linear_speed: 0.4
max_angular_speed: 1.0
target_distance: 1.5
```

#### 姿態模仿

**演示模式（保守）：**
```yaml
height_deadzone: 0.02       # 較大死區，減少抖動
roll_deadzone: 5.0          # 較大死區
smoothing_window: 5         # 更平滑
```

**競技模式（靈敏）：**
```yaml
height_deadzone: 0.005      # 較小死區，更靈敏
roll_deadzone: 2.0          # 較小死區
smoothing_window: 2         # 更快響應
```

---

## 📝 版本資訊

### 當前版本

- **人體追蹤**：v1.0.0
- **姿態模仿**：v1.4.5.5 ⭐ 最新穩定版

### v1.4.5.5 更新內容（2026-01-02）

✅ **主要功能：**
1. 橫滾雙手控制（左手 -20°，右手 +20°）
2. 橫滾嚴格檢測（平舉不觸發）
3. 超時自動重置（3秒無檢測恢復初始狀態）
4. 初始高度正確（0.25m）
5. 俯仰控制（已實現，可選啟用）

✅ **性能特性：**
- 立即響應（橫滾 1幀 = 33ms @ 30fps）
- 精確歸零（0.0°，無殘留）
- 智能檢測（使用相對閾值，適應不同身高）
- 自動安全（超時自動重置）

### 版本演進歷史

```
v1.4.3   → 修復俯仰控制
v1.4.4   → 實現單手橫滾
v1.4.4.1 → 橫滾立即響應
v1.4.5   → 右手指向俯仰
v1.4.5.1 → 橫滾雙手控制
v1.4.5.2 → 俯仰角方向修正
v1.4.5.3 → 橫滾檢測優化
v1.4.5.4 → 超時重置功能
v1.4.5.5 → 初始高度修正 ⭐ 當前版本
```

---

## 📞 技術支援

### 檢查清單

如遇問題，請依序檢查：

1. ✅ ROS2 和 RealSense 驅動版本相容性
2. ✅ 系統資源佔用情況（CPU/記憶體）
3. ✅ 相機是否正常工作（`ros2 topic list | grep camera`）
4. ✅ 日誌輸出資訊（查看錯誤訊息）
5. ✅ 配置檔案參數是否正確
6. ✅ 最新版本是否已編譯

### 常用診斷命令

```bash
# 查看所有節點
ros2 node list

# 查看話題列表
ros2 topic list

# 查看節點資訊
ros2 node info /people_follower_node
ros2 node info /posture_mimic_node

# 查看參數
ros2 param list /posture_mimic_node

# 查看日誌
ros2 run rqt_console rqt_console
```

### 獲取更多資訊

詳細文檔位於：`~/ros2_ws/src/people_follower/`

- 📄 各版本更新說明文檔
- 📄 測試指南文檔
- 📄 故障排除文檔

---

## 📄 授權

MIT License

---

## 👥 開發團隊

**開發者**: robotester1  
**最後更新**: 2026-01-02  
**版本**: v1.4.5.5

---

## 🎯 快速參考

### 最常用命令

```bash
# 啟動 RealSense 相機（含配置）
ros2 launch realsense2_camera rs_launch.py \
  config_file:=/home/robotester1/legged_robot/realsense/realsense_params.yaml

# 或簡化版（手動啟用深度對齊）
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

# 啟動人體追蹤
ros2 launch people_follower people_follower.launch.py

# 啟動姿態模仿
ros2 launch people_follower posture_mimic.launch.py

# 啟動姿態模仿（MediaPipe 版本）
ros2 launch people_follower posture_mimic_mediapipe.launch.py

# 監控姿態
ros2 topic echo /cmd_posture

# 查看除錯影像
ros2 run rqt_image_view rqt_image_view /posture_mimic/debug_image

# 緊急停止
Ctrl+C
```

### 姿態模仿測試清單

- [ ] 左手舉高 → 橫滾 -20°
- [ ] 右手舉高 → 橫滾 +20°
- [ ] 手放下 → 橫滾 0°
- [ ] 平舉不觸發 → 橫滾保持 0°
- [ ] 蹲下 → 高度降低
- [ ] 站起 → 高度恢復
- [ ] 雙手舉高 → 站高到 0.35m
- [ ] 離開3秒 → 自動重置

**所有測試通過 = 系統正常！** ✅

---

**享受智能機器人帶來的樂趣！** 🤖🎉
