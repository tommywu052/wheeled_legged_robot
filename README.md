# 輪足機器人導航系統 - 一鍵啟動 - 立即開始

本指南將幫助您在5分鐘內完成所有配置並開始使用！

---

## 步驟1：編譯系統 （2分鐘 , Note I'm using Jetson Orin NX 16G）

```bash
cd ~/legged_robot/ROS2_Packages
colcon build --packages-select wheeled_legged_pkg --symlink-install
source install/setup.bash
```

---

## 步驟2：添加執行權限（30秒）

```bash
chmod +x ~/legged_robot/ROS2_Packages/wheeled_legged_pkg/scripts/start_full_navigation.sh
chmod +x ~/legged_robot/ROS2_Packages/wheeled_legged_pkg/scripts/auto_init_amcl.py
```

---

## 步驟3：一鍵啟動！（30秒）

```bash
~/legged_robot/ROS2_Packages/wheeled_legged_pkg/scripts/start_full_navigation.sh
```

系統會自動：
- ✅ 檢查串口權限
- ✅ 啟動機器人基礎節點（自動選擇 /dev/ttyACM0）
- ✅ 啟動激光雷達
- ✅ 啟動導航系統（包含 RViz）
- ✅ 等待15秒後自動初始化 AMCL

---

## 步驟4：在 RViz 中導航（1分鐘）

等待自動初始化完成（約15秒），然後：

1. 在 RViz 中，將 Fixed Frame 切換到 "map"
2. (可選) 使用 "2D Pose Estimate" 微調機器人位置
3. 使用 "2D Goal Pose" 設置導航目標
4. 觀察機器人自動導航！

---

## 管理 tmux 會話

### 查看會話
```bash
tmux attach -t nav_system
```

### 切換窗口
按 `Ctrl+B` 然後按數字鍵 (0,1,2,3)
- 0 = robot_base (機器人基礎)
- 1 = lidar (激光雷達)
- 2 = navigation (導航系統)
- 3 = auto_init (自動初始化)

### 退出查看
按 `Ctrl+B` 然後按 `D`

### 關閉系統
```bash
tmux kill-session -t nav_system
```

---

## 常見問題快速修復

### 問題：串口權限不足

**臨時解決：**
```bash
sudo chmod 666 /dev/ttyACM0
```

**永久解決：**
```bash
~/legged_robot/ROS2_Packages/wheeled_legged_pkg/scripts/setup_serial_permissions.sh
```

### 問題：激光雷達未啟動

**檢查：**
```bash
ros2 topic hz /scan
```

**手動啟動：**
```bash
cd ~/legged_robot/LSLIDAR_X_ROS2/src
source install/setup.bash
ros2 launch lslidar_driver lsn10_launch.py
```

### 問題：AMCL 未初始化

**檢查：**
```bash
ros2 lifecycle get /amcl
```

**手動初始化：**
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]}}'
```

---

## 系統狀態檢查

### 檢查所有節點
```bash
ros2 node list
```

### 檢查激光雷達
```bash
ros2 topic hz /scan
```

### 檢查里程計
```bash
ros2 topic hz /odom
```

### 檢查 AMCL 狀態
```bash
ros2 lifecycle get /amcl
```

### 檢查 TF 樹
```bash
ros2 run tf2_ros view_frames
```

---

## 就這麼簡單！

現在運行：

```bash
~/legged_robot/ROS2_Packages/wheeled_legged_pkg/scripts/start_full_navigation.sh
```

享受自動化的導航體驗！🚀


