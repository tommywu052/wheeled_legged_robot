#!/bin/bash
# 完整導航系統一鍵啟動腳本
# 支持跨工作空間啟動所有組件

set -e  # 遇到錯誤立即退出

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║        輪足機器人導航系統 - 一鍵啟動腳本                    ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# ═══════════════════════════════════════════════════════════════
# 配置參數
# ═══════════════════════════════════════════════════════════════

# 機器人基礎節點
ROBOT_WS="$HOME/legged_robot/ROS2_Packages/src"
ROBOT_SERIAL_PORT="/dev/ttyACM0"

# 激光雷達
LIDAR_WS="$HOME/legged_robot/LSLIDAR_X_ROS2/src"
LIDAR_LAUNCH="lsn10_launch.py"

# 地圖和參數
MAP_FILE="$HOME/maps/my_map2.yaml"

# AMCL 初始位置
INIT_X=0.0
INIT_Y=0.0
INIT_YAW=0.0
WAIT_TIME=15  # 等待時間（秒）

# ═══════════════════════════════════════════════════════════════
# 檢查先決條件
# ═══════════════════════════════════════════════════════════════

echo "1️⃣  檢查先決條件..."
echo ""

# 檢查 tmux
if ! command -v tmux &> /dev/null; then
    echo "❌ 未安裝 tmux"
    echo "   請安裝: sudo apt install tmux"
    exit 1
fi
echo "✓ tmux 已安裝"

# 檢查機器人工作空間
if [ ! -d "$ROBOT_WS" ]; then
    echo "❌ 機器人工作空間不存在: $ROBOT_WS"
    exit 1
fi
echo "✓ 機器人工作空間: $ROBOT_WS"

# 檢查激光雷達工作空間
if [ ! -d "$LIDAR_WS" ]; then
    echo "⚠️  警告：激光雷達工作空間不存在: $LIDAR_WS"
    echo "   將跳過激光雷達啟動"
    SKIP_LIDAR=true
else
    echo "✓ 激光雷達工作空間: $LIDAR_WS"
    SKIP_LIDAR=false
fi

# 檢查地圖文件
if [ ! -f "$MAP_FILE" ]; then
    echo "❌ 地圖文件不存在: $MAP_FILE"
    exit 1
fi
echo "✓ 地圖文件: $MAP_FILE"

# 檢查串口權限
if [ -e "$ROBOT_SERIAL_PORT" ]; then
    if [ ! -r "$ROBOT_SERIAL_PORT" ] || [ ! -w "$ROBOT_SERIAL_PORT" ]; then
        echo "⚠️  警告：串口權限不足: $ROBOT_SERIAL_PORT"
        echo "   建議運行串口權限配置腳本"
        echo "   或臨時修復: sudo chmod 666 $ROBOT_SERIAL_PORT"
        read -p "是否繼續？[Y/n] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Nn]$ ]]; then
            exit 1
        fi
    else
        echo "✓ 串口權限: $ROBOT_SERIAL_PORT"
    fi
else
    echo "⚠️  警告：串口不存在: $ROBOT_SERIAL_PORT"
fi

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "2️⃣  啟動配置"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "機器人串口: $ROBOT_SERIAL_PORT"
echo "地圖文件: $MAP_FILE"
echo "初始位置: ($INIT_X, $INIT_Y), yaw=$INIT_YAW"
echo "等待時間: ${WAIT_TIME}秒"
echo ""
read -p "確認啟動？[Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Nn]$ ]]; then
    echo "已取消"
    exit 0
fi

# ═══════════════════════════════════════════════════════════════
# 啟動 tmux 會話
# ═══════════════════════════════════════════════════════════════

SESSION_NAME="nav_system"

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "3️⃣  啟動導航系統..."
echo "═══════════════════════════════════════════════════════════════"
echo ""

# 檢查會話是否已存在
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo "⚠️  tmux 會話 '$SESSION_NAME' 已存在"
    read -p "是否關閉並重新創建？[Y/n] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
        tmux kill-session -t $SESSION_NAME
        echo "✓ 已關閉舊會話"
    else
        echo "❌ 請先關閉舊會話: tmux kill-session -t $SESSION_NAME"
        exit 1
    fi
fi

# 創建新會話
echo "正在創建 tmux 會話: $SESSION_NAME"
tmux new-session -d -s $SESSION_NAME -n "robot_base"

# ========== 窗口1：機器人基礎節點 ==========
echo "  [1/4] 啟動機器人基礎節點..."
tmux send-keys -t $SESSION_NAME:robot_base "cd $ROBOT_WS" C-m
tmux send-keys -t $SESSION_NAME:robot_base "source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:robot_base "ros2 run wheeled_legged_pkg wl_base_node --ros-args -p serial_port:=$ROBOT_SERIAL_PORT -p auto_select:=true" C-m

sleep 2

# ========== 窗口2：激光雷達 ==========
if [ "$SKIP_LIDAR" = false ]; then
    echo "  [2/4] 啟動激光雷達..."
    tmux new-window -t $SESSION_NAME -n "lidar"
    tmux send-keys -t $SESSION_NAME:lidar "cd $LIDAR_WS" C-m
    tmux send-keys -t $SESSION_NAME:lidar "source install/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:lidar "ros2 launch lslidar_driver $LIDAR_LAUNCH" C-m
    sleep 2
else
    echo "  [2/4] ⚠️  跳過激光雷達啟動（工作空間不存在）"
fi

# ========== 窗口3：導航系統 ==========
echo "  [3/4] 啟動導航系統..."
tmux new-window -t $SESSION_NAME -n "navigation"
tmux send-keys -t $SESSION_NAME:navigation "cd $ROBOT_WS" C-m
tmux send-keys -t $SESSION_NAME:navigation "source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:navigation "ros2 launch wheeled_legged_pkg navigation_with_lidar.launch.py map:=$MAP_FILE" C-m

sleep 3

# ========== 窗口4：自動初始化 AMCL ==========
echo "  [4/4] 啟動自動初始化..."
tmux new-window -t $SESSION_NAME -n "auto_init"
tmux send-keys -t $SESSION_NAME:auto_init "cd $ROBOT_WS" C-m
tmux send-keys -t $SESSION_NAME:auto_init "source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo '⏳ 等待 ${WAIT_TIME} 秒后自动初始化 AMCL...'" C-m
tmux send-keys -t $SESSION_NAME:auto_init "sleep $WAIT_TIME" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo '📍 正在初始化 AMCL...'" C-m
tmux send-keys -t $SESSION_NAME:auto_init "ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: \"map\"}, pose: {pose: {position: {x: $INIT_X, y: $INIT_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]}}'" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo ''" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo '✅ AMCL 已初始化！'" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo ''" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo '🎯 现在您可以在 RViz 中：'" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo '   1. Fixed Frame 切换到 \"map\"'" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo '   2. 使用 2D Pose Estimate 微調位置（可選）'" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo '   3. 使用 2D Goal Pose 設置導航目標'" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo ''" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo '📋 常用命令：'" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo '   檢查 AMCL: ros2 lifecycle get /amcl'" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo '   檢查 TF: ros2 run tf2_ros tf2_echo map odom'" C-m
tmux send-keys -t $SESSION_NAME:auto_init "echo '   話題列表: ros2 topic list'" C-m

# 回到第一個窗口
tmux select-window -t $SESSION_NAME:robot_base

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "✅ 所有組件啟動完成！"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "tmux 會話: $SESSION_NAME"
echo ""
echo "窗口列表："
echo "  robot_base   - 機器人基礎節點"
if [ "$SKIP_LIDAR" = false ]; then
    echo "  lidar        - 激光雷達"
fi
echo "  navigation   - 導航系統（包含 RViz）"
echo "  auto_init    - 自動初始化"
echo ""
echo "操作命令："
echo "  查看會話: tmux attach -t $SESSION_NAME"
echo "  切換窗口: Ctrl+B 然後按數字鍵 (0,1,2,3)"
echo "  退出查看: Ctrl+B 然後按 D"
echo "  關閉系統: tmux kill-session -t $SESSION_NAME"
echo ""
echo "═══════════════════════════════════════════════════════════════"
echo ""

read -p "是否現在連接到 tmux 會話查看？[Y/n] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Nn]$ ]]; then
    echo ""
    echo "正在連接到 tmux 會話..."
    echo "提示：按 Ctrl+B 然後按 D 可以退出查看（系統繼續在後台運行）"
    sleep 2
    tmux attach -t $SESSION_NAME
else
    echo ""
    echo "系統正在後台運行"
    echo "連接命令: tmux attach -t $SESSION_NAME"
fi

