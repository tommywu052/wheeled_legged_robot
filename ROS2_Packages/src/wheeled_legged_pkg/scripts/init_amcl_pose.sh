#!/bin/bash
# AMCL 初始位置快速设置脚本

echo "=========================================="
echo "AMCL 初始位置设置"
echo "=========================================="

echo ""
echo "1. 检查 AMCL 状态..."
STATE=$(ros2 lifecycle get /amcl 2>/dev/null)
if [[ "$STATE" != *"active"* ]]; then
    echo "   ❌ AMCL 未激活！"
    echo "   请先运行: ros2 lifecycle set /amcl configure && sleep 2 && ros2 lifecycle set /amcl activate"
    exit 1
fi
echo "   ✓ AMCL 已激活"

echo ""
echo "2. 检查订阅..."
INIT_SUBS=$(ros2 topic info /initialpose 2>/dev/null | grep "Subscription count:" | awk '{print $3}')
if [ "$INIT_SUBS" -eq 0 ]; then
    echo "   ❌ AMCL 没有订阅 /initialpose"
    exit 1
fi
echo "   ✓ AMCL 正在订阅 /initialpose"

echo ""
echo "3. 发送初始位置..."
echo ""
echo "⚠️  重要：初始位置必须在地图的有效区域（灰色可行走区域）内"
echo ""
echo "选项："
echo "  1) 使用地图原点 (0, 0) - 推荐，最可靠"
echo "  2) 使用地图中心"
echo "  3) 手动输入坐标"
echo ""
read -p "您的选择 [1/2/3]: " -n 1 -r
echo
echo ""

if [[ $REPLY == "1" ]] || [[ -z $REPLY ]]; then
    echo "使用地图原点 (0, 0)..."
    INIT_X=0.0
    INIT_Y=0.0
    INIT_YAW=0.0
elif [[ $REPLY == "2" ]]; then
    echo "正在计算地图中心..."
    # 获取地图信息
    MAP_INFO=$(timeout 3 ros2 topic echo /map --once 2>/dev/null | grep -A 15 "info:")
    if [ -n "$MAP_INFO" ]; then
        WIDTH=$(echo "$MAP_INFO" | grep "width:" | awk '{print $2}')
        HEIGHT=$(echo "$MAP_INFO" | grep "height:" | awk '{print $2}')
        RESOLUTION=$(echo "$MAP_INFO" | grep "resolution:" | awk '{print $2}')
        ORIGIN_X=$(echo "$MAP_INFO" | grep -A 3 "origin:" | grep "x:" | awk '{print $2}')
        ORIGIN_Y=$(echo "$MAP_INFO" | grep -A 4 "origin:" | grep "y:" | awk '{print $2}')
        
        # 计算中心
        INIT_X=$(echo "$ORIGIN_X + ($WIDTH * $RESOLUTION / 2)" | bc -l)
        INIT_Y=$(echo "$ORIGIN_Y + ($HEIGHT * $RESOLUTION / 2)" | bc -l)
        INIT_YAW=0.0
        
        echo "地图中心: ($INIT_X, $INIT_Y)"
    else
        echo "⚠️  无法获取地图信息，使用原点"
        INIT_X=0.0
        INIT_Y=0.0
        INIT_YAW=0.0
    fi
elif [[ $REPLY == "3" ]]; then
    echo "请输入坐标（在 RViz 中查看）："
    read -p "X 坐标: " INIT_X
    read -p "Y 坐标: " INIT_Y
    read -p "朝向 (度数, 0-360): " INIT_YAW_DEG
    # 转换为弧度
    INIT_YAW=$(echo "$INIT_YAW_DEG * 3.14159265359 / 180" | bc -l)
fi

# 计算四元数 (仅 Z 轴旋转)
QUAT_Z=$(echo "s($INIT_YAW / 2)" | bc -l)
QUAT_W=$(echo "c($INIT_YAW / 2)" | bc -l)

echo "发送初始位置: X=$INIT_X, Y=$INIT_Y, Yaw=$INIT_YAW"

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: $INIT_X, y: $INIT_Y, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: $QUAT_Z, w: $QUAT_W}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
  }
}" > /dev/null 2>&1

echo "   ✓ 初始位置已发送"

echo ""
echo "4. 等待 AMCL 初始化 (5 秒)..."
sleep 5

echo ""
echo "5. 验证粒子云..."
timeout 2 ros2 topic echo /particle_cloud --once > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "   ✓ 粒子云存在 - AMCL 已初始化！"
else
    echo "   ❌ 粒子云不存在 - AMCL 可能未初始化"
    echo "   请检查 navigation 终端的日志"
    exit 1
fi

echo ""
echo "6. 验证 map->odom TF..."
timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "At time"
if [ $? -eq 0 ]; then
    echo "   ✓ map->odom TF 存在！"
else
    echo "   ❌ map->odom TF 不存在"
    exit 1
fi

echo ""
echo "=========================================="
echo "✅ AMCL 初始化成功！"
echo ""
echo "在 RViz 中："
echo "  - Fixed Frame 下拉列表应该出现 'map'"
echo "  - 可以切换 Fixed Frame 为 'map'"
echo "  - 如果添加了 PoseArray，会看到绿色粒子云"
echo ""
echo "现在可以设置导航目标了："
echo "  1. 点击 '2D Goal Pose'"
echo "  2. 在地图上点击目标位置"
echo "  3. 拖动设置朝向"
echo "=========================================="

