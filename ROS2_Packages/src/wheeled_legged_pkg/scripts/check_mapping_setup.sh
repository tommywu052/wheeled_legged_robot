#!/bin/bash

# N10 Lidar + 轮足机器人建图系统检查脚本
# 用于验证所有必需的组件是否正确配置

echo "========================================"
echo "  N10 Lidar 建图系统配置检查"
echo "========================================"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_package() {
    local package=$1
    if ros2 pkg list | grep -q "^${package}$"; then
        echo -e "${GREEN}✓${NC} $package 已安装"
        return 0
    else
        echo -e "${RED}✗${NC} $package 未安装"
        return 1
    fi
}

check_topic() {
    local topic=$1
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC} 话题 $topic 存在"
        
        # 检查话题频率
        echo -n "  正在检查发布频率... "
        timeout 3s ros2 topic hz $topic 2>/dev/null | grep "average rate" && echo "" || echo -e "${YELLOW}无法获取频率${NC}"
        return 0
    else
        echo -e "${RED}✗${NC} 话题 $topic 不存在"
        return 1
    fi
}

check_node() {
    local node=$1
    if ros2 node list 2>/dev/null | grep -q "$node"; then
        echo -e "${GREEN}✓${NC} 节点 $node 正在运行"
        return 0
    else
        echo -e "${RED}✗${NC} 节点 $node 未运行"
        return 1
    fi
}

# 1. 检查ROS2环境
echo "1. 检查ROS2环境"
echo "----------------"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗${NC} ROS2环境未配置"
    echo "请运行: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo -e "${GREEN}✓${NC} ROS2版本: $ROS_DISTRO"
fi
echo ""

# 2. 检查必需的ROS2包
echo "2. 检查必需的ROS2包"
echo "-------------------"
check_package "slam_toolbox"
check_package "nav2_map_server"
check_package "lslidar_driver"
check_package "wheeled_legged_pkg"
echo ""

# 3. 检查必需的话题（如果有节点在运行）
echo "3. 检查必需的话题"
echo "----------------"
echo "提示：请确保以下节点正在运行："
echo "  - wl_base_node (机器人节点)"
echo "  - lslidar驱动节点"
echo ""

# 检查是否有节点在运行
if ros2 node list 2>/dev/null | grep -q "."; then
    check_topic "/odom"
    check_topic "/imu/data"
    check_topic "/scan"
    
    # 检查scan话题的frame_id
    echo ""
    echo "检查/scan话题的frame_id："
    timeout 2s ros2 topic echo /scan --once 2>/dev/null | grep "frame_id" | head -n 1
else
    echo -e "${YELLOW}⚠${NC} 没有ROS2节点在运行，跳过话题检查"
fi
echo ""

# 4. 检查TF树
echo "4. 检查TF变换树"
echo "--------------"
if ros2 node list 2>/dev/null | grep -q "."; then
    echo "检查关键TF变换："
    
    # 检查 odom -> base_link
    if timeout 2s ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -q "Translation"; then
        echo -e "${GREEN}✓${NC} odom -> base_link 变换存在"
    else
        echo -e "${RED}✗${NC} odom -> base_link 变换不存在"
    fi
    
    # 检查 base_link -> laser
    if timeout 2s ros2 run tf2_ros tf2_echo base_link laser 2>/dev/null | grep -q "Translation"; then
        echo -e "${GREEN}✓${NC} base_link -> laser 变换存在"
    else
        echo -e "${YELLOW}⚠${NC} base_link -> laser 变换不存在（需要手动发布静态TF）"
        echo "  运行: ros2 run tf2_ros static_transform_publisher 0.15 0 0.25 0 0 0 base_link laser"
    fi
else
    echo -e "${YELLOW}⚠${NC} 没有ROS2节点在运行，跳过TF检查"
fi
echo ""

# 5. 检查激光数据质量
echo "5. 检查激光雷达数据质量"
echo "---------------------"
if ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
    echo "采样激光数据..."
    timeout 2s ros2 topic echo /scan --once 2>/dev/null > /tmp/scan_data.txt
    
    if [ -s /tmp/scan_data.txt ]; then
        echo -e "${GREEN}✓${NC} 激光数据接收成功"
        
        # 提取关键信息
        echo ""
        echo "激光雷达参数："
        grep "angle_min\|angle_max\|range_min\|range_max" /tmp/scan_data.txt | head -n 4
        
        # 检查有效数据点
        valid_ranges=$(grep "ranges:" -A 10 /tmp/scan_data.txt | grep -o "[0-9]\+\.[0-9]\+" | wc -l)
        if [ $valid_ranges -gt 0 ]; then
            echo -e "${GREEN}✓${NC} 检测到 $valid_ranges 个有效数据点"
        else
            echo -e "${RED}✗${NC} 未检测到有效数据点"
        fi
    else
        echo -e "${RED}✗${NC} 无法接收激光数据"
    fi
    
    rm -f /tmp/scan_data.txt
else
    echo -e "${YELLOW}⚠${NC} /scan话题不存在，跳过数据检查"
fi
echo ""

# 6. 提供启动建议
echo "6. 系统状态总结"
echo "--------------"
echo ""

if ros2 node list 2>/dev/null | grep -q "."; then
    echo -e "${GREEN}系统正在运行${NC}"
    echo ""
    echo "当前运行的节点："
    ros2 node list 2>/dev/null
    echo ""
    
    # 检查是否可以启动SLAM
    has_odom=false
    has_scan=false
    has_tf=false
    
    ros2 topic list 2>/dev/null | grep -q "^/odom$" && has_odom=true
    ros2 topic list 2>/dev/null | grep -q "^/scan$" && has_scan=true
    timeout 1s ros2 run tf2_ros tf2_echo base_link laser 2>/dev/null | grep -q "Translation" && has_tf=true
    
    if $has_odom && $has_scan && $has_tf; then
        echo -e "${GREEN}✓ 所有必需组件就绪，可以启动SLAM！${NC}"
        echo ""
        echo "启动SLAM命令："
        echo "ros2 run slam_toolbox async_slam_toolbox_node --ros-args \\"
        echo "  -p mode:=asynchronous \\"
        echo "  -p map_frame:=map \\"
        echo "  -p odom_frame:=odom \\"
        echo "  -p base_frame:=base_link \\"
        echo "  -p scan_topic:=/scan"
    else
        echo -e "${YELLOW}⚠ 系统未完全就绪${NC}"
        echo ""
        echo "缺少的组件："
        $has_odom || echo "  - /odom 话题 (启动 wl_base_node)"
        $has_scan || echo "  - /scan 话题 (启动 lslidar驱动)"
        $has_tf || echo "  - base_link -> laser TF变换 (发布静态TF)"
    fi
else
    echo -e "${YELLOW}系统未启动${NC}"
    echo ""
    echo "启动步骤："
    echo ""
    echo "终端1 - 启动机器人节点："
    echo "  ros2 run wheeled_legged_pkg wl_base_node"
    echo ""
    echo "终端2 - 启动激光雷达："
    echo "  ros2 launch lslidar_driver lslidar_x10_launch.py"
    echo ""
    echo "终端3 - 发布静态TF："
    echo "  ros2 run tf2_ros static_transform_publisher 0.15 0 0.25 0 0 0 base_link laser"
    echo ""
    echo "终端4 - 启动SLAM："
    echo "  ros2 run slam_toolbox async_slam_toolbox_node --ros-args \\"
    echo "    -p mode:=asynchronous -p map_frame:=map -p odom_frame:=odom \\"
    echo "    -p base_frame:=base_link -p scan_topic:=/scan"
    echo ""
    echo "或使用Launch文件（推荐）："
    echo "  ros2 launch wheeled_legged_pkg mapping_with_lidar.launch.py"
fi

echo ""
echo "========================================"
echo "检查完成！"
echo "========================================"

