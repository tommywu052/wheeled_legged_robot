#!/usr/bin/env python3
"""
自动初始化 AMCL 节点
等待 AMCL 激活后，自动发布初始位置
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
import time


class AutoInitAMCL(Node):
    def __init__(self):
        super().__init__('auto_init_amcl')
        
        # 声明参数
        self.declare_parameter('wait_time', 10.0)  # 等待时间（秒）
        self.declare_parameter('init_x', 0.0)      # 初始X坐标
        self.declare_parameter('init_y', 0.0)      # 初始Y坐标
        self.declare_parameter('init_yaw', 0.0)    # 初始朝向（弧度）
        
        # 获取参数
        self.wait_time = self.get_parameter('wait_time').value
        self.init_x = self.get_parameter('init_x').value
        self.init_y = self.get_parameter('init_y').value
        self.init_yaw = self.get_parameter('init_yaw').value
        
        # 创建发布者
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos
        )
        
        # 创建服务客户端检查 AMCL 状态
        self.amcl_state_client = self.create_client(
            GetState,
            '/amcl/get_state'
        )
        
        self.get_logger().info('🚀 自动初始化 AMCL 节点已启动')
        self.get_logger().info(f'   等待时间: {self.wait_time}秒')
        self.get_logger().info(f'   初始位置: ({self.init_x}, {self.init_y}), yaw={self.init_yaw}')
        
        # 创建定时器
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.start_time = time.time()
        self.initialized = False
        self.amcl_ready = False

    def check_amcl_state(self):
        """检查 AMCL 是否已激活"""
        if not self.amcl_state_client.wait_for_service(timeout_sec=1.0):
            return False
        
        request = GetState.Request()
        future = self.amcl_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            state = future.result().current_state.label
            self.get_logger().info(f'AMCL 状态: {state}')
            return state == 'active'
        return False

    def publish_initial_pose(self):
        """发布初始位置"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # 位置
        msg.pose.pose.position.x = self.init_x
        msg.pose.pose.position.y = self.init_y
        msg.pose.pose.position.z = 0.0
        
        # 姿态（四元数）
        import math
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.init_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.init_yaw / 2.0)
        
        # 协方差矩阵
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787
        ]
        
        self.pose_pub.publish(msg)
        self.get_logger().info('✅ 已发布初始位置到 AMCL')
        self.get_logger().info(f'   位置: ({self.init_x}, {self.init_y}, {self.init_yaw} rad)')

    def timer_callback(self):
        """定时器回调"""
        if self.initialized:
            return
        
        elapsed = time.time() - self.start_time
        
        # 检查是否超过等待时间
        if elapsed < self.wait_time:
            remaining = self.wait_time - elapsed
            if int(remaining) % 5 == 0 and remaining - int(remaining) < 1.0:
                self.get_logger().info(f'⏳ 等待 {int(remaining)} 秒后初始化 AMCL...')
            return
        
        # 检查 AMCL 是否准备好
        if not self.amcl_ready:
            self.get_logger().info('🔍 检查 AMCL 状态...')
            self.amcl_ready = self.check_amcl_state()
            if not self.amcl_ready:
                self.get_logger().warn('⚠️  AMCL 尚未激活，等待1秒后重试...')
                return
        
        # 发布初始位置
        self.get_logger().info('📍 正在发布初始位置...')
        self.publish_initial_pose()
        
        # 标记为已初始化
        self.initialized = True
        
        # 再发布几次确保收到
        for i in range(3):
            time.sleep(0.5)
            self.publish_initial_pose()
        
        self.get_logger().info('🎉 AMCL 初始化完成！')
        self.get_logger().info('   您现在可以在 RViz 中设置导航目标了。')
        
        # 停止定时器
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = AutoInitAMCL()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

