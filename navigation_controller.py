
"""
导航控制模块 - 修复控制冲突版本
确保后退期间不被其他系统覆盖
"""

import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from config import Config

class NavigationController:
    """导航控制器 - 统一控制版本"""
    
    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        
        # 当前状态
        self.current_pose = None
        self.current_state = 'MOVING'
        self.stop_requested = False
        self.manual_override = False
        
        # 障碍物检测状态
        self.depth_too_close = False
        self.obstacle_close_time = None
        
        # 后退功能相关
        self.is_backing_up = False
        self.backup_start_pose = None
        self.backup_target_distance = Config.BACKWARD_DISTANCE
        self.backup_start_time = None
        self.backup_step_count = 0  # 新增：后退步数计数
        
        # 发布器和订阅器
        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = node.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        
        # 定时器
        self.timer = node.create_timer(0.1, self.main_loop)
        
        self.logger.info("✅ Navigation Controller initialized with unified control")
    
    def _odom_callback(self, msg):
        """里程计回调"""
        self.current_pose = msg.pose.pose
    
    def calculate_distance_traveled(self, start_pose, current_pose):
        """计算从起始位置到当前位置的距离"""
        if start_pose is None or current_pose is None:
            return 0.0
        
        dx = current_pose.position.x - start_pose.position.x
        dy = current_pose.position.y - start_pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def initiate_backup(self):
        """启动后退操作"""
        if self.current_pose is None:
            self.logger.warn("⚠️ No pose available, using step-based backup")
        
        self.is_backing_up = True
        self.backup_start_pose = self.current_pose
        self.backup_target_distance = Config.BACKWARD_DISTANCE
        self.backup_start_time = time.time()
        self.backup_step_count = 0  # 重置步数计数
        
        self.logger.warn(f"🔙 BACKUP INITIATED - target: {Config.BACKWARD_DISTANCE:.2f}m")
        self.logger.info(f"🎯 Control priority: BACKUP MODE")
    
    def update_backup_status(self):
        """更新后退状态"""
        if not self.is_backing_up:
            return
        
        current_time = time.time()
        self.backup_step_count += 1
        
        # 检查超时
        if current_time - self.backup_start_time > Config.BACKUP_TIMEOUT:
            self.logger.warn("⚠️ Backup timeout reached, stopping backup")
            self._finish_backup()
            return
        
        # 检查是否已经后退了足够的距离
        if self.current_pose and self.backup_start_pose:
            distance_traveled = self.calculate_distance_traveled(self.backup_start_pose, self.current_pose)
            
            if distance_traveled >= self.backup_target_distance:
                self.logger.info(f"✅ Backup completed - traveled {distance_traveled:.2f}m")
                self._finish_backup()
                return
        else:
            # 如果没有里程计，使用步数估算
            # 假设每步0.1秒，速度0.2m/s，所以每步约0.02m
            estimated_distance = self.backup_step_count * 0.02
            
            if estimated_distance >= self.backup_target_distance:
                self.logger.info(f"✅ Backup completed - estimated {estimated_distance:.2f}m ({self.backup_step_count} steps)")
                self._finish_backup()
                return
        
        # 继续后退
        remaining_time = Config.BACKUP_TIMEOUT - (current_time - self.backup_start_time)
        self.logger.info(f"🔙 Backing up... step {self.backup_step_count}, {remaining_time:.1f}s remaining")
    
    def _finish_backup(self):
        """完成后退操作"""
        self.is_backing_up = False
        self.backup_start_pose = None
        self.backup_step_count = 0
        self.logger.info(f"✅ Backup finished, returning to normal navigation")
        self.logger.info(f"🎯 Control priority: NAVIGATION MODE")
    
    def cancel_backup(self):
        """取消后退操作"""
        if self.is_backing_up:
            self.logger.info("❌ Backup cancelled manually")
            self._finish_backup()
        else:
            self.logger.info("ℹ️ Not currently backing up")
    
    def force_backup(self):
        """手动触发后退"""
        if not self.is_backing_up:
            self.logger.info("🔙 Manual backup triggered")
            self.initiate_backup()
        else:
            self.logger.info("⚠️ Already backing up")
    
    def yaw_from_quat(self, q):
        """从四元数计算偏航角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def send_manual_command(self, linear_x=0.0, angular_z=0.0):
        """发送手动控制命令"""
        twist = Twist()
        twist.linear.x = Config.MANUAL_LINEAR_MULTIPLIER * linear_x
        twist.angular.z = Config.MANUAL_ANGULAR_MULTIPLIER * angular_z
        self.manual_override = True
        self.cmd_pub.publish(twist)
        self.logger.info(f"🎮 Manual command: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")
    
    def stop_robot(self):
        """停止机器人"""
        self.stop_requested = True
        self.cancel_backup()  # 停止时也取消后退
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.logger.info("🛑 Robot stop requested")
    
    def resume_navigation(self):
        """恢复自动导航"""
        self.stop_requested = False
        self.manual_override = False
        self.logger.info("▶️ Navigation resumed")
    
    def update_obstacle_status(self, obstacle_detected):
        """更新障碍物检测状态"""
        if obstacle_detected:
            if self.obstacle_close_time is None:
                self.obstacle_close_time = time.time()
            elif time.time() - self.obstacle_close_time >= Config.DEPTH_HOLD_TIME:
                if not self.depth_too_close:
                    self.logger.warn(f"🛑 Obstacle too close for {Config.DEPTH_HOLD_TIME}s")
                    self.depth_too_close = True
        else:
            if self.obstacle_close_time:
                self.logger.info("✅ Obstacle cleared")
            self.obstacle_close_time = None
            self.depth_too_close = False
    
    def main_loop(self):
        """主导航循环 - 统一控制版本"""
        
        # 🔥 关键修复：如果手动控制激活，跳过自动控制
        if self.manual_override:
            return
            
        # 🔥 关键修复：如果正在后退，优先执行后退逻辑
        if self.is_backing_up:
            self.update_backup_status()
            
            if self.is_backing_up:  # 检查是否仍在后退（可能在update中结束了）
                twist = Twist()
                twist.linear.x = Config.BACKWARD_SPEED
                twist.angular.z = 0.0
                self.current_state = 'BACKING_UP'
                self.cmd_pub.publish(twist)
                return  # 🔥 重要：直接返回，不执行正常导航逻辑
        
        # 正常导航逻辑
        twist = Twist()

        if self.stop_requested:
            self.current_state = 'STOPPED'
            self.cmd_pub.publish(twist)
            return

        # 导航逻辑
        if self.current_pose:
            dx = Config.GOAL_X - self.current_pose.position.x
            dy = Config.GOAL_Y - self.current_pose.position.y
            dist = math.hypot(dx, dy)
            yaw = self.yaw_from_quat(self.current_pose.orientation)
            goal_angle = math.atan2(dy, dx)
            angle_diff = math.atan2(math.sin(goal_angle - yaw), math.cos(goal_angle - yaw))

            if dist < Config.GOAL_TOLERANCE:
                self.logger.info("🎯 Goal reached")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.current_state = 'IDLE'
            elif self.depth_too_close:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.current_state = 'STOPPED'
            elif abs(angle_diff) > Config.TURN_THRESHOLD:
                twist.linear.x = 0.0
                twist.angular.z = Config.ANGULAR_SPEED * angle_diff
                self.current_state = 'TURNING'
            else:
                twist.linear.x = Config.LINEAR_SPEED
                twist.angular.z = 0.0
                self.current_state = 'MOVING'

        self.cmd_pub.publish(twist)
    
    def get_current_state(self):
        """获取当前导航状态"""
        return self.current_state
    
    def set_goal(self, x, y):
        """设置新的目标点"""
        Config.GOAL_X = x
        Config.GOAL_Y = y
        self.logger.info(f"🎯 New goal set: ({x:.2f}, {y:.2f})")
    
    def update_backup_distance(self, distance):
        """更新后退距离"""
        if 0.1 <= distance <= 2.0:
            Config.BACKWARD_DISTANCE = distance
            self.logger.info(f"🔧 Backup distance updated to {distance:.2f}m")
            return True
        else:
            self.logger.warn("⚠️ Backup distance must be between 0.1 and 2.0 meters")
            return False
    
    def update_trigger_distance(self, distance):
        """更新触发距离"""
        if 0.3 <= distance <= 2.0:
            Config.BACKWARD_TRIGGER_DISTANCE = distance
            self.logger.info(f"🔧 Trigger distance updated to {distance:.2f}m")
            return True
        else:
            self.logger.warn("⚠️ Trigger distance must be between 0.3 and 2.0 meters")
            return False
    
    def get_control_status(self):
        """获取控制状态信息"""
        status = {
            'current_state': self.current_state,
            'is_backing_up': self.is_backing_up,
            'manual_override': self.manual_override,
            'stop_requested': self.stop_requested,
            'backup_step_count': self.backup_step_count if self.is_backing_up else 0
        }
        return status
