"""
传感器融合模块 - 新建文件
整合激光雷达、超声波、YOLO和深度相机数据
"""

import math
import time
import numpy as np
from threading import Lock
from collections import deque
from sensor_msgs.msg import LaserScan, Range
from config import Config

class SensorFusion:
    """传感器融合系统"""
    
    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        
        # 传感器数据
        self.laser_data = None
        self.range_sensors = {'fl': None, 'fr': None, 'rl': None, 'rr': None}
        self.fusion_lock = Lock()
        
        # 融合结果
        self.current_obstacle_info = {
            'min_distance': float('inf'),
            'min_angle': 0,
            'confidence': 0.0,
            'sensor_sources': [],
            'emergency_stop': False
        }
        
        # 订阅传感器数据
        self._setup_subscriptions()
        
        # 融合处理定时器
        self.fusion_timer = node.create_timer(0.1, self._fusion_loop)
        
        self.logger.info("✅ Sensor fusion system initialized")
    
    def _setup_subscriptions(self):
        """设置传感器订阅"""
        # 激光雷达
        self.laser_sub = self.node.create_subscription(
            LaserScan, '/scan', self._laser_callback, 10)
        
        # 超声波传感器
        range_topics = ['/range/fl', '/range/fr', '/range/rl', '/range/rr']
        for topic in range_topics:
            sensor_name = topic.split('/')[-1]
            self.node.create_subscription(
                Range, topic, 
                lambda msg, name=sensor_name: self._range_callback(msg, name), 10)
        
        self.logger.info("📡 Subscribed to: LiDAR + 4x Ultrasonic sensors")
    
    def _laser_callback(self, msg):
        """激光雷达数据回调"""
        with self.fusion_lock:
            self.laser_data = msg
    
    def _range_callback(self, msg, sensor_name):
        """超声波传感器回调"""
        with self.fusion_lock:
            self.range_sensors[sensor_name] = msg
    
    def _fusion_loop(self):
        """传感器融合主循环"""
        try:
            obstacle_info = self._analyze_sensors()
            
            with self.fusion_lock:
                self.current_obstacle_info = obstacle_info
                
        except Exception as e:
            self.logger.error(f"Fusion loop error: {e}")
    
    def _analyze_sensors(self):
        """分析传感器数据"""
        obstacle_info = {
            'min_distance': float('inf'),
            'min_angle': 0,
            'confidence': 0.0,
            'sensor_sources': [],
            'emergency_stop': False
        }
        
        with self.fusion_lock:
            # 分析激光雷达
            laser_result = self._analyze_laser()
            if laser_result:
                obstacle_info.update(laser_result)
                obstacle_info['sensor_sources'].append('lidar')
            
            # 分析超声波传感器
            range_result = self._analyze_ultrasonic()
            if range_result:
                if range_result['min_distance'] < obstacle_info['min_distance']:
                    obstacle_info['min_distance'] = range_result['min_distance']
                    obstacle_info['min_angle'] = range_result['min_angle']
                obstacle_info['confidence'] += 0.2
                obstacle_info['sensor_sources'].append('ultrasonic')
        
        # 最终判断
        obstacle_info['emergency_stop'] = obstacle_info['min_distance'] < 0.3
        obstacle_info['confidence'] = min(1.0, obstacle_info['confidence'])
        
        return obstacle_info
    
    def _analyze_laser(self):
        """分析激光雷达数据"""
        if not self.laser_data:
            return None
        
        ranges = np.array(self.laser_data.ranges)
        angles = np.linspace(self.laser_data.angle_min, 
                           self.laser_data.angle_max, len(ranges))
        
        # 过滤有效数据
        valid_mask = (np.isfinite(ranges) & 
                     (ranges > self.laser_data.range_min) & 
                     (ranges < self.laser_data.range_max))
        
        if not np.any(valid_mask):
            return None
        
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # 重点关注前方区域 (±60度)
        front_mask = np.abs(valid_angles) < np.pi/3
        if np.any(front_mask):
            front_ranges = valid_ranges[front_mask]
            front_angles = valid_angles[front_mask]
            
            min_idx = np.argmin(front_ranges)
            return {
                'min_distance': float(front_ranges[min_idx]),
                'min_angle': float(front_angles[min_idx]),
                'confidence': 0.5
            }
        
        return None
    
    def _analyze_ultrasonic(self):
        """分析超声波传感器数据"""
        front_sensors = ['fl', 'fr']
        min_distance = float('inf')
        min_angle = 0
        
        for sensor_name in front_sensors:
            sensor_data = self.range_sensors[sensor_name]
            if (sensor_data and 
                sensor_data.range < sensor_data.max_range and 
                sensor_data.range > sensor_data.min_range):
                
                if sensor_data.range < min_distance:
                    min_distance = sensor_data.range
                    min_angle = 0.3 if sensor_name == 'fl' else -0.3
        
        if min_distance < float('inf'):
            return {
                'min_distance': min_distance,
                'min_angle': min_angle,
                'confidence': 0.3
            }
        
        return None
    
    def get_obstacle_info(self):
        """获取当前障碍物信息"""
        with self.fusion_lock:
            return self.current_obstacle_info.copy()
    
    def update_yolo_detection(self, yolo_distance, yolo_confidence):
        """更新YOLO检测结果"""
        if yolo_distance < 2.0 and yolo_confidence > 0.5:
            with self.fusion_lock:
                # 如果YOLO检测与其他传感器一致，提升置信度
                if abs(yolo_distance - self.current_obstacle_info['min_distance']) < 0.5:
                    self.current_obstacle_info['confidence'] += 0.2
                    if 'yolo' not in self.current_obstacle_info['sensor_sources']:
                        self.current_obstacle_info['sensor_sources'].append('yolo')