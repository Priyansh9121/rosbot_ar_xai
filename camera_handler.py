"""
图像处理模块
处理RGB图像、深度图像的接收和预处理
"""

import cv2
import numpy as np
import time
import threading
from threading import Lock
from collections import deque
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from config import Config

class CameraHandler:
    """摄像头图像处理器"""
    
    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        self.bridge = CvBridge()
        
        # 图像数据
        self.latest_frame = None
        self.latest_stereo_frame = None
        self.latest_depth_data = None
        self.frame_lock = Lock()
        
        # 处理控制
        self.frame_count = 0
        self.last_processed_time = 0
        self.use_compressed_image = False
        
        # 网络诊断
        self.image_receive_times = deque(maxlen=10)
        self.last_image_time = None
        self.network_delay_warning_shown = False
        
        # 延迟测量工具
        self.delay_measurement = {
            'image_receive': 0,
            'yolo_start': 0,
            'yolo_end': 0,
            'display_start': 0,
            'total_delay': 0
        }
        self.measuring_delay = True
        
        self._setup_subscriptions()
        self._start_diagnostic_thread()
    
    def _setup_subscriptions(self):
        """设置图像订阅"""
        image_qos = Config.get_image_qos()
        
        self.logger.info("🔍 Setting up subscriptions with low-latency QoS...")
        """      
        # 原始图像订阅作为主要方式
        self.image_sub = self.node.create_subscription(
            Image, '/oak/rgb/image_raw', self.image_callback, image_qos)
        """
        # 尝试压缩图像作为优化
        self.compressed_image_sub = self.node.create_subscription(
                CompressedImage, '/oak/rgb/image_raw/compressed', 
                self.compressed_image_callback, image_qos)
        self.use_compressed_image = True
        self.logger.info("✅ Using compressed RGB images for better performance")

        # 深度图像订阅
        self.depth_sub = self.node.create_subscription(
            Image, '/oak/stereo/image_raw', self.depth_callback, image_qos)
        
        self.compressed_depth_sub = self.node.create_subscription(
            CompressedImage, '/oak/rgb/image_raw/compressedDepth', 
            self.compressed_depth_callback, image_qos)
    
    def _start_diagnostic_thread(self):
        """启动网络诊断线程"""
        self.diagnostic_thread = threading.Thread(target=self._network_diagnostic_loop, daemon=True)
        self.diagnostic_thread.start()
    
    def _network_diagnostic_loop(self):
        """网络延迟诊断循环 - 针对大图像优化"""
        while True:  # 替代rclpy.ok()检查
            time.sleep(Config.NETWORK_CHECK_INTERVAL)
            
            if len(self.image_receive_times) >= 3:
                # 计算图像接收间隔
                intervals = []
                times = list(self.image_receive_times)
                for i in range(1, len(times)):
                    intervals.append(times[i] - times[i-1])
                
                avg_interval = sum(intervals) / len(intervals)
                max_interval = max(intervals)
                estimated_fps = 1.0 / avg_interval if avg_interval > 0 else 0
                
                self.logger.info(f"📊 Network Stats - FPS: {estimated_fps:.1f}, Avg: {avg_interval:.2f}s, Max: {max_interval:.2f}s")
                
                # 根据实际测量调整警告阈值
                if avg_interval > 0.5:  # 低于2 FPS
                    if not self.network_delay_warning_shown:
                        self.logger.warn("🐌 LOW FRAME RATE DETECTED!")
                        self.logger.warn("💡 Image size optimization suggestions:")
                        self.logger.warn("   - Use compressed image topics: /oak/rgb/image_raw/compressed")
                        self.logger.warn("   - Reduce camera resolution on robot")
                        self.logger.warn("   - Check: ros2 param set /oak width 640")
                        self.logger.warn("   - Check: ros2 param set /oak height 480")
                        self.logger.warn(f"   - Current estimated FPS: {estimated_fps:.1f}")
                        self.network_delay_warning_shown = True
                elif estimated_fps > 8:
                    self.logger.info("✅ Good frame rate performance!")
            
            # 检查是否长时间没有收到图像
            if self.last_image_time:
                time_since_last = time.time() - self.last_image_time
                if time_since_last > Config.IMAGE_TIMEOUT:
                    if not self.network_delay_warning_shown:
                        self.logger.error("❌ No images received for 10+ seconds!")
                        self.logger.error("🔧 Check robot connection and topic names")
                        self.logger.info("   Run these commands to diagnose:")
                        self.logger.info("   ros2 topic list | grep image")
                        self.logger.info("   ros2 topic hz /oak/rgb/image_raw")
                        self.logger.info("   ros2 topic bw /oak/rgb/image_raw")
                        self.logger.info("   Compressed image received")
                        self.network_delay_warning_shown = True
                else:
                    self.network_delay_warning_shown = False  # Reset when images resume

    
    def compressed_image_callback(self, msg):
        """处理压缩RGB图像 - 带延迟测量"""
        current_time = time.time()
        
        # 延迟测量 - 记录图像接收时间
        if self.measuring_delay:
            self.delay_measurement['image_receive'] = current_time
        
        # 记录图像接收时间
        self.image_receive_times.append(current_time)
        self.last_image_time = current_time
        
        # 激进的处理频率控制 - 减少缓冲
        if current_time - self.last_processed_time < Config.PROCESSING_INTERVAL:
            return

        try:
            # 快速解压缩
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_frame is not None:
                cv_frame = self._resize_frame(cv_frame)
                
                # 立即更新最新帧，不等待
                with self.frame_lock:
                    self.latest_frame = cv_frame
                self.last_processed_time = current_time
                
                self._update_frame_stats(msg)
            
        except Exception as e:
            self.logger.error(f"Compressed image error: {e}")

    def image_callback(self, msg):
        """原始图像回调 - 激进优化版本"""
        # 如果正在使用压缩图像，完全跳过原始图像
        if self.use_compressed_image:
            return
            
        current_time = time.time()
        self.image_receive_times.append(current_time)
        self.last_image_time = current_time
        
        # 激进的处理控制
        if current_time - self.last_processed_time < Config.PROCESSING_INTERVAL:
            return

        try:
            cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_frame = self._resize_frame(cv_frame)
            
            with self.frame_lock:
                self.latest_frame = cv_frame
            self.last_processed_time = current_time
            
            self.frame_count += 1
                
        except Exception as e:
            self.logger.error(f"Image conversion error: {e}")

    def depth_callback(self, msg):
        """处理深度图像 (16UC1格式)"""
        try:
            depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            with self.frame_lock:
                self.latest_stereo_frame = depth_frame
        except Exception as e:
            self.logger.error(f"Depth image conversion error: {e}")

    def compressed_depth_callback(self, msg):
        """处理压缩深度数据"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            with self.frame_lock:
                self.latest_depth_data = depth_image
        except Exception as e:
            self.logger.error(f"Compressed depth conversion error: {e}")
    
    def _resize_frame(self, cv_frame):
        """调整图像大小以提高处理速度"""
        height, width = cv_frame.shape[:2]
        if width > Config.MAX_IMAGE_WIDTH or height > Config.MAX_IMAGE_HEIGHT:
            scale = min(Config.MAX_IMAGE_WIDTH/width, Config.MAX_IMAGE_HEIGHT/height)
            new_width = int(width * scale)
            new_height = int(height * scale)
            cv_frame = cv2.resize(cv_frame, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
            if self.frame_count == 1:
                self.logger.info(f"📏 Aggressively resized to {new_width}x{new_height} for speed")
        return cv_frame
    
    def _update_frame_stats(self, msg):
        """更新帧统计信息"""
        self.frame_count += 1
        if self.frame_count % Config.FRAME_REPORT_INTERVAL == 0:  # 每60帧报告一次
            compressed_size = len(msg.data) / (1024*1024)
            
            # 添加延迟报告
            if self.measuring_delay and self.delay_measurement['total_delay'] > 0:
                self.logger.info(f"📷 Frame {self.frame_count}, size: {compressed_size:.2f}MB")
                self.logger.info(f"⏱️  DELAY BREAKDOWN:")
                self.logger.info(f"   Network → Processing: {self.delay_measurement['total_delay']:.2f}s")
                
                # 简单估算：如果总延迟>>网络间隔，说明有缓冲问题
                if self.delay_measurement['total_delay'] > 2.0:
                    self.logger.warn(f"🚨 HIGH CUMULATIVE DELAY DETECTED: {self.delay_measurement['total_delay']:.2f}s")
                    self.logger.warn("🔧 This suggests system/ROS buffering issues, not just network!")
    
    def get_latest_frame(self):
        """
        Returns the latest RGB image frame as a copy, if available.
        Adds logging for diagnostics and access counting.
        """
        with self.frame_lock:
            if self.latest_frame is not None:
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    self.logger.info(f"📈 get_latest_frame: Served {self.frame_count} frames")
                self.logger.debug("🟢 get_latest_frame: Returning valid RGB frame")
                return self.latest_frame.copy()
            else:
                self.logger.warn("🔴 get_latest_frame: No RGB frame available yet")
                return None


    
    def get_latest_depth_data(self):
        """获取最新的深度数据"""
        with self.frame_lock:
            return self.latest_depth_data, self.latest_stereo_frame