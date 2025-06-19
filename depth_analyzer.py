#!/usr/bin/env python3
"""
深度数据分析工具
帮助诊断深度数据格式和单位问题
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class DepthAnalyzer(Node):
    """深度数据分析器"""
    
    def __init__(self):
        super().__init__('depth_analyzer')
        self.bridge = CvBridge()
        
        # 订阅深度相关话题
        self.depth_sub = self.create_subscription(
            Image, '/oak/stereo/image_raw', self.depth_callback, 10)
        
        self.compressed_depth_sub = self.create_subscription(
            CompressedImage, '/oak/rgb/image_raw/compressedDepth', 
            self.compressed_depth_callback, 10)
        
        self.analysis_count = 0
        self.last_analysis = 0
        
        print("🔍 Depth Analyzer started")
        print("📡 Listening to depth topics...")
        
    def depth_callback(self, msg):
        """原始深度图像回调"""
        current_time = time.time()
        if current_time - self.last_analysis < 2.0:  # 每2秒分析一次
            return
            
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.analyze_depth_data(depth_image, "Raw Depth Image")
            self.last_analysis = current_time
            
        except Exception as e:
            self.get_logger().error(f"Raw depth analysis error: {e}")
    
    def compressed_depth_callback(self, msg):
        """压缩深度图像回调"""
        current_time = time.time()
        if current_time - self.last_analysis < 2.0:
            return
            
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            
            if depth_image is not None:
                self.analyze_depth_data(depth_image, "Compressed Depth Image")
                self.last_analysis = current_time
                
        except Exception as e:
            self.get_logger().error(f"Compressed depth analysis error: {e}")
    
    def analyze_depth_data(self, depth_data, source):
        """详细分析深度数据"""
        self.analysis_count += 1
        
        print(f"\n{'='*60}")
        print(f"📊 DEPTH ANALYSIS #{self.analysis_count} - {source}")
        print(f"{'='*60}")
        
        # 基本信息
        print(f"🔍 Basic Info:")
        print(f"  Shape: {depth_data.shape}")
        print(f"  Data type: {depth_data.dtype}")
        print(f"  Memory size: {depth_data.nbytes / 1024:.1f} KB")
        
        # 数值范围分析
        print(f"\n📈 Value Range Analysis:")
        print(f"  Min value: {np.min(depth_data)}")
        print(f"  Max value: {np.max(depth_data)}")
        print(f"  Mean value: {np.mean(depth_data):.2f}")
        print(f"  Std deviation: {np.std(depth_data):.2f}")
        
        # 零值分析
        zero_count = np.sum(depth_data == 0)
        zero_ratio = zero_count / depth_data.size
        print(f"  Zero pixels: {zero_count} ({zero_ratio:.1%})")
        
        # 数值分布分析
        non_zero_data = depth_data[depth_data > 0]
        if len(non_zero_data) > 0:
            print(f"\n📊 Non-zero Value Distribution:")
            percentiles = [1, 5, 25, 50, 75, 95, 99]
            for p in percentiles:
                val = np.percentile(non_zero_data, p)
                print(f"  {p:2d}th percentile: {val:.0f}")
        
        # 可能的单位分析
        print(f"\n🎯 Possible Unit Analysis:")
        if len(non_zero_data) > 0:
            median_val = np.median(non_zero_data)
            
            # 假设典型场景：物体在1-5米距离
            print(f"  If data is in millimeters:")
            print(f"    Median distance: {median_val/1000:.2f}m")
            print(f"    Range: {np.min(non_zero_data)/1000:.2f}m - {np.max(non_zero_data)/1000:.2f}m")
            
            print(f"  If data is in centimeters:")
            print(f"    Median distance: {median_val/100:.2f}m")
            print(f"    Range: {np.min(non_zero_data)/100:.2f}m - {np.max(non_zero_data)/100:.2f}m")
            
            print(f"  If data is already in meters:")
            print(f"    Median distance: {median_val:.2f}m")
            print(f"    Range: {np.min(non_zero_data):.2f}m - {np.max(non_zero_data):.2f}m")
        
        # 采样区域分析（模拟物体检测框）
        self.analyze_sample_regions(depth_data)
        
        # 可视化直方图
        self.create_histogram_analysis(non_zero_data if len(non_zero_data) > 0 else depth_data)
        
        print(f"{'='*60}\n")
    
    def analyze_sample_regions(self, depth_data):
        """分析样本区域（模拟物体检测）"""
        print(f"\n🎯 Sample Region Analysis:")
        
        h, w = depth_data.shape[:2]
        
        # 定义几个测试区域
        regions = [
            ("Center", w//2-50, h//2-50, w//2+50, h//2+50),
            ("Left", w//4-25, h//2-25, w//4+25, h//2+25),
            ("Right", 3*w//4-25, h//2-25, 3*w//4+25, h//2+25),
        ]
        
        for name, x1, y1, x2, y2 in regions:
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            
            if x2 > x1 and y2 > y1:
                roi = depth_data[y1:y2, x1:x2]
                valid_pixels = roi[roi > 0]
                
                if len(valid_pixels) > 0:
                    median_depth = np.median(valid_pixels)
                    print(f"  {name:8s}: {median_depth:6.0f} raw → ", end="")
                    print(f"{median_depth/1000:5.2f}m(mm) ", end="")
                    print(f"{median_depth/100:5.2f}m(cm) ", end="")
                    print(f"{median_depth:5.2f}m(m)")
                else:
                    print(f"  {name:8s}: No valid data")
    
    def create_histogram_analysis(self, data):
        """创建直方图分析"""
        if len(data) == 0:
            return
            
        print(f"\n📊 Histogram Analysis:")
        
        # 创建简单的ASCII直方图
        hist, bin_edges = np.histogram(data, bins=10)
        max_count = np.max(hist)
        
        print(f"  Value Range → Count (ASCII bar)")
        for i in range(len(hist)):
            range_str = f"{bin_edges[i]:6.0f}-{bin_edges[i+1]:6.0f}"
            bar_length = int(20 * hist[i] / max_count) if max_count > 0 else 0
            bar = "█" * bar_length
            print(f"  {range_str}: {hist[i]:6d} {bar}")

def main():
    """主函数"""
    print("🔍 Starting Depth Data Analyzer...")
    print("This tool will help diagnose depth data format issues")
    print("Press Ctrl+C to stop")
    
    rclpy.init()
    
    try:
        analyzer = DepthAnalyzer()
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        print("\n🛑 Analysis stopped by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()