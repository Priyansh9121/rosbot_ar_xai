"""
距离计算问题诊断与修复
解决距离严重不准确的问题
"""

import cv2
import numpy as np
from collections import deque
from config import Config

class DebugDepthCalculator:
    """调试版距离计算器 - 解决距离不准确问题"""
    
    def __init__(self, logger):
        self.logger = logger
        self.distance_history = {}
        
        # 立体视觉匹配器
        self.stereo_matcher = cv2.StereoBM_create(
            numDisparities=96,
            blockSize=21
        )
        
        # 启用详细调试
        self.debug_mode = True
        self.frame_count = 0
        
        # 深度数据分析缓存
        self.depth_analysis_cache = {}
        
        # 已知物体的真实尺寸（用于距离验证）
        self.object_real_sizes = {
            'person': {'width': 0.5, 'height': 1.7},
            'chair': {'width': 0.5, 'height': 0.9},
            'bottle': {'width': 0.08, 'height': 0.25},
            'cup': {'width': 0.08, 'height': 0.12},
            'backpack': {'width': 0.35, 'height': 0.45},
            'book': {'width': 0.15, 'height': 0.25},
            'cell phone': {'width': 0.07, 'height': 0.15}
        }
    
    def analyze_depth_data_quality(self, depth_data, label="unknown"):
        """分析深度数据质量"""
        if depth_data is None:
            return {"quality": "no_data", "info": "No depth data available"}
        
        # 基本统计
        valid_pixels = depth_data[(depth_data > 0) & (depth_data < 10000)]
        
        if len(valid_pixels) == 0:
            return {"quality": "invalid", "info": "No valid depth pixels"}
        
        stats = {
            "min_depth": np.min(valid_pixels),
            "max_depth": np.max(valid_pixels),
            "mean_depth": np.mean(valid_pixels),
            "std_depth": np.std(valid_pixels),
            "valid_pixel_ratio": len(valid_pixels) / depth_data.size,
            "data_type": str(depth_data.dtype),
            "data_range": f"{np.min(depth_data)} - {np.max(depth_data)}"
        }
        
        # 判断数据质量
        if stats["std_depth"] > 1000:
            quality = "noisy"
        elif stats["valid_pixel_ratio"] < 0.1:
            quality = "sparse" 
        elif stats["min_depth"] == stats["max_depth"]:
            quality = "constant"
        else:
            quality = "good"
        
        self.logger.info(f"🔍 Depth Quality Analysis for {label}:")
        self.logger.info(f"  Quality: {quality}")
        self.logger.info(f"  Valid pixels: {stats['valid_pixel_ratio']:.2%}")
        self.logger.info(f"  Depth range: {stats['min_depth']:.0f} - {stats['max_depth']:.0f}")
        self.logger.info(f"  Mean±Std: {stats['mean_depth']:.0f}±{stats['std_depth']:.0f}")
        self.logger.info(f"  Data type: {stats['data_type']}")
        
        return {"quality": quality, "stats": stats}
    
    def get_distance_from_compressed_depth(self, depth_data, x1, y1, x2, y2):
        """增强版深度距离计算 - 多种方法验证"""
        if depth_data is None:
            return None
        
        self.frame_count += 1
        
        try:
            h, w = depth_data.shape[:2]
            
            # 确保边界框有效
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            
            if x2 <= x1 or y2 <= y1:
                return None
            
            # 提取ROI区域
            roi = depth_data[y1:y2, x1:x2]
            
            if self.debug_mode and self.frame_count % 30 == 0:  # 每30帧分析一次
                self.analyze_depth_data_quality(roi, "ROI")
            
            # 尝试多种深度解析方法
            distances = []
            methods = []
            
            # 方法1：原始数据直接转换（毫米到米）
            dist1 = self._method_direct_mm_conversion(roi)
            if dist1:
                distances.append(dist1)
                methods.append("direct_mm")
            
            # 方法2：数据可能是厘米
            dist2 = self._method_cm_conversion(roi)
            if dist2:
                distances.append(dist2)
                methods.append("cm_conversion")
            
            # 方法3：数据可能已经是米
            dist3 = self._method_direct_meters(roi)
            if dist3:
                distances.append(dist3)
                methods.append("direct_meters")
            
            # 方法4：16位深度特殊处理
            dist4 = self._method_16bit_depth(roi)
            if dist4:
                distances.append(dist4)
                methods.append("16bit_depth")
            
            # 选择最合理的距离
            best_distance = self._select_most_reasonable_distance(distances, methods, x2-x1, y2-y1)
            
            if self.debug_mode and best_distance:
                self.logger.info(f"📏 Distance methods: {dict(zip(methods, distances))}")
                self.logger.info(f"✅ Selected: {best_distance:.2f}m (method: {methods[distances.index(best_distance)] if best_distance in distances else 'filtered'})")
            
            return best_distance
            
        except Exception as e:
            self.logger.error(f"Enhanced depth calculation error: {e}")
            return None
    
    def _method_direct_mm_conversion(self, roi):
        """方法1：假设数据单位是毫米"""
        valid_depths = roi[(roi > 100) & (roi < 15000)]  # 10cm - 15m 范围
        
        if len(valid_depths) > 10:
            # 使用25%分位数避免噪声
            distance_mm = np.percentile(valid_depths, 25)
            distance_m = distance_mm / 1000.0
            
            if 0.2 <= distance_m <= 15.0:
                return distance_m
        return None
    
    def _method_cm_conversion(self, roi):
        """方法2：假设数据单位是厘米"""
        valid_depths = roi[(roi > 10) & (roi < 1500)]  # 10cm - 15m 范围
        
        if len(valid_depths) > 10:
            distance_cm = np.percentile(valid_depths, 25)
            distance_m = distance_cm / 100.0
            
            if 0.2 <= distance_m <= 15.0:
                return distance_m
        return None
    
    def _method_direct_meters(self, roi):
        """方法3：假设数据已经是米"""
        # 对于浮点数深度图
        if roi.dtype in [np.float32, np.float64]:
            valid_depths = roi[(roi > 0.2) & (roi < 15.0)]
            
            if len(valid_depths) > 10:
                distance_m = np.percentile(valid_depths, 25)
                
                if 0.2 <= distance_m <= 15.0:
                    return distance_m
        return None
    
    def _method_16bit_depth(self, roi):
        """方法4：16位深度特殊处理"""
        if roi.dtype == np.uint16:
            # 某些16位深度图有特殊的缩放因子
            valid_depths = roi[(roi > 0) & (roi < 65535)]
            
            if len(valid_depths) > 10:
                raw_depth = np.percentile(valid_depths, 25)
                
                # 尝试常见的缩放因子
                scale_factors = [0.001, 0.01, 0.1, 1.0]  # mm, cm, dm, m
                
                for scale in scale_factors:
                    distance_m = raw_depth * scale
                    if 0.2 <= distance_m <= 15.0:
                        return distance_m
        return None
    
    def _select_most_reasonable_distance(self, distances, methods, pixel_width, pixel_height):
        """选择最合理的距离值"""
        if not distances:
            return None
        
        if len(distances) == 1:
            return distances[0]
        
        # 过滤明显不合理的距离
        reasonable_distances = []
        reasonable_methods = []
        
        for i, dist in enumerate(distances):
            # 基本合理性检查
            if 0.3 <= dist <= 12.0:
                reasonable_distances.append(dist)
                reasonable_methods.append(methods[i])
        
        if not reasonable_distances:
            return None
        
        if len(reasonable_distances) == 1:
            return reasonable_distances[0]
        
        # 多个合理距离，选择最一致的
        distances_array = np.array(reasonable_distances)
        
        # 如果距离差异小，取中位数
        if np.std(distances_array) < 0.5:
            return np.median(distances_array)
        
        # 如果差异大，优先选择中等距离（避免极值）
        sorted_distances = sorted(reasonable_distances)
        
        # 去掉最大和最小值，选择中间的
        if len(sorted_distances) >= 3:
            middle_distances = sorted_distances[1:-1]
            return np.mean(middle_distances)
        else:
            return np.median(sorted_distances)
    
    def validate_distance_with_object_size(self, distance, pixel_width, pixel_height, label):
        """基于物体真实尺寸验证距离"""
        if label not in self.object_real_sizes or distance is None:
            return distance
        
        # 相机内参（需要根据实际相机调整）
        focal_length_pixels = 525.0  # 这个值需要校准
        
        # 计算在当前距离下物体应该有的像素尺寸
        real_width = self.object_real_sizes[label]['width']
        real_height = self.object_real_sizes[label]['height']
        
        expected_pixel_width = (real_width * focal_length_pixels) / distance
        expected_pixel_height = (real_height * focal_length_pixels) / distance
        
        # 计算尺寸匹配程度
        width_ratio = pixel_width / expected_pixel_width
        height_ratio = pixel_height / expected_pixel_height
        
        # 允许的尺寸误差范围（考虑视角、物体变化等）
        min_ratio, max_ratio = 0.3, 3.0
        
        width_reasonable = min_ratio <= width_ratio <= max_ratio
        height_reasonable = min_ratio <= height_ratio <= max_ratio
        
        if self.debug_mode:
            self.logger.info(f"🔍 Size validation for {label} at {distance:.2f}m:")
            self.logger.info(f"  Expected: {expected_pixel_width:.0f}x{expected_pixel_height:.0f} px")
            self.logger.info(f"  Actual: {pixel_width}x{pixel_height} px")
            self.logger.info(f"  Ratios: W={width_ratio:.2f}, H={height_ratio:.2f}")
            self.logger.info(f"  Reasonable: W={width_reasonable}, H={height_reasonable}")
        
        # 如果尺寸不合理，尝试基于尺寸重新估算距离
        if not (width_reasonable and height_reasonable):
            # 基于宽度重新估算
            corrected_distance_w = (real_width * focal_length_pixels) / pixel_width
            # 基于高度重新估算  
            corrected_distance_h = (real_height * focal_length_pixels) / pixel_height
            
            # 选择更保守（更远）的距离
            corrected_distance = max(corrected_distance_w, corrected_distance_h)
            
            # 确保修正后的距离在合理范围内
            corrected_distance = max(0.3, min(15.0, corrected_distance))
            
            if self.debug_mode:
                self.logger.warn(f"🔧 Distance corrected: {distance:.2f}m → {corrected_distance:.2f}m")
            
            return corrected_distance
        
        return distance
    
    def get_object_distance(self, depth_data, stereo_frame, x1, y1, x2, y2, label):
        """主要距离计算接口 - 增强调试版"""
        
        # 计算像素尺寸
        pixel_width = x2 - x1
        pixel_height = y2 - y1
        
        if self.debug_mode:
            self.logger.info(f"🎯 Computing distance for {label} (bbox: {pixel_width}x{pixel_height})")
        
        # 尝试从深度数据获取距离
        depth_distance = self.get_distance_from_compressed_depth(depth_data, x1, y1, x2, y2)
        
        # 尝试从立体数据获取距离
        stereo_distance = self.get_distance_from_stereo(stereo_frame, x1, y1, x2, y2) if stereo_frame is not None else None
        
        # 基于尺寸的距离估算（作为备用）
        size_distance = self.get_distance_from_width(x1, y1, x2, y2, label)
        
        # 收集所有可用的距离测量
        distances = []
        methods = []
        
        if depth_distance:
            distances.append(depth_distance)
            methods.append("depth")
        
        if stereo_distance:
            distances.append(stereo_distance)
            methods.append("stereo")
        
        distances.append(size_distance)
        methods.append("size_based")
        
        # 选择最佳距离
        if len(distances) >= 2:
            # 检查一致性
            distances_array = np.array(distances)
            std_dev = np.std(distances_array)
            
            if std_dev < 0.8:  # 结果一致
                final_distance = np.median(distances_array)
                if self.debug_mode:
                    self.logger.info(f"✅ Consistent measurements: {final_distance:.2f}m")
            else:
                # 结果不一致，可能有问题
                if self.debug_mode:
                    self.logger.warn(f"⚠️ Inconsistent measurements: {distances}")
                
                # 选择最保守的距离（通常是最大值，除非明显不合理）
                reasonable_distances = [d for d in distances if 0.5 <= d <= 10.0]
                if reasonable_distances:
                    final_distance = max(reasonable_distances)
                    if self.debug_mode:
                        self.logger.info(f"🛡️ Using conservative distance: {final_distance:.2f}m")
                else:
                    final_distance = size_distance
        else:
            final_distance = distances[0] if distances else 2.0
        
        # 尺寸验证
        validated_distance = self.validate_distance_with_object_size(
            final_distance, pixel_width, pixel_height, label)
        
        # 平滑处理
        smoothed_distance = self.smooth_distance(f"{label}_{x1//50}_{y1//50}", validated_distance)
        
        if self.debug_mode:
            self.logger.info(f"🎯 Final distance for {label}: {smoothed_distance:.2f}m")
        
        return smoothed_distance
    
    # 保持原有接口兼容性
    def get_distance_from_stereo(self, stereo_frame, x1, y1, x2, y2):
        """立体视觉距离计算（简化版本）"""
        if stereo_frame is None:
            return None
        
        try:
            h, w = stereo_frame.shape
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            # 简单的中心区域采样
            sample_size = min(20, (x2-x1)//3)
            roi_x1 = max(0, center_x - sample_size//2)
            roi_x2 = min(w, center_x + sample_size//2)
            roi_y1 = max(0, center_y - sample_size//2)
            roi_y2 = min(h, center_y + sample_size//2)
            
            if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:
                return None
            
            depth_roi = stereo_frame[roi_y1:roi_y2, roi_x1:roi_x2]
            valid_depths = depth_roi[(depth_roi > 100) & (depth_roi < 8000)]
            
            if len(valid_depths) > 5:
                distance = np.percentile(valid_depths, 25) / 1000.0
                if 0.3 <= distance <= 15.0:
                    return distance
                    
        except Exception as e:
            self.logger.error(f"Stereo distance error: {e}")
        
        return None
    
    def get_distance_from_width(self, x1, y1, x2, y2, label):
        """基于宽度的距离估算"""
        width_pixels = x2 - x1
        
        if label in self.object_real_sizes:
            estimated_real_width = self.object_real_sizes[label]['width']
        else:
            estimated_real_width = 0.3  # 默认值
        
        focal_length_pixels = 525.0
        
        if width_pixels > 0:
            estimated_distance = (estimated_real_width * focal_length_pixels) / width_pixels
            return max(0.5, min(12.0, estimated_distance))
        
        return 2.0
    
    def smooth_distance(self, object_key, new_distance):
        """距离平滑处理"""
        if object_key not in self.distance_history:
            self.distance_history[object_key] = deque(maxlen=3)  # 减少历史长度
        
        self.distance_history[object_key].append(new_distance)
        history = list(self.distance_history[object_key])
        
        if len(history) == 1:
            return history[0]
        
        # 检测异常跳跃
        if len(history) >= 2:
            recent_change = abs(history[-1] - history[-2])
            if recent_change > 2.0:  # 距离突变超过2米
                if self.debug_mode:
                    self.logger.warn(f"🚨 Large distance jump: {recent_change:.2f}m")
                # 使用历史中位数
                return np.median(history[:-1]) if len(history) > 2 else history[-2]
        
        # 正常平滑
        return np.mean(history)


# 别名保持兼容性
DepthCalculator = DebugDepthCalculator