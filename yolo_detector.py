"""
YOLO检测模块 - 修复控制冲突版本
只负责检测，不直接发布速度指令
"""

import cv2
import time
import threading
from ultralytics import YOLO
from std_msgs.msg import String
from config import Config
from depth_calculator import DepthCalculator

class YOLODetector:
    """YOLO物体检测器 - 修复版"""
    
    def __init__(self, node, logger, camera_handler):
        self.node = node
        self.logger = logger
        self.camera_handler = camera_handler
        
        # 检测状态
        self.detected_obstacle = False
        self.consecutive_close_detections = 0
        
        # 初始化YOLO模型
        self._setup_yolo_model()
        
        # 初始化距离计算器
        self.depth_calculator = DepthCalculator(logger)
        
        # 发布器
        self.obj_pub = node.create_publisher(String, '/detected_objects', 10)
        
        # 启动检测线程
        self.detector_thread = threading.Thread(target=self._yolo_loop, daemon=True)
        self.detector_thread.start()
    
    def _setup_yolo_model(self):
        """初始化YOLO模型"""
        try:
            self.model = YOLO(Config.YOLO_MODEL_PATH)
            self.model.fuse()
            # 关闭YOLO的详细输出
            import logging
            logging.getLogger("ultralytics").setLevel(logging.WARNING)
            self.logger.info("✅ YOLOv8 model loaded and optimized")
        except Exception as e:
            self.logger.error(f"❌ YOLO model load failed: {e}")
            raise e
    
    def _yolo_loop(self):
        """YOLO检测循环"""
        frames_processed = 0
        start_time = time.time()
        
        while True:
            frame = self.camera_handler.get_latest_frame()
            
            if frame is None:
                time.sleep(0.05)
                continue

            try:
                yolo_start = time.time()
                results = self.model(frame, verbose=False)[0]
                yolo_time = time.time() - yolo_start
                
                detection_results = self._process_detection_results(results, frame)
                
                # 发布检测结果
                if detection_results['labels_detected']:
                    msg = String()
                    msg.data = ', '.join(sorted(detection_results['labels_detected']))
                    self.obj_pub.publish(msg)

                # 更新检测状态
                self.detected_obstacle = detection_results['found_close_obstacle'] or detection_results['found_very_close_obstacle']
                
                # 处理后退逻辑（只设置标志，不直接控制）
                self._handle_backup_logic(detection_results['found_very_close_obstacle'])
                
                # 显示检测结果
                self._display_detection_results(frame, detection_results, frames_processed, start_time, yolo_time)
                
                frames_processed += 1

            except Exception as e:
                self.logger.error(f"YOLO detect failed: {e}")
                self.detected_obstacle = False

            time.sleep(0.01)
    
    def _process_detection_results(self, results, frame):
        """处理YOLO检测结果"""
        found_close_obstacle = False
        found_very_close_obstacle = False
        labels_detected = set()
        min_distance = float('inf')
        
        depth_data, stereo_frame = self.camera_handler.get_latest_depth_data()

        if results.boxes is not None:
            for box in results.boxes:
                cls_id = int(box.cls[0].item())
                label = self.model.names[cls_id]
                if label not in Config.ALLOWED_LABELS:
                    continue
                    
                labels_detected.add(label)

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # 使用距离计算器获取距离
                distance = self.depth_calculator.get_object_distance(
                    depth_data, stereo_frame, x1, y1, x2, y2, label)
                min_distance = min(min_distance, distance)
                
                # 添加置信度
                confidence = float(box.conf[0].item())
                
                # 根据距离设置颜色和状态
                if distance < Config.BACKWARD_TRIGGER_DISTANCE:
                    color = (0, 0, 255)  # 红色 - 触发后退
                    found_very_close_obstacle = True
                elif distance < Config.DEPTH_MIN_DISTANCE:
                    color = (0, 165, 255)  # 橙色 - 需要停止
                    found_close_obstacle = True
                else:
                    color = (0, 255, 0)  # 绿色 - 安全
                
                # 绘制检测框和信息
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                info_text = f'{label} {confidence:.2f} ({distance:.2f}m)'
                cv2.putText(frame, info_text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                # 记录日志
                if distance < Config.BACKWARD_TRIGGER_DISTANCE:
                    self.logger.info(f"🚨 VERY CLOSE obstacle '{label}' at {distance:.2f}m (conf: {confidence:.2f})")
                elif distance < Config.DEPTH_MIN_DISTANCE:
                    self.logger.info(f"🛑 Close obstacle '{label}' at {distance:.2f}m (conf: {confidence:.2f})")
        
        return {
            'found_close_obstacle': found_close_obstacle,
            'found_very_close_obstacle': found_very_close_obstacle,
            'labels_detected': labels_detected,
            'min_distance': min_distance
        }
    
    def _handle_backup_logic(self, found_very_close_obstacle):
        """处理后退逻辑 - 只设置标志，不直接控制速度"""
        
        # 检查导航控制器
        if not hasattr(self.node, 'navigation_controller'):
            self.logger.error("❌ Navigation controller not found!")
            return
        
        if found_very_close_obstacle:
            self.consecutive_close_detections += 1
            self.logger.info(f"🚨 Close obstacle detected! Count: {self.consecutive_close_detections}/{Config.CLOSE_DETECTION_THRESHOLD}")
            
            # 检查触发条件
            if (self.consecutive_close_detections >= Config.CLOSE_DETECTION_THRESHOLD and 
                not self.node.navigation_controller.is_backing_up):
                
                self.logger.warn(f"🚨 TRIGGERING BACKUP THROUGH NAVIGATION CONTROLLER")
                
                # 通过导航控制器触发后退（统一控制）
                try:
                    self.node.navigation_controller.initiate_backup()
                    self.logger.info(f"✅ Backup initiated successfully")
                except Exception as e:
                    self.logger.error(f"❌ Backup initiation failed: {e}")
                    
        else:
            # 重置计数器
            if self.consecutive_close_detections > 0:
                self.logger.info(f"✅ Obstacle cleared, resetting count from {self.consecutive_close_detections}")
            self.consecutive_close_detections = 0
    
    def _display_detection_results(self, frame, detection_results, frames_processed, start_time, yolo_time):
        """显示检测结果和状态信息"""
        current_time = time.time()
        elapsed = current_time - start_time
        fps = frames_processed / elapsed if elapsed > 0 else 0
        
        # 构建状态显示
        state_display = getattr(self.node, 'current_state', 'UNKNOWN')
        if hasattr(self.node, 'navigation_controller') and self.node.navigation_controller.is_backing_up:
            state_display += " (BACKING UP)"
        
        status_text = f"State: {state_display} | FPS: {fps:.1f} | YOLO: {yolo_time*1000:.0f}ms"
        cv2.putText(frame, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 检测信息
        network_status = f"Detected: {len(detection_results['labels_detected'])} | Frame: {frames_processed}"
        if detection_results['min_distance'] != float('inf'):
            network_status += f" | Min Dist: {detection_results['min_distance']:.2f}m"
        cv2.putText(frame, network_status, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 后退状态信息
        backup_status = f"Backup Count: {self.consecutive_close_detections}/{Config.CLOSE_DETECTION_THRESHOLD}"
        backup_color = (0, 255, 255)  # 黄色
        
        if hasattr(self.node, 'navigation_controller'):
            if self.node.navigation_controller.is_backing_up:
                backup_status += " | 🔙 BACKING UP"
                backup_color = (0, 0, 255)  # 红色
            else:
                backup_status += " | ✅ NORMAL"
                backup_color = (0, 255, 0)  # 绿色
        
        cv2.putText(frame, backup_status, (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, backup_color, 2)
        
        # 控制权状态
        control_status = "Control: Navigation System"
        if (hasattr(self.node, 'navigation_controller') and 
            self.node.navigation_controller.is_backing_up):
            control_status = "Control: Backup Mode"
        
        cv2.putText(frame, control_status, (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # 显示检测结果
        cv2.imshow("YOLOv8 Unified Control", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            pass
    
    def is_obstacle_detected(self):
        """检查是否检测到障碍物"""
        return self.detected_obstacle
    
    def get_consecutive_detections(self):
        """获取连续检测次数"""
        return self.consecutive_close_detections