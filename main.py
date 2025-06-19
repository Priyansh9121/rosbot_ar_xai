"""
主程序文件 - 传感器融合增强版本
整合所有模块，新增传感器融合功能
"""

import rclpy
from rclpy.node import Node
import cv2

# 导入自定义模块
from config import Config
from camera_handler import CameraHandler
from yolo_detector import YOLODetector
from navigation_controller import NavigationController
from gui_controller import GUIController

class YoloBug2Navigator(Node):
    """增强版YOLO Bug2导航节点 - 集成传感器融合"""
    
    def __init__(self):
        super().__init__('yolo_bug2_node')
        
        self.get_logger().info("🚀 Initializing Enhanced YoloBug2Navigator...")
        
        # 打印传感器融合配置
        Config.print_fusion_config()
        
        # 初始化各个模块
        self._initialize_modules()
        
        self.get_logger().info("✅ Enhanced YoloBug2Navigator initialized successfully!")
        self.get_logger().info("📡 Sensors: YOLO + Depth + LiDAR + Ultrasonic")
    
    def _initialize_modules(self):
        """初始化所有模块"""
        # 1. 初始化摄像头处理器
        self.get_logger().info("📷 Initializing camera handler...")
        self.camera_handler = CameraHandler(self, self.get_logger())
        
        # 2. 初始化导航控制器
        self.get_logger().info("🧭 Initializing navigation controller...")
        self.navigation_controller = NavigationController(self, self.get_logger())
        
        # 3. 初始化增强版YOLO检测器（自动包含传感器融合）
        self.get_logger().info("🎯 Initializing enhanced YOLO detector with sensor fusion...")
        self.yolo_detector = YOLODetector(self, self.get_logger(), self.camera_handler)
        
        # 4. 启动GUI控制器
        self.get_logger().info("🖥️ Starting GUI controller...")
        self.gui_controller = GUIController(self.navigation_controller)
        self.gui_thread = self.gui_controller.start_gui()
        
        # 5. 创建主循环定时器来同步各模块
        self.sync_timer = self.create_timer(0.1, self._sync_modules)
        
        # 6. 新增：创建传感器状态监控定时器
        self.sensor_monitor_timer = self.create_timer(2.0, self._monitor_sensors)
        
        self.get_logger().info("🔄 All modules initialized and synchronized")
    
    def _sync_modules(self):
        """同步各模块的状态"""
        # 更新导航控制器的障碍物状态
        obstacle_detected = self.yolo_detector.is_obstacle_detected()
        self.navigation_controller.update_obstacle_status(obstacle_detected)
        
        # 更新当前状态供其他模块使用
        self.current_state = self.navigation_controller.get_current_state()
    
    def _monitor_sensors(self):
        """新增：监控传感器状态"""
        try:
            # 获取融合状态（如果YOLO检测器有融合功能）
            if hasattr(self.yolo_detector, 'get_fusion_status'):
                fusion_status = self.yolo_detector.get_fusion_status()
                
                # 检查传感器健康状态
                if fusion_status and 'sensor_sources' in fusion_status:
                    active_sensors = fusion_status['sensor_sources']
                    if len(active_sensors) >= 2:
                        self.get_logger().info(f"✅ Multi-sensor fusion active: {', '.join(active_sensors)}")
                    elif len(active_sensors) == 1:
                        self.get_logger().warn(f"⚠️ Single sensor mode: {active_sensors[0]}")
                    else:
                        self.get_logger().warn("⚠️ No active sensors detected")
                
                # 报告最近的障碍物信息
                if (fusion_status and 
                    fusion_status.get('min_distance', float('inf')) < 3.0):
                    distance = fusion_status['min_distance']
                    confidence = fusion_status.get('confidence', 0)
                    sources = fusion_status.get('sensor_sources', [])
                    
                    self.get_logger().info(
                        f"📍 Obstacle detected: {distance:.2f}m, "
                        f"confidence: {confidence:.2f}, "
                        f"sources: {sources}"
                    )
            
        except Exception as e:
            self.get_logger().error(f"Sensor monitoring error: {e}")
    
    def get_current_state(self):
        """获取当前状态"""
        return getattr(self, 'current_state', 'INITIALIZING')
    
    def get_sensor_status(self):
        """新增：获取传感器状态"""
        if hasattr(self.yolo_detector, 'get_fusion_status'):
            return self.yolo_detector.get_fusion_status()
        return None

def main(args=None):
    """主程序入口"""
    print("🤖 Starting Enhanced YoloBug2Navigator with Sensor Fusion...")
    
    # 初始化ROS2
    rclpy.init(args=args)
    
    try:
        # 创建节点
        node = YoloBug2Navigator()
        
        print("✅ Enhanced node created, spinning...")
        print("📡 Available sensors will be automatically detected")
        print("🎯 Multi-sensor fusion will activate when sensors are available")
        print("Press Ctrl+C to stop")
        
        # 运行节点
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("🛑 Keyboard interrupt received")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("🔄 Cleaning up...")
        
        # 清理资源
        try:
            if 'node' in locals():
                node.destroy_node()
        except Exception as e:
            print(f"Cleanup error: {e}")
        
        cv2.destroyAllWindows()
        rclpy.shutdown()
        
        print("👋 Shutdown complete")

if __name__ == '__main__':
    main()