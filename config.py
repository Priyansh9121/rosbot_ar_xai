"""
配置参数模块 - 增强版
保留所有原有功能，新增激光雷达和融合配置
"""

class Config:
    """机器人导航系统配置 - 增强版"""
    
    # =================== 原有配置 ===================
    
    # 导航参数
    GOAL_X = 10.0
    GOAL_Y = 0.0
    GOAL_TOLERANCE = 0.3
    
    # 深度检测参数
    DEPTH_MIN_DISTANCE = 1.25
    DEPTH_HOLD_TIME = 4.0
    
    # 后退功能参数
    BACKWARD_TRIGGER_DISTANCE = 1.0  # 触发后退的距离阈值（米）
    BACKWARD_DISTANCE = 0.5  # 后退的距离（米）
    BACKWARD_SPEED = -0.15  # 后退速度（m/s）
    BACKUP_TIMEOUT = 5.0  # 后退超时时间（秒）
    CLOSE_DETECTION_THRESHOLD = 3  # 连续检测到近距离障碍物的次数阈值
    
    # 图像处理参数
    PROCESSING_INTERVAL = 0.2
    MAX_IMAGE_WIDTH = 960
    MAX_IMAGE_HEIGHT = 540
    
    # 距离平滑参数
    DISTANCE_HISTORY_LENGTH = 4
    DISTANCE_THRESHOLD = 0.3
    
    # 立体视觉参数 - 改进版
    STEREO_NUM_DISPARITIES = 96    # 增加视差范围
    STEREO_BLOCK_SIZE = 21         # 增加块大小提高精度
    FOCAL_LENGTH = 500.0           # 更准确的焦距值
    BASELINE = 0.075
    
    # 运动控制参数
    LINEAR_SPEED = 0.28
    ANGULAR_SPEED = 0.3
    TURN_THRESHOLD = 0.2

    # 手动控制参数
    MANUAL_LINEAR_MULTIPLIER = 0.28
    MANUAL_ANGULAR_MULTIPLIER = 0.3
    
    # YOLO检测参数
    YOLO_MODEL_PATH = "yolov8n.pt"
    ALLOWED_LABELS = [
        "person",
        "bottle",
        "cup",
        "backpack",
        "book",
        "keyboard",
        "mouse",
        "shoe",
        "handbag",
        "remote"
    ]
    XKCD_CSV_PATH = "/home/rmitaiil/aiil_workspace/humble_workspace/src/COSC2781-Ros2-AR-For-Explainable-AI/rosbot_navigation/xkcd_rgb.csv"

    ENABLE_GUI_PREVIEW = False

    # ───── Logging ─────
    ENABLE_FILE_LOGGING = True
    LOG_FILE_PATH = "colour_log.txt"

    ENABLE_TERMINAL_LOG = True
    ENABLE_ROS2_PUBLISH = True
    ROS2_TOPIC = "/features/colour"

    # 网络诊断参数
    NETWORK_CHECK_INTERVAL = 5.0  # 网络诊断间隔（秒）
    LOW_FPS_THRESHOLD = 2.0  # 低帧率警告阈值
    IMAGE_TIMEOUT = 10.0  # 图像接收超时（秒）
    FRAME_REPORT_INTERVAL = 60  # 每N帧报告一次状态
    
    # QoS参数
    QOS_DEPTH = 1  # 只保留最新的1帧
    
    # =================== 新增：激光雷达配置 ===================
    
    # 激光雷达参数（基于ROSbot的RPLIDAR）
    LASER_MAX_RANGE = 12.0           # 最大检测距离
    LASER_MIN_RANGE = 0.12           # 最小检测距离
    LASER_CLUSTER_DISTANCE = 0.2     # 聚类距离阈值
    LASER_CLUSTER_ANGLE = 0.1        # 聚类角度阈值（弧度）
    LASER_MIN_CLUSTER_SIZE = 3       # 最小聚类大小
    LASER_CONFIDENCE_FACTOR = 0.1    # 置信度计算因子
    
    # =================== 新增：传感器融合配置 ===================
    
    # 融合权重
    SENSOR_WEIGHTS = {
        'lidar': 0.9,        # 激光雷达距离最准确
        'yolo': 0.7,         # YOLO提供语义信息
        'depth': 0.6         # 深度相机
    }
    
    # 融合阈值
    FUSION_DISTANCE_THRESHOLD = 0.4   # 同一物体距离阈值（米）
    FUSION_ANGLE_THRESHOLD = 0.2      # 同一物体角度阈值（弧度）
    DETECTION_TIMEOUT = 1.0           # 检测超时时间（秒）
    
    # 置信度提升
    CONFIDENCE_BOOST_PER_SENSOR = 0.15  # 每个额外传感器的置信度提升
    MAX_CONFIDENCE_BOOST = 0.3          # 最大置信度提升
    
    # =================== 新增：增强避障配置 ===================
    
    # 多层避障距离（基于融合后的数据）
    EMERGENCY_STOP_DISTANCE = 0.4     # 紧急停止距离
    DANGER_DISTANCE = 0.8             # 危险距离（开始避障）
    WARNING_DISTANCE = 1.5            # 警告距离（开始减速）
    SAFE_DISTANCE = 2.0               # 安全距离
    
    # 角度范围
    FRONT_DETECTION_ANGLE = 90        # 前方检测角度范围（度）
    SIDE_DETECTION_ANGLE = 135        # 侧方检测角度范围（度）
    
    # 控制参数
    AVOIDANCE_ANGULAR_SPEED = 0.5     # 避障转向速度
    WARNING_LINEAR_SPEED = 0.1        # 警告时的线速度
    
    # =================== 新增：深度计算改进 ===================
    
    # 深度数据处理
    DEPTH_SCALE_FACTOR = 1000.0       # 深度缩放因子（可调整）
    DEPTH_ROI_SIZE = 20               # ROI区域大小
    DEPTH_MIN_VALID_PIXELS = 5        # 最小有效像素数
    DEPTH_FILTER_MIN = 100            # 深度过滤最小值
    DEPTH_FILTER_MAX = 8000           # 深度过滤最大值
    
    # 距离合理性范围
    MIN_REASONABLE_DISTANCE = 0.3     # 最小合理距离
    MAX_REASONABLE_DISTANCE = 10.0    # 最大合理距离
    
    OBJECT_REAL_SIZES = {
    'person': 0.5,
    'bottle': 0.08,
    'cup': 0.08,
    'backpack': 0.35,
    'book': 0.15,
    'handbag': 0.25,
    'remote': 0.12,
    'keyboard': 0.4,
    'mouse': 0.1,
    'shoe': 0.25
    }


    
    # =================== 新增：性能优化 ===================
    
    # 处理频率
    FUSION_FREQUENCY = 10.0           # 融合处理频率（Hz）
    LASER_PROCESSING_FREQUENCY = 20.0 # 激光处理频率（Hz）
    
    # 调试选项
    DEBUG_FUSION = True               # 启用融合调试
    DEBUG_LASER = False               # 启用激光调试
    DEBUG_DISTANCE_CALCULATION = True # 启用距离计算调试
    
    # =================== 方法 ===================
    
    @classmethod
    def get_image_qos(cls):
        """获取图像订阅的QoS配置"""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 允许丢帧以减少延迟
            history=HistoryPolicy.KEEP_LAST,
            depth=cls.QOS_DEPTH,  # 只保留最新的1帧
            durability=DurabilityPolicy.VOLATILE
        )
    
    @classmethod
    def get_laser_qos(cls):
        """获取激光雷达订阅的QoS配置"""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # 激光数据也只保留最新
        )
    
    @classmethod
    def update_depth_scale(cls, new_scale):
        """动态更新深度缩放因子"""
        cls.DEPTH_SCALE_FACTOR = new_scale
        print(f"✅ Depth scale factor updated to: {new_scale}")
    
    @classmethod
    def get_fusion_config(cls):
        """获取融合配置字典"""
        return {
            'sensor_weights': cls.SENSOR_WEIGHTS.copy(),
            'distance_threshold': cls.FUSION_DISTANCE_THRESHOLD,
            'angle_threshold': cls.FUSION_ANGLE_THRESHOLD,
            'detection_timeout': cls.DETECTION_TIMEOUT,
            'confidence_boost': cls.CONFIDENCE_BOOST_PER_SENSOR,
            'max_boost': cls.MAX_CONFIDENCE_BOOST
        }
    
    @classmethod
    def get_laser_config(cls):
        """获取激光雷达配置字典"""
        return {
            'max_range': cls.LASER_MAX_RANGE,
            'min_range': cls.LASER_MIN_RANGE,
            'cluster_distance': cls.LASER_CLUSTER_DISTANCE,
            'cluster_angle': cls.LASER_CLUSTER_ANGLE,
            'min_cluster_size': cls.LASER_MIN_CLUSTER_SIZE,
            'confidence_factor': cls.LASER_CONFIDENCE_FACTOR
        }
    
    @classmethod
    def print_enhanced_config(cls):
        """打印增强配置摘要"""
        print("\n" + "="*60)
        print("🤖 Enhanced Navigation System Configuration")
        print("="*60)
        
        print("\n📡 Sensor Configuration:")
        print(f"  Depth Camera: {cls.MIN_REASONABLE_DISTANCE:.1f}m - {cls.MAX_REASONABLE_DISTANCE:.1f}m")
        print(f"  Laser Scanner: {cls.LASER_MIN_RANGE:.2f}m - {cls.LASER_MAX_RANGE:.1f}m")
        print(f"  YOLO Objects: {len(cls.ALLOWED_LABELS)} types")
        
        print("\n🚨 Avoidance Zones:")
        print(f"  Emergency: < {cls.EMERGENCY_STOP_DISTANCE:.1f}m")
        print(f"  Danger:    < {cls.DANGER_DISTANCE:.1f}m")
        print(f"  Warning:   < {cls.WARNING_DISTANCE:.1f}m")
        print(f"  Safe:      > {cls.SAFE_DISTANCE:.1f}m")
        
        print("\n🔧 Fusion Settings:")
        print(f"  Distance threshold: {cls.FUSION_DISTANCE_THRESHOLD:.2f}m")
        print(f"  Angle threshold: {cls.FUSION_ANGLE_THRESHOLD:.2f} rad")
        print(f"  Sensor weights: LiDAR({cls.SENSOR_WEIGHTS['lidar']}) | YOLO({cls.SENSOR_WEIGHTS['yolo']}) | Depth({cls.SENSOR_WEIGHTS['depth']})")
        
        print("\n⚡ Performance:")
        print(f"  Fusion frequency: {cls.FUSION_FREQUENCY:.1f} Hz")
        print(f"  Debug mode: Fusion({cls.DEBUG_FUSION}) | Distance({cls.DEBUG_DISTANCE_CALCULATION})")
        
        print("="*60 + "\n")