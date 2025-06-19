#!/usr/bin/env python3
"""
Advanced Colour Detection Node
──────────────────────────────
Live camera input → YOLO object detection → Colour analysis → ROS2 output + file log
"""

import rclpy
from rclpy.node import Node
import cv2
import time
import numpy as np
from std_msgs.msg import String
from camera_handler import CameraHandler
from colour_matcher import ColourMatcher
from ultralytics import YOLO
from config import Config


class ColourDetectionNode(Node):
    def __init__(self):
        super().__init__('colour_detection_node')
        self.get_logger().info("🎨 Starting Colour Detection Node")

        # Initialize modules
        self.camera_handler = CameraHandler(self, self.get_logger())
        self.colour_matcher = ColourMatcher(Config.XKCD_CSV_PATH, use_tags=True)
        self.yolo_model = YOLO(Config.YOLO_MODEL_PATH)

        self.colour_pub = self.create_publisher(String, '/features/colour', 10)
        self.log_file = open("colour_log.txt", "w")

        self.timer = self.create_timer(Config.PROCESSING_INTERVAL, self._process_frame)
        self.frame_counter = 0
        self.start_time = time.time()

    def _process_frame(self):
        frame = self.camera_handler.get_latest_frame()
        if frame is None:
            return

        frame = cv2.resize(frame, (640, 480))
        results = self.yolo_model(frame, verbose=False)[0]
        if not results.boxes:
            return

        overlay = frame.copy()

        for box in results.boxes:
            conf = float(box.conf[0])
            if conf < 0.35:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            label = self.yolo_model.names[cls_id]  # ALL classes allowed

            if label not in Config.ALLOWED_LABELS:
                continue


            crop = frame[y1:y2, x1:x2]
            if crop.size == 0:
                continue

            mean_bgr = np.mean(crop.reshape(-1, 3), axis=0)
            rgb = (int(mean_bgr[2]), int(mean_bgr[1]), int(mean_bgr[0]))

            colour_name, tags = self.colour_matcher.match_color(rgb)
            tag_str = ", ".join(tags)
            caption = f"{label}: {colour_name} (RGB {rgb}) [{tag_str}]"

            self.get_logger().info(f"📦 {caption}")
            self.log_file.write(caption + "\n")
            self.colour_pub.publish(String(data=caption))

            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(overlay, caption, (x1, max(y1 - 4, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if Config.ENABLE_GUI_PREVIEW:
            cv2.imshow("Colour Detection Overlay", overlay)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

        self.frame_counter += 1
        if self.frame_counter % 30 == 0:
            elapsed = time.time() - self.start_time
            fps = self.frame_counter / elapsed
            self.get_logger().info(f"📷 FPS: {fps:.2f}")

    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ColourDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
