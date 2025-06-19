#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class BackupTester(Node):
    def __init__(self):
        super().__init__('backup_tester')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def test_backup(self):
        print("🔙 Testing backup commands...")
        
        backup_cmd = Twist()
        backup_cmd.linear.x = -0.2
        backup_cmd.angular.z = 0.0
        
        for i in range(10):
            self.cmd_pub.publish(backup_cmd)
            print(f"📤 Backup command #{i+1}: linear.x = -0.2")
            time.sleep(0.5)
        
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        print("⏹ Stop command sent")

def main():
    rclpy.init()
    tester = BackupTester()
    
    try:
        tester.test_backup()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()