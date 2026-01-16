#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time

class DepthDataCollector(Node):
    def __init__(self):
        super().__init__('depth_collector')
        # 訂閱對齊後的深度圖
        self.sub = self.create_subscription(Image, '/camera/camera_cb/aligned_depth_to_color/image_raw', self.callback, 10)
        self.bridge = CvBridge()
        
        # --- 設定參數 (必須記住這些數值！) ---
        self.MIN_DIST = 0      # mm
        self.MAX_DIST = 3000   # mm (3公尺，超過視為背景)
        self.SAVE_DIR = "src/ultralytics-ros/datasets/depth_dataset"
        
        if not os.path.exists(self.SAVE_DIR):
            os.makedirs(self.SAVE_DIR)  
            
        self.get_logger().info("按下 'space' 鍵存圖")

    def callback(self, msg):
        try:
            # 1. 取得原始深度 (16-bit)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 2. 轉換為偽彩色 (這是核心步驟)
            # Clip: 限制距離在 0~3m
            depth_clamped = np.clip(depth_image, self.MIN_DIST, self.MAX_DIST)
            
            # Normalize: 轉成 0-255
            depth_norm = cv2.normalize(depth_clamped, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Colormap: 上色 (JET 模式對比度高，適合 YOLO)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            
            # 3. 顯示與存檔
            cv2.imshow("Depth View (Training Data)", depth_color)
            key = cv2.waitKey(1)
            
            if key == 32:
                timestamp = int(time.time() * 1000)
                filename = f"{self.SAVE_DIR}/img_{timestamp}.jpg"
                cv2.imwrite(filename, depth_color)
                self.get_logger().info(f"Saved: {filename}")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthDataCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()