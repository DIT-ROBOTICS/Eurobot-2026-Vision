#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class CageDetector(Node):
    def __init__(self):
        super().__init__('cage_detector_node')
        # 指向你訓練好的權重
        self.model_path = "/home/vision/vision_ws/src/yolo_ros/weights/best.pt"
        self.model = YOLO(self.model_path)
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera_cb/color/image_raw', # 確保 topic 名稱正確
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Aruco Detector 已啟動。")

    def image_callback(self, msg):
        try:
            # 現在 numpy<2 了，這行不會報錯了
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            image = cv2.merge([gray_image, gray_image, gray_image])

            # 使用 GPU 執行偵測
            results = self.model(image, conf=0.7, verbose=False, device=0)

            # 畫出結果
            annotated_frame = results[0].plot()
        
            # 顯示畫面
            cv2.imshow("Eurobot 2026 - Hazelnut Detection", annotated_frame)
            # HighGUI needs waitKey to update/handle window events.
            # Without this, the window can appear stuck/frozen.
            cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f"偵測發生錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CageDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        print("正在關閉節點...")
    finally:
        # 統一在這裡關閉一次就好
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()