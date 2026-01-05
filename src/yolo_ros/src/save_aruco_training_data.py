#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import uuid
import time

class ImageCollector(Node):
    def __init__(self):
        super().__init__('image_collector_node')

        # --- 設定參數 ---
        # RealSense D435 預設的 Color Topic 通常是這個
        # 如果跑不出來，請先用 'ros2 topic list' 檢查你的 Topic 名稱
        self.topic_name = '/camera/camera_cb/color/image_raw' 
        self.folder_name = 'src/yolo_ros/datasets/aruco_data'

        # --- 初始化儲存資料夾 ---
        self.count = 0
        self.setup_folder()

        # --- ROS 設定 ---
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            10
        )
        self.current_frame = None

        # --- GUI Loop Timer ---
        # 建立一個 Timer，每 0.01 秒 (100Hz) 執行一次顯示迴圈
        # 這樣可以模擬原本 while True 的效果，又不會卡住 ROS 的通訊
        self.timer = self.create_timer(0.01, self.gui_loop)
        
        self.get_logger().info(f"等待接收 Topic: {self.topic_name} ...")
        self.get_logger().info("按 'space' 存檔")

    def setup_folder(self):
        try:
            if not os.path.exists(self.folder_name):
                os.mkdir(self.folder_name)
                self.count = 0
                print(f"Created folder: {self.folder_name}")
            else:
                print('Folder already exists.')
                # 計算既有的圖片數量，避免檔名混淆雖有用 uuid 但計數器還是為了顯示方便
                list_ = os.listdir(self.folder_name)
                self.count = len(list_)
        except Exception as e:
            self.get_logger().error(f"Error creating folder: {e}")

    def image_callback(self, msg):
        """
        這是 ROS 收到圖片時會觸發的函式
        """
        try:
            # 將 ROS Image 訊息轉換成 OpenCV 格式 (bgr8)
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

    def gui_loop(self):
        """
        這是處理顯示與鍵盤輸入的函式 (模擬原本的 while True)
        """
        if self.current_frame is None:
            return

        # 複製當前畫面以免修改到原始資料
        img = self.current_frame.copy()

        # 顯示文字
        text = "count: " + str(self.count)
        cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1, cv2.LINE_AA)
        
        # 顯示畫面
        cv2.imshow('RealSense Collector', img)

        # 鍵盤監聽
        keyin = cv2.waitKey(1) & 0xFF

        if keyin == 32:
            # 存檔邏輯
            filename = '{}/{}.jpg'.format(self.folder_name, uuid.uuid1())
            cv2.imwrite(filename, self.current_frame)
            self.count += 1
            print(f"Saved: {filename}")
            time.sleep(0.05) # 稍微防手抖，雖然 timer 機制本身就有間隔

def main(args=None):
    rclpy.init(args=args)
    node = ImageCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        # 清理資源
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()