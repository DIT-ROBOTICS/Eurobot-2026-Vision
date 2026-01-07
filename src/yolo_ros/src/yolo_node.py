#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import tf2_ros
import message_filters

class HazelnutDetector(Node):
    def __init__(self):
        super().__init__('hazelnut_detector_node')

        self.model_path = "/home/vision/vision_ws/src/yolo_ros/weights/best.pt"
        self.model = YOLO(self.model_path)

        self.camera_info_subscription = self.create_subscription(CameraInfo,'/camera/camera_cb/color/camera_info', self.camera_info_callback, 10)
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/camera_cb/color/image_raw')
        self.dep_sub = message_filters.Subscriber(self, Image, '/camera/camera_cb/aligned_depth_to_color/image_raw')
        
        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.camera_intrinsics = None
        self.target_height = 0.03

        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.dep_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        self.get_logger().info("Hazelnut Detector 已啟動。")

    def camera_info_callback(self, msg):
        self.camera_intrinsics = msg

    def broadcast_hazelnut_tf(self, x, y, z, header, index):
        t = TransformStamped()
        t.header.stamp = header.stamp
        t.header.frame_id = header.frame_id
        t.child_frame_id = f"hazelnut_{index}"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def image_callback(self, rgb_msg, dep_msg):
        if self.camera_intrinsics is None:
            self.get_logger().warning("尚未收到相機內參資訊，無法進行偵測。")
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            dep_image = self.bridge.imgmsg_to_cv2(dep_msg, "16UC1")

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            image = cv2.merge([gray_image, gray_image, gray_image])
            results = self.model(image, conf=0.75, verbose=False, device=0)

            for i, box in enumerate(results[0].boxes):
                xyxy = box.xyxy[0].cpu().numpy()
                u = int((xyxy[0] + xyxy[2]) / 2)
                v = int((xyxy[1] + xyxy[3]) / 2)

                u_min, u_max = max(0, u-2), min(dep_image.shape[1], u+2)
                v_min, v_max = max(0, v-2), min(dep_image.shape[0], v+2)
                depth_roi = dep_image[v_min:v_max, u_min:u_max]

                valid_depths = depth_roi[depth_roi > 0]
                if valid_depths.size == 0:
                    self.get_logger().warning(f"目標 {i} 無有效深度資料，跳過。")
                    continue

                depth = np.median(valid_depths) / 1000.0

                fx = self.camera_intrinsics.k[0]
                fy = self.camera_intrinsics.k[4]
                cx = self.camera_intrinsics.k[2]
                cy = self.camera_intrinsics.k[5]

                z_cam = float(depth)
                x_cam = (u - cx) * z_cam / fx
                y_cam = (v - cy) * z_cam / fy

                self.broadcast_hazelnut_tf(x_cam, y_cam, z_cam, rgb_msg.header, i)

            annotated_frame = results[0].plot()
            cv2.imshow("Eurobot 2026 - Hazelnut Detection", annotated_frame)
            cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f"偵測發生錯誤: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = HazelnutDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        print("正在關閉節點...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()