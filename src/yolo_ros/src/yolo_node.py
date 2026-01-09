#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseArray, Pose, PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import tf2_ros
import message_filters
import traceback
import tf2_geometry_msgs

class HazelnutDetector(Node):
    def __init__(self):
        super().__init__('hazelnut_detector_node')

        self.model_path = "/home/vision/vision_ws/src/yolo_ros/weights/best_obb.pt"
        self.model = YOLO(self.model_path)

        self.camera_info_subscription = self.create_subscription(CameraInfo,'/camera/camera_cb/color/camera_info', self.camera_info_callback, 10)
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/camera_cb/color/image_raw')
        self.dep_sub = message_filters.Subscriber(self, Image, '/camera/camera_cb/aligned_depth_to_color/image_raw')
        self.pose_pub = self.create_publisher(PoseArray, 'hazelnut_poses', 10)

        #設置一個陣列變數除存當前偵測到的榛果位姿
        self.hazelnut_poses = PoseArray()
        self.hazelnut_poses.header.frame_id = "camera_frame"

        self.rgb_img = None
        self.dep_img = None

        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.camera_intrinsics = None
        self.target_height = 0.03
        
        self.aruco_points = np.array([
            [-0.02, -0.02, 0],
            [ 0.02, -0.02, 0],
            [ 0.02,  0.02, 0],
            [-0.02,  0.02, 0]
        ], dtype=np.float32)

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

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                header.frame_id,
                header.stamp,
                rclpy.duration.Duration(seconds=0.1)
            )

            p_in_cam = PoseStamped()
            p_in_cam.pose.position.x = float(x)
            p_in_cam.pose.position.y = float(y)
            p_in_cam.pose.position.z = float(z)
            p_in_cam.pose.orientation.w = 1.0

            p_in_map = tf2_geometry_msgs.do_transform_pose(p_in_cam.pose, transform)

            self.hazelnut_poses.poses.append(p_in_map)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"無法將 hazelnut_{index} 轉換至 map 座標系: {e}")


    def bb_to_tf(self, results, rgb_msg):
        for i, box in enumerate(results[0].boxes):
            xyxy = box.xyxy[0].cpu().numpy()
            u = int((xyxy[0] + xyxy[2]) / 2)
            v = int((xyxy[1] + xyxy[3]) / 2)

            u_min, u_max = max(0, u-2), min(self.dep_img.shape[1], u+2)
            v_min, v_max = max(0, v-2), min(self.dep_img.shape[0], v+2)
            depth_roi = self.dep_img[v_min:v_max, u_min:u_max]

            valid_depths = depth_roi[depth_roi > 0]
            if valid_depths.size == 0:
                self.get_logger().warning(f"目標 {i} 無有效深度資料，跳過。")
                continue

            depth = np.median(valid_depths) / 1000.0

            self.get_logger().info(f"目標 {i} 深度: {depth:.2f} m")

            fx = self.camera_intrinsics.k[0]
            fy = self.camera_intrinsics.k[4]
            cx = self.camera_intrinsics.k[2]
            cy = self.camera_intrinsics.k[5]

            z_cam = float(depth)
            x_cam = (u - cx) * z_cam / fx
            y_cam = (v - cy) * z_cam / fy

            self.broadcast_hazelnut_tf(x_cam, y_cam, z_cam, rgb_msg.header, i)

    def obb_to_tf(self, results, rgb_msg):
        for i, box in enumerate(results[0].obb):
            obb = box.xywhr[0].cpu().numpy()
            u,v = int(obb[0]), int(obb[1])

            u_min, u_max = max(0, u-2), min(self.dep_img.shape[1], u+2)
            v_min, v_max = max(0, v-2), min(self.dep_img.shape[0], v+2)
            depth_roi = self.dep_img[v_min:v_max, u_min:u_max]

            valid_depths = depth_roi[depth_roi > 0]
            if valid_depths.size == 0:
                self.get_logger().warning(f"目標 {i} 無有效深度資料，跳過。")
                continue

            depth = np.median(valid_depths) / 1000.0

            self.get_logger().info(f"目標 {i} 深度: {depth:.2f} m")

            fx = self.camera_intrinsics.k[0]
            fy = self.camera_intrinsics.k[4]
            cx = self.camera_intrinsics.k[2]
            cy = self.camera_intrinsics.k[5]

            z_cam = float(depth)
            x_cam = (u - cx) * z_cam / fx
            y_cam = (v - cy) * z_cam / fy

            self.broadcast_hazelnut_tf(x_cam, y_cam, z_cam, rgb_msg.header, i)

    def obb_pnp_to_tf(self, results, rgb_msg):
        for i, box in enumerate(results[0].obb):
            image_points = box.xyxyxyxy[0].cpu().numpy().astype(np.float32)

            camera_matrix = np.array(self.camera_intrinsics.k).reshape(3, 3)
            dist_coeffs = np.array(self.camera_intrinsics.d)

            ret, rvec, tvec = cv2.solvePnP(self.aruco_points, image_points, camera_matrix, dist_coeffs)

            if not ret:
                self.get_logger().warning(f"目標 {i} PnP 求解失敗，跳過。")
                continue

            x_cam, y_cam, z_cam = tvec.flatten()

            self.broadcast_hazelnut_tf(x_cam, y_cam, z_cam, rgb_msg.header, i)

    def image_callback(self, rgb_msg, dep_msg):
        if self.camera_intrinsics is None:
            self.get_logger().warning("尚未收到相機內參資訊，無法進行偵測。")
            return
        
        try:
            self.hazelnut_poses.poses = []
            self.hazelnut_poses.header.frame_id = "map" 
            self.hazelnut_poses.header.stamp = rgb_msg.header.stamp

            self.rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            self.dep_img = self.bridge.imgmsg_to_cv2(dep_msg, "16UC1")

            gray_image = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2GRAY)
            image = cv2.merge([gray_image, gray_image, gray_image])
            results = self.model(image, conf=0.75, verbose=False, device=0)

            self.obb_to_tf(results, rgb_msg)

            self.pose_pub.publish(self.hazelnut_poses)

            annotated_frame = results[0].plot(labels=False, conf=True)
            cv2.imshow("Eurobot 2026 - Hazelnut Detection", annotated_frame)
            cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f"偵測發生錯誤: {e}")
            self.get_logger().error(traceback.format_exc())

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