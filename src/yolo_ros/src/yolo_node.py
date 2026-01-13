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
from rcl_interfaces.msg import SetParametersResult 
from scipy.spatial.transform import Rotation as R

class HazelnutDetector(Node):
    def __init__(self):
        super().__init__('hazelnut_detector_node')

        self.model_path = "/home/vision/vision_ws/src/yolo_ros/weights/best_obb.pt"
        self.model = YOLO(self.model_path)

        self.camera_info_subscription = self.create_subscription(CameraInfo,'/camera/camera_cb/color/camera_info', self.camera_info_callback, 10)
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/camera_cb/color/image_raw')
        self.dep_sub = message_filters.Subscriber(self, Image, '/camera/camera_cb/aligned_depth_to_color/image_raw')
        self.pose_pub = self.create_publisher(PoseArray, 'hazelnut_poses', 10)

        self.hazelnut_poses = PoseArray()
        self.hazelnut_poses.header.frame_id = "camera_frame"

        self.rgb_img = None
        self.dep_img = None

        self.bridge = CvBridge()
        self.annotated_frame = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.map_to_camera_tf = None

        self.camera_intrinsics = None
        self.target_height = 0.03
        
        self.aruco_points = np.array([
            [-0.02, -0.02, 0],
            [ 0.02, -0.02, 0],
            [ 0.02,  0.02, 0],
            [-0.02,  0.02, 0]
        ], dtype=np.float32)

        self.fixed_marker_ids = np.array([20, 21, 22, 23])
        self.fixed_marker_positions = np.array([
            [0.6, 1.4, 0.0],
            [2.4, 1.4, 0.0],
            [0.6, 0.6, 0.0],
            [2.4, 0.6, 0.0],
        ], dtype=np.float32)
        self.fixed_marker_idx = -1

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_params)

        self.use_fixed_aruco = False
        self.p_fixed_aruco = None

        self.mode_list = ['bb', 'obb_dep', 'obb_pnp', 'obb_mix']
        self.func_list = [self.bb_to_tf, self.obb_to_tf, self.obb_pnp_to_tf, self.obb_mix_to_tf]

        self.declare_parameter('mode', 'obb_mix')
        mode_str = self.get_parameter('mode').get_parameter_value().string_value
        if mode_str in self.mode_list:
            self.mode = self.mode_list.index(mode_str)
        else:
            self.get_logger().error(f"初始模式 '{mode_str}' 無效，設為 'obb_mix'",)
            self.mode = self.mode_list.index('obb_mix')
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info(f"目前偵測模式: {self.mode}")

        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.dep_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        self.get_logger().info("Hazelnut Detector 已啟動。")
        
    def parameter_callback(self, params):
        result = SetParametersResult()
        result.successful = True

        for param in params:
            if param.name == 'mode':
                if param.value in self.mode_list:
                    self.mode = self.mode_list.index(param.value)
                    self.get_logger().info(f"已動態切換至模式: {param.value} (索引: {self.mode})")
                else:
                    result.successful = False
                    result.reason = f"無效模式: {param.value}。可用選項: {self.mode_list}"

        return result

    def camera_info_callback(self, msg):
        self.camera_intrinsics = msg
        if self.camera_intrinsics is not None:
            self.destroy_subscription(self.camera_info_subscription)
            self.camera_info_subscription = None
            self.get_logger().info("已接收相機內參資訊，停止訂閱 CameraInfo。")

    def map_to_camera_lookup(self, header):
        try:
            self.map_to_camera_tf = self.tf_buffer.lookup_transform(
                "map",
                header.frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"無法取得 map 到 camera 的轉換: {e}")

    def broadcast_hazelnut_tf(self, pose_in_map, index):
        t = TransformStamped()
        t.header.stamp = self.hazelnut_poses.header.stamp
        t.header.frame_id = "map" 
        t.child_frame_id = f"hazelnut_{index}"

        t.transform.translation.x = pose_in_map.position.x
        t.transform.translation.y = pose_in_map.position.y
        t.transform.translation.z = pose_in_map.position.z

        t.transform.rotation = pose_in_map.orientation

        self.tf_broadcaster.sendTransform(t)

    def calculate_final_pose(self, p_cam):
        try:
            p_nut_map_tf = tf2_geometry_msgs.do_transform_pose(p_cam.pose, self.map_to_camera_tf)
        except Exception as e:
            self.get_logger().error(f"無法轉換 Hazelnut TF: {e}")
            return Pose()

        if self.use_fixed_aruco and self.p_fixed_aruco is not None:
            try:
                p_aruco_map_tf = tf2_geometry_msgs.do_transform_pose(self.p_fixed_aruco.pose, self.map_to_camera_tf)
                
                actual_aruco_pos = self.fixed_marker_positions[self.fixed_marker_idx]

                offset_x = actual_aruco_pos[0] - p_aruco_map_tf.position.x
                offset_y = actual_aruco_pos[1] - p_aruco_map_tf.position.y
                
                final_pose = Pose()
                final_pose.position.x = p_nut_map_tf.position.x + offset_x
                final_pose.position.y = p_nut_map_tf.position.y + offset_y
                final_pose.position.z = 0.03  
                final_pose.orientation.w = 1.0
                
                return final_pose
            except Exception as e:
                self.get_logger().warn(f"補償計算失敗: {e}")
                return p_nut_map_tf
            
        return p_nut_map_tf

    def bb_to_tf(self, i, results):
        xyxy = results.box.xyxy[0].cpu().numpy()
        u = int((xyxy[0] + xyxy[2]) / 2)
        v = int((xyxy[1] + xyxy[3]) / 2)

        u_min, u_max = max(0, u-2), min(self.dep_img.shape[1], u+2)
        v_min, v_max = max(0, v-2), min(self.dep_img.shape[0], v+2)
        depth_roi = self.dep_img[v_min:v_max, u_min:u_max]

        valid_depths = depth_roi[depth_roi > 0]

        if valid_depths.size == 0:
            self.get_logger().warning(f"目標 {i} 無有效深度資料，跳過。")
            return None

        depth = np.median(valid_depths) / 1000.0

        # self.get_logger().info(f"目標 {i} 深度: {depth:.2f} m")

        fx = self.camera_intrinsics.k[0]
        fy = self.camera_intrinsics.k[4]
        cx = self.camera_intrinsics.k[2]
        cy = self.camera_intrinsics.k[5]

        z_cam = float(depth)
        x_cam = (u - cx) * z_cam / fx
        y_cam = (v - cy) * z_cam / fy

        p_dep = PoseStamped()
        p_dep.pose.position.x = float(x_cam)
        p_dep.pose.position.y = float(y_cam)
        p_dep.pose.position.z = float(z_cam)
        p_dep.pose.orientation.w = 1.0

        if self.use_fixed_aruco and self.p_fixed_aruco is not None:
            return self.calculate_final_pose(p_dep)
        else:
            return tf2_geometry_msgs.do_transform_pose(p_dep.pose, self.map_to_camera_tf)


    def obb_to_tf(self, i, results):
        obb = results.obb.xywhr[0].cpu().numpy()
        u,v = int(obb[0]), int(obb[1])

        u_min, u_max = max(0, u-2), min(self.dep_img.shape[1], u+2)
        v_min, v_max = max(0, v-2), min(self.dep_img.shape[0], v+2)
        depth_roi = self.dep_img[v_min:v_max, u_min:u_max]

        valid_depths = depth_roi[depth_roi > 0]

        if valid_depths.size == 0:
            self.get_logger().warning(f"目標 {i} 無有效深度資料，跳過。")
            return None

        depth = np.median(valid_depths) / 1000.0

        # self.get_logger().info(f"目標 {i} 深度: {depth:.2f} m")

        fx = self.camera_intrinsics.k[0]
        fy = self.camera_intrinsics.k[4]
        cx = self.camera_intrinsics.k[2]
        cy = self.camera_intrinsics.k[5]

        z_cam = float(depth)
        x_cam = (u - cx) * z_cam / fx
        y_cam = (v - cy) * z_cam / fy

        p_dep = PoseStamped()
        p_dep.pose.position.x = float(x_cam)
        p_dep.pose.position.y = float(y_cam)
        p_dep.pose.position.z = float(z_cam)
        p_dep.pose.orientation.w = 1.0

        if self.use_fixed_aruco and self.p_fixed_aruco is not None:
            return self.calculate_final_pose(p_dep)
        else:
            return tf2_geometry_msgs.do_transform_pose(p_dep.pose, self.map_to_camera_tf)

    def obb_pnp_to_tf(self,i, results):

        image_points = results.obb.xyxyxyxy[0].cpu().numpy().astype(np.float32)

        camera_matrix = np.array(self.camera_intrinsics.k).reshape(3, 3)
        dist_coeffs = np.array(self.camera_intrinsics.d)

        ret, rvec, tvec = cv2.solvePnP(self.aruco_points, image_points, camera_matrix, dist_coeffs)

        if not ret:
            self.get_logger().warning(f"目標 {i} PnP 求解失敗，跳過。")
            return None

        x_cam, y_cam, z_cam = tvec.flatten()

        p_pnp = PoseStamped()
        p_pnp.pose.position.x = float(x_cam)
        p_pnp.pose.position.y = float(y_cam)
        p_pnp.pose.position.z = float(z_cam)
        p_pnp.pose.orientation.w = 1.0
        
        if self.use_fixed_aruco and self.p_fixed_aruco is not None:
            return self.calculate_final_pose(p_pnp)
        else:
            return tf2_geometry_msgs.do_transform_pose(p_pnp.pose, self.map_to_camera_tf)


    def obb_mix_to_tf(self,i, results):

        pose_map_dep = self.obb_to_tf(i, results)
        pose_map_pnp = self.obb_pnp_to_tf(i, results)

        mixed_pose_map = Pose()
        mixed_pose_map.position.x = (pose_map_dep.position.x + pose_map_pnp.position.x) / 2.0
        mixed_pose_map.position.y = (pose_map_dep.position.y + pose_map_pnp.position.y) / 2.0
        mixed_pose_map.position.z = (pose_map_dep.position.z + pose_map_pnp.position.z) / 2.0
        mixed_pose_map.orientation.w = 1.0

        return mixed_pose_map
    
    def calibrate_by_fixed_aruco(self, corners, ids):
        obj_points = []
        img_points = []
        h = 0.05
        marker_corners_3d = np.array([
            [-h,  h, 0], [ h,  h, 0], [ h, -h, 0], [-h, -h, 0]
        ], dtype=np.float32)

        for i, m_id in enumerate(ids.flatten()):
            if m_id in self.fixed_marker_ids:
                cv2.aruco.drawDetectedMarkers(self.annotated_frame, [corners[i]], np.array(m_id).reshape(-1, 1), (0, 255, 0))
                idx = np.where(self.fixed_marker_ids == m_id)[0][0]
                world_center = self.fixed_marker_positions[idx]
                obj_points.extend(marker_corners_3d + world_center)
                img_points.extend(corners[i].reshape(4, 2))

        if len(obj_points) < 4:
            return False
        camera_matrix = np.array(self.camera_intrinsics.k).reshape(3, 3)
        dist_coeffs = np.array(self.camera_intrinsics.d)
        
        success, rvec, tvec = cv2.solvePnP(
            np.array(obj_points), np.array(img_points), 
            camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
        )

        if success:
            rmat, _ = cv2.Rodrigues(rvec)
            R_inv = rmat.T
            t_inv = -R_inv @ tvec
            pos = t_inv.flatten()
            rmat = R_inv
            t = TransformStamped()
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pos.tolist()
            quat = R.from_matrix(rmat).as_quat()
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat.tolist()
            self.map_to_camera_tf = t
            return True
        
        return False
    
    def find_fixed_marker(self, gray_img):
        gray_img = cv2.createCLAHE(3.0).apply(gray_img)
        gray_img = cv2.GaussianBlur(gray_img, (3, 3), 0)
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray_img)
        if ids is not None:
            self.calibrate_by_fixed_aruco(corners, ids)
            # ids_flat = ids.flatten()
            # for i, marker_id in enumerate(ids_flat):
            #     if(marker_id in self.fixed_marker_ids):
            #         img_pts = corners[i].astype(np.float32)
            #         cv2.aruco.drawDetectedMarkers(self.annotated_frame, [corners[i]], np.array(marker_id).reshape(-1, 1), (0, 255, 0))
            #         self.fixed_marker_idx = np.where(self.fixed_marker_ids == marker_id)[0][0]
            #         h = 0.05
            #         local_pts = np.array([
            #             [-h,  h, 0],
            #             [ h,  h, 0],
            #             [ h, -h, 0],
            #             [-h, -h, 0]  
            #         ], dtype=np.float32)
            #         camera_matrix = np.array(self.camera_intrinsics.k).reshape(3, 3)
            #         dist_coeffs = np.array(self.camera_intrinsics.d)
            #         ret, rvec, tvec = cv2.solvePnP(local_pts, img_pts, camera_matrix, dist_coeffs)
            #         if not ret:
            #             self.get_logger().warning(f"ID {marker_id} PnP 求解失敗，改使用相機 frame")
            #             return None
            #         x_cam, y_cam, z_cam = tvec.flatten()
            #         p_pnp = PoseStamped()
            #         p_pnp.pose.position.x = float(x_cam)
            #         p_pnp.pose.position.y = float(y_cam)
            #         p_pnp.pose.position.z = float(z_cam)
            #         p_pnp.pose.orientation.w = 1.0
            #         self.use_fixed_aruco = True
            #         return p_pnp
        return None

    def image_callback(self, rgb_msg, dep_msg):
        if self.camera_intrinsics is None:
            self.get_logger().warning("尚未收到相機內參資訊，無法進行偵測。")
            return
        try:
            self.map_to_camera_lookup(rgb_msg.header) #在內部修正相機法中應該可以用不到

            self.fixed_marker_idx = -1
            self.use_fixed_aruco = False
            
            self.hazelnut_poses.poses = []
            self.hazelnut_poses.header.frame_id = "map" 
            self.hazelnut_poses.header.stamp = rgb_msg.header.stamp

            self.rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            self.dep_img = self.bridge.imgmsg_to_cv2(dep_msg, "16UC1")

            gray_image = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2GRAY)
            image = cv2.merge([gray_image, gray_image, gray_image])
            results = self.model(image, conf=0.7, verbose=False, device=0)
            self.annotated_frame = results[0].plot(labels=False, conf=True)

            self.p_fixed_aruco = self.find_fixed_marker(gray_image) #要用初始版本的話要註解掉這行

            for i, res in enumerate(results[0]):
                res = self.func_list[self.mode](i, res)
                if res is None:
                    continue
                pose_in_map = res

                self.broadcast_hazelnut_tf(pose_in_map, i)

                self.hazelnut_poses.poses.append(pose_in_map)

            self.pose_pub.publish(self.hazelnut_poses)

            status_text = f"Mode: {self.mode_list[self.mode]} | ArUco: {'LOCKED' if self.use_fixed_aruco else 'LOST'}"
            cv2.putText(self.annotated_frame, status_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Eurobot 2026 - Hazelnut Detection", self.annotated_frame)
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