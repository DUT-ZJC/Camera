import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import yaml
import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        self.declare_parameter('image_topic',descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('Board_type','Aruco',descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.Board_type = self.get_parameter('Board_type').get_parameter_value().string_value

        self.bridge = CvBridge()

        # 相机标定内参文件路径
        self.camera_info_file = "/home/zhujiachen/.ros/camera_info/default_cam.yaml"
        self.board_size = (8, 6)  # 棋盘格的尺寸
        self.square_size = 0.025  # 每个棋盘格小方块的尺寸，单位：米（这里为 2.5 cm）
        # 加载内参
        self.load_camera_parameters()

        # 订阅图像话题
        self.create_subscription(
            Image,
            image_topic,  # 这里需要替换成你使用的实际话题名
            self.image_callback,
            10
        )
        self.pub = self.create_publisher(PoseStamped, "/camera/Extrinsics", 1)

    def load_camera_parameters(self):
        with open(self.camera_info_file, 'r') as f:
            camera_info = yaml.safe_load(f)
            self.camera_matrix = np.array(camera_info['camera_matrix']['data']).reshape((3, 3))
            self.distortion_coefficients = np.array(camera_info['distortion_coefficients']['data'])
            self.get_logger().info("Camera parameters loaded successfully.")

    def image_callback(self, msg):
        try:
            # 将 ROS 图像消息转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except Exception as e:
            self.get_logger().error(f"Failed to convert imsg 只在 if ret: 或 if len(cornage: {e}")
            return

        # 检测并计算外参
        self.calculate_extrinsic(cv_image)

    def calculate_extrinsic(self, image):
        pose_msg = None  # 确保 pose_msg 在所有路径下都定义

        if self.Board_type == 'Chessboard':

            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 查找棋盘格角点
            ret, corners = cv2.findChessboardCorners(gray_image, self.board_size, None)

            if ret:
                # 精确化角点位置
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
                corners2 = cv2.cornerSubPix(gray_image, corners, (11, 11), (-1, -1), criteria)

                # 生成棋盘格的物理坐标
                objp = np.zeros((np.prod(self.board_size), 3), np.float32)
                objp[:, :2] = np.indices(self.board_size).T.reshape(-1, 2)
                objp *= self.square_size  # 单位是米，注意这里是 2.5 cm 转换为 0.025 米

                # 使用 solvePnP 计算旋转向量和位移向量
                _, rvecs, tvecs = cv2.solvePnP(objp, corners2, self.camera_matrix, self.distortion_coefficients)

                # 将旋转向量转换为旋转矩阵
                rotation_matrix, _ = cv2.Rodrigues(rvecs)

                # 将旋转矩阵转换为四元数
                quat = R.from_matrix(rotation_matrix).as_quat()

                # 创建姿态信息
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_frame"
                pose_msg.pose.position.x = float(tvecs[0])
                pose_msg.pose.position.y = float(tvecs[1])
                pose_msg.pose.position.z = float(tvecs[2])
                pose_msg.pose.orientation.x = float(quat[0])
                pose_msg.pose.orientation.y = float(quat[1])
                pose_msg.pose.orientation.z = float(quat[2])
                pose_msg.pose.orientation.w = float(quat[3])

                # 发布姿态信息，假设你已经有相关的 ROS 2 发布器配置
            else:
                print("未能找到棋盘格角点")
        
        else:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters_create()
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if len(corners) > 0:
                for corner, marker_id in zip(corners, ids):
                    ret, rvec, tvec = aruco.estimatePoseSingleMarkers(corner, 0.05, self.camera_matrix, self.distortion_coefficients)
                    if ret:
                        rotation_matrix, _ = cv2.Rodrigues(rvec)
                        quat = R.from_matrix(rotation_matrix).as_quat()

                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.header.frame_id = "camera_frame"
                        pose_msg.pose.position.x = float(tvec[0][0])
                        pose_msg.pose.position.y = float(tvec[0][1])
                        pose_msg.pose.position.z = float(tvec[0][2])
                        pose_msg.pose.orientation.x = float(quat[0])
                        pose_msg.pose.orientation.y = float(quat[1])
                        pose_msg.pose.orientation.z = float(quat[2])
                        pose_msg.pose.orientation.w = float(quat[3])

                        self.get_logger().info(f'Marker {marker_id}: Rotation:\n{rotation_matrix}\nTranslation:\n{tvec}')
            else:
                self.get_logger().warn("No Aruco markers detected.")
        if pose_msg:
            self.pub.publish(pose_msg)
            self.get_logger().info(f"外参: {pose_msg}")
        else:
            self.get_logger().warn("无外参输出")
    


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
