import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np

from tf2_ros import Buffer

import std_msgs.msg

from camera_geometry import CameraGeometry

from tf_transformations import euler_from_quaternion
roll = None
pitch = None
yaw = None
focalLengthX = 947.8956909179688
focalLengthY = 947.8956909179688
opticalCenterX = 633.8579711914062
opticalCenterY = 352.96539306640625
intrinsic_matrix = np.array([
        [focalLengthX, 0, opticalCenterX],
        [0, focalLengthY, opticalCenterY],
        [0, 0, 1]
    ])
inverse_intrinsic_matrix = np.linalg.inv(intrinsic_matrix)

def imu_callback(msg):
    global pitch, roll, yaw
    tf_buffer = Buffer()
    # Convert quaternion to RPY angles
    msg = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(msg)


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',
            self.image_callback,
            10
        )
        imu_topic = '/zed/zed_node/imu/data'
        imu_subscriber = self.create_subscription(Imu, imu_topic, imu_callback, 10)
        self.publisher = self.create_publisher(PointCloud2, 'aruco/point_cloud', 10)
        self.bridge = CvBridge()



    def image_callback(self, msg):
        try:
            global pitch, roll, yaw
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            self.cg = CameraGeometry(pitch_deg=pitch, yaw_deg=yaw, roll_deg=roll ,image_height=cv_image.shape[0],image_width=cv_image.shape[1])
            # ArUco marker detection
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, _ = detector.detectMarkers(gray)
            
            # Process detected markers and publish PointCloud2
            if ids is not None:
                print('works 0')
                point_cloud_msg = self.create_point_cloud(corners)
                print('work')
                self.publisher.publish(point_cloud_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def create_point_cloud(self, corners):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "zed_left_camera_optical_frame"  # Change this frame_id to match your camera frame

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        points = []
        for marker_corners in corners:
            # Assuming the ArUco marker is a square, calculate the center
            center = np.mean(marker_corners[0], axis=0)
            print(center)
            xy = self.cg.uv_to_roadXYZ_roadframe_iso8855(u=center[0], v=center[1])
            print('work 1')
            point = [xy[0], xy[1], 0.0]
            points.append(point)
        points_array = np.array(points, dtype=np.float32).flatten()

        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = header
        point_cloud_msg.fields = fields
        point_cloud_msg.data = points_array.tobytes()
        point_cloud_msg.point_step = len(fields) * 4  # Size of a single point in bytes
        point_cloud_msg.row_step = len(points_array)  # Total size of the point cloud in bytes
        point_cloud_msg.width = len(points)  # Number of points in the cloud
        point_cloud_msg.height = 1  # Assuming a single row of points
        point_cloud_msg.is_dense = True  # Assuming all points are finite
        return point_cloud_msg


def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoNode()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
