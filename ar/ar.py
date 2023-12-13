import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

import numpy as np
import math
from object_module import *
import sys
import aruco_module as aruco 
from my_constants import *
from utils import get_extended_RT
arg = sys.argv[1]
class ARNode(Node):
	def __init__(self):
		super().__init__('ar_node')
		self.subscription = self.create_subscription(Image, arg, self.image, 10)
		self.publisher_ = self.create_publisher(Image, arg + '/ar', 10)
		self.bridge = CvBridge()

	def image(self, msg):
		self.img =cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"), cv2.COLOR_RGBA2RGB)
		obj = three_d_object('/home/abhiyaan-orin/ros2_ws/install/ar/lib/ar/barrel3.obj', '/home/abhiyaan-orin/ros2_ws/install/ar/lib/ar/barrel3.png')
		marker_colored = cv2.imread('/home/abhiyaan-orin/ros2_ws/install/ar/lib/ar/data/m1.png')
		assert marker_colored is not None, "Could not find the aruco marker image file"
		#accounts for lateral inversion caused by the webcam
		marker_colored = cv2.flip(marker_colored, 1)

		marker_colored =  cv2.resize(marker_colored, (480,480), interpolation = cv2.INTER_CUBIC )
		marker = cv2.cvtColor(marker_colored, cv2.COLOR_BGR2GRAY)
		h,w = marker.shape
		#considering all 4 rotations
		marker_sig1 = aruco.get_bit_sig(marker, np.array([[0,0],[0,w], [h,w], [h,0]]).reshape(4,1,2))
		marker_sig2 = aruco.get_bit_sig(marker, np.array([[0,w], [h,w], [h,0], [0,0]]).reshape(4,1,2))
		marker_sig3 = aruco.get_bit_sig(marker, np.array([[h,w],[h,0], [0,0], [0,w]]).reshape(4,1,2))
		marker_sig4 = aruco.get_bit_sig(marker, np.array([[h,0],[0,0], [0,w], [h,w]]).reshape(4,1,2))

		sigs = [marker_sig1, marker_sig2, marker_sig3, marker_sig4]

		rval, frame = True, self.img
		assert rval, "couldn't access the webcam"
		h2, w2,  _ = frame.shape

		h_canvas = h2
		w_canvas = w2

		

		canvas = np.zeros((h_canvas, w_canvas, 3), np.uint8) #final display
			#canvas[:h, :w, :] = marker_colored #marker for reference

		success, H = aruco.find_homography_aruco(frame, marker, sigs)
			# success = False
		if not success:
				# print('homograpy est failed')
			canvas[:h2 , : , :] = np.flip(frame, axis = 1)
			

		R_T = get_extended_RT(A, H)
		transformation = A.dot(R_T) 
			
		augmented = np.flip(augment(frame, obj, transformation, marker), axis = 1) #flipped for better control
		canvas[:h2 , : , :] = augmented
		ros_image = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
		ros_image.header.stamp = self.get_clock().now().to_msg()
		self.publisher_.publish(ros_image)
		#print('published')
		#cv2.imshow("webcam", canvas)

def main(args=None):
    rclpy.init(args=args)

    arnode = ARNode()

    rclpy.spin(arnode)

    arnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
