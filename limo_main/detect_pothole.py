import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image


class PotholeDetector(Node):

    def __init__(self):
        super().__init__('detect_pothole')
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image, 
                                                    "/limo/depth_camera_link/image_raw",
                                                    self.image_callback,
                                                    qos_profile=rclpy.qos.qos_profile_sensor_data) # Set QoS Profile
        
    def image_callback(self, ros_img):
        robot_sensor_img = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
        self.detect_hole(robot_sensor_img)

    def detect_hole(self, img):
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, (140, 100, 80), (160, 255, 255))
        pothole_img = cv2.bitwise_and(img, img, mask=mask)
        cv2.imshow('pink', pothole_img)
        cv2.imshow('robot_image', img)
        cv2.waitKey(1)

 #   def localize_img(self, img):
        

 #   def is_img_new(self, img):


def main(args=None):
    rclpy.init(args=args)
    pothole_detector = PotholeDetector()
    rclpy.spin(pothole_detector)

    pothole_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()