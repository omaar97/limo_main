# This node detects potholes for both types of maps.
# The variable self.map_type in the class constructer determines which map to use
# self.map_type = 1 for the realistic pothole map, and self.map_type = 0 for the simple pothole map

# Re-build the package if the self.map_type is changed!

import rclpy
import cv2
import numpy as np
import cv_bridge
import image_geometry
import tf2_ros
from copy import deepcopy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Pose

class PotholeDetector(Node):

    def __init__(self):
        super().__init__('detect_pothole')
        # The relevant variables and publishers/subscribers are initialized here 
        self.map_type = 0
        
        self.depth_img = None
        self.camera_model = None
        self.all_potholes = MarkerArray() # To visualize the pothole locations as markers in rviz
        self.n_id = 0

        self.bridge = cv_bridge.CvBridge()
        
        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                                self.get_camera_model, 
                                                qos_profile=rclpy.qos.qos_profile_sensor_data)
        
        self.depth_sub = self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', 
                                                  self.analyze_depth, qos_profile=rclpy.qos.qos_profile_sensor_data)

        self.rgb_sub = self.create_subscription(Image, 
                                                    "/limo/depth_camera_link/image_raw",
                                                    self.analyze_rgb,
                                                    qos_profile=rclpy.qos.qos_profile_sensor_data) 
        
        self.pothole_location = self.create_publisher(MarkerArray, '/limo/pothole_location', 10) # A publisher for the potholes location

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
    # The first two functions are used to initialize the camera model and acquire the depth image
    def get_camera_model(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def analyze_depth(self, ros_depth_img):
        self.depth_img = self.bridge.imgmsg_to_cv2(ros_depth_img, "32FC1")
    
    # Now pass the rgb image to the message subscriber callback
    def analyze_rgb(self, ros_rgb_img):
        robot_sensor_img = self.bridge.imgmsg_to_cv2(ros_rgb_img, "bgr8")
        self.detect_hole(robot_sensor_img) # The main function to detect the holes

    def detect_hole(self, img):

        # Start by making sure the camera model and depth image are received

        if self.camera_model is None:
            return

        if self.depth_img is None:
            return
        
        # Detect the pothole based on the map type (0 for simple map, 1 for realistic map)
        if self.map_type == 0:

            # The simple map pothole detector is simply based on the contours of the purple-ish colored potholes
            # The hsv values for the pothole colors were determined experimentally.
            # The first part simply draws the contour of purple-colored sections in the HSV rgb image
            hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, (140, 100, 80), (160, 255, 255))
            pothole_img = cv2.bitwise_and(img, img, mask=mask)
            gray_img = cv2.cvtColor(pothole_img, cv2.COLOR_BGR2GRAY)
            _, to_get_contours = cv2.threshold(gray_img,5,255,cv2.THRESH_BINARY)

            # The function is stopped of there isn't any pothole in the image
            if np.amax(to_get_contours) == 0:
                print('No potholes exist in the current image.')
                return
            
            contours, hierarchy = cv2.findContours(to_get_contours,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
            
            cv2.drawContours(img, contours, -1, (0,255,0), 3) # To visualize the pothole in the robot image
            cv2.imshow('Potholes', img)
            
            self.locate_holes(contours) # Now locate the location of the potholes

        else: # If the map type is 1 (realistic map)

            # The realistic map pothole detector works by detecting unique features of the potholes.
            # Start by converting the image to HSV.
            hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, (0, 0, 0), (180, 25, 255)) # Delete the colored parts from the image to minimize errors.
            no_colors = cv2.bitwise_and(img, img, mask=mask)
            gray_img = cv2.cvtColor(no_colors, cv2.COLOR_BGR2GRAY)
            # The result now is a grayscale image that converts the colored elements in the image to a black color
            # This only works well because the potholes are not colored. 

            current_depth = deepcopy(self.depth_img) # Get the current depth image
            
            canny_1 = cv2.Canny(gray_img, 0, 150) # Edge detection of what can be interpreted as uniform edges
            canny_2 = cv2.Canny(gray_img, 0, 20) # Edge detection of most existing edges
            result = cv2.subtract(canny_2, canny_1) # Get only the relatively non-uniform edges
            result[result < 0] = 0 # Eliminate negative values from the image
            far_pixels = np.where(current_depth > 0.75) # Only consider the image within 0.75m of the robot
            result[far_pixels] = 0
            kernel = np.ones((11, 11), np.uint8)
            pothole_img = cv2.dilate(result, kernel) # Now dilate the result to enable contour detection
            # The function is stopped of there isn't any pothole in the image
            if np.amax(pothole_img) == 0:
                print('No potholes exist in the current image.')
                return
            
            contours, hierarchy = cv2.findContours(pothole_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

            final_contours = []
            # From all the detected contours, the following loop only considers contours with certain features:
            # 1- Contour area larger than 1000 pixels: eliminates small random contours)
            # 2- The smallest bounding rectangle of the contour should have an aspect ratio is larger than 0.1 and an area smaller than 7000
            # The second condition eliminates random rectangular features that the robot usually detects
            # Both conditions were observed and their threshold values were found by trial and error
            for cnt in contours:
                (_, _), (w, h), _  = cv2.minAreaRect(cnt)
                aspect_ratio = min(w,h) / max(w,h)
                rect_area = w*h
                if cv2.contourArea(cnt) > 1000 and aspect_ratio > 0.1 and rect_area < 7000:
                    final_contours.append(cnt)
            
            cv2.drawContours(img, final_contours, -1, (0,255,0), 3) # Visualize the potholes
            cv2.imshow('Potholes', img)

            self.locate_holes(final_contours) # Now locate the location of the potholes

        self.pothole_location.publish(self.all_potholes) # Now publish the locations of the potholes to rviz
        cv2.waitKey(1)

    def locate_holes(self, contours):

        for c in contours:
            
            # The first part is a typical way of locating the centroid of contours:
            M = cv2.moments(c)
            if M["m00"] == 0:
                return
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            # The following part converts the location of the pothole centroid from the rgb image to the depth image 
            # This is a slightly modified version of the available code from the coursework assignment template
            camera_coords = self.camera_model.projectPixelTo3dRay((cX, cY)) # project the image coords (x,y) into 3D ray in camera coords 
            camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
            camera_coords = [x*self.depth_img[cY, cX] for x in camera_coords] # multiply the vector by depth

            # Initialize and grap the pose of the pothole as seen by the depth camera
            test_pothole = Pose()
            test_pothole.position.x = camera_coords[0]
            test_pothole.position.y = camera_coords[1]
            test_pothole.position.z = camera_coords[2]
            test_pothole.orientation.x = 0.0
            test_pothole.orientation.y = 0.0
            test_pothole.orientation.z = 0.0
            test_pothole.orientation.w = 1.0

            self.is_pothole_new(test_pothole) # Check if the pothole is new

    def is_pothole_new(self, pothole):

        # Convert the location of the pothole from the depth image to the map coordinates:
        try:
            depth_to_map = self.tf_buffer.lookup_transform('map', 'depth_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return
        pose_to_check = do_transform_pose(pothole, depth_to_map)

        # A special condition for the first pothole:
        if len(self.all_potholes.markers) == 0:
            new_pothole = Marker() # Initialize the pothole as a Marker
            new_pothole.header.frame_id = "map" # The reference point of the marker
            new_pothole.id = self.n_id # The unique ID of the marker
            new_pothole.type = Marker.SPHERE # The shape of the marker
            new_pothole.action = 0
            new_pothole.pose.position.x = pose_to_check.position.x # The pose of the marker
            new_pothole.pose.position.y = pose_to_check.position.y
            new_pothole.pose.position.z = pose_to_check.position.z
            new_pothole.pose.orientation.x = pose_to_check.orientation.x
            new_pothole.pose.orientation.y = pose_to_check.orientation.y
            new_pothole.pose.orientation.z = pose_to_check.orientation.z
            new_pothole.pose.orientation.w = pose_to_check.orientation.w
            new_pothole.scale.x = 0.05 # The size of the marker
            new_pothole.scale.y = 0.05
            new_pothole.scale.z = 0.05
            new_pothole.color.r = 0.0 # The color of the marker
            new_pothole.color.g = 1.0
            new_pothole.color.b = 0.0
            new_pothole.color.a = 1.0
            self.all_potholes.markers.append(new_pothole) # Add the Marker to the MarkerArray
            self.n_id += 1 # Increment the unique ID for the next pothole
            return

        # Now check if the pothole already exists:
        thresh = 0.2 # Threshold to check if the pothole already exists (20cm, determined experimentally)
        check_potholes = deepcopy(self.all_potholes.markers)
        for marker in check_potholes:
            if marker.pose.position.x - thresh <= pose_to_check.position.x <= marker.pose.position.x + thresh \
            and marker.pose.position.y - thresh <= pose_to_check.position.y <= marker.pose.position.y + thresh:
                return # Exit the function if it already exists
        
        # The following lines will be called if the pothole does not exist to add the new pothole:
        new_pothole = Marker()
        new_pothole.header.frame_id = "map"
        new_pothole.id = self.n_id
        new_pothole.type = Marker.SPHERE
        new_pothole.action = 0
        new_pothole.pose.position.x = pose_to_check.position.x
        new_pothole.pose.position.y = pose_to_check.position.y
        new_pothole.pose.position.z = pose_to_check.position.z
        new_pothole.pose.orientation.x = pose_to_check.orientation.x
        new_pothole.pose.orientation.y = pose_to_check.orientation.y
        new_pothole.pose.orientation.z = pose_to_check.orientation.z
        new_pothole.pose.orientation.w = pose_to_check.orientation.w
        new_pothole.scale.x = 0.05
        new_pothole.scale.y = 0.05
        new_pothole.scale.z = 0.05
        new_pothole.color.r = 0.0
        new_pothole.color.g = 1.0
        new_pothole.color.b = 0.0
        new_pothole.color.a = 1.0

        self.all_potholes.markers.append(new_pothole)
        self.n_id += 1

def main(args=None):
    rclpy.init(args=args)
    pothole_detector = PotholeDetector()
    rclpy.spin(pothole_detector)
    pothole_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()