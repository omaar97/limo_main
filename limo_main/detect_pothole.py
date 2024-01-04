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

        self.depth_img = None
        self.camera_model = None
        self.all_potholes = MarkerArray()
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
        
        self.pothole_location = self.create_publisher(MarkerArray, '/limo/pothole_location', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
    def get_camera_model(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def analyze_depth(self, ros_depth_img):
        self.depth_img = self.bridge.imgmsg_to_cv2(ros_depth_img, "32FC1")

    def analyze_rgb(self, ros_rgb_img):
        robot_sensor_img = self.bridge.imgmsg_to_cv2(ros_rgb_img, "bgr8")
        self.detect_hole(robot_sensor_img, 1)

    def detect_hole(self, img, map_type):

        if self.camera_model is None:
            return

        if self.depth_img is None:
            return
        
        if map_type == 0:
        
            hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, (140, 100, 80), (160, 255, 255))
            pothole_img = cv2.bitwise_and(img, img, mask=mask)
            gray_img = cv2.cvtColor(pothole_img, cv2.COLOR_BGR2GRAY)
            _, to_get_contours = cv2.threshold(gray_img,5,255,cv2.THRESH_BINARY)
            print(np.amax(to_get_contours))
            if np.amax(to_get_contours) == 0:
                print('No potholes exist in the current image.')
                return
            
            contours, hierarchy = cv2.findContours(to_get_contours,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
            
            cv2.drawContours(img, contours, -1, (0,255,0), 3)
            cv2.imshow('pink', pothole_img)
            cv2.imshow('robot_image', img)
            
            self.locate_holes(contours)

        else:

            hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, (0, 0, 0), (180, 25, 255))
            pothole_img = cv2.bitwise_and(img, img, mask=mask)
            gray_img = cv2.cvtColor(pothole_img, cv2.COLOR_BGR2GRAY)
            enhanced_img = cv2.convertScaleAbs(img, alpha=4)
            #enhanced_img_2 = cv2.convertScaleAbs(img, beta=150)
            #Gaussian = cv2.GaussianBlur(gray_img, (7, 7), 0) 
            #canny = cv2.Canny(img, 100, 150)

            canny = cv2.Canny(gray_img, 85, 255) 

            def callback(x):
                print(x)

            cv2.namedWindow('image') # make a window with name 'image'

            cv2.createTrackbar('L', 'image', 0, 255, callback) #lower threshold trackbar for window 'image
            cv2.createTrackbar('U', 'image', 0, 255, callback) #upper threshold trackbar for window 'image

            while(1):
                #numpy_horizontal_concat = np.concatenate((img, canny), axis=1) # to display image side by side
                cv2.imshow('image', canny)
                k = cv2.waitKey(1) & 0xFF
                if k == 27: #escape key
                    break
                l = cv2.getTrackbarPos('L', 'image')
                u = cv2.getTrackbarPos('U', 'image')

                canny = cv2.Canny(img, l, u)

            cv2.destroyAllWindows()
            
            kernel = np.ones((7, 7), np.uint8)
            #img_erosion = cv2.erode(canny, kernel) 
            img_dilation = cv2.dilate(canny, kernel)
            gray_img_2 = cv2.cvtColor(enhanced_img, cv2.COLOR_BGR2GRAY)
            #canny_2 = cv2.Canny(gray_img_2, 1, 5, apertureSize=3)
            for_blob = cv2.bitwise_not(gray_img_2)
            #lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
            #for x1,y1,x2,y2 in lines[0]:
             #   cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
            #img_erosion = cv2.erode(img_dilation, kernel) 
            params = cv2.SimpleBlobDetector.Params()
            params.filterByInertia = False
            params.filterByConvexity = False
            params.filterByArea = True
            params.minArea = 100
            params.filterByColor = True
            params.blobColor = 255

            detector = cv2.SimpleBlobDetector.create(params)
            keypoints = detector.detect(for_blob)
            im_with_keypoints = cv2.drawKeypoints(canny, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            #_, to_get_contours = cv2.threshold(gray_img,5,255,cv2.THRESH_BINARY)
            
            #contours, hierarchy = cv2.findContours(to_get_contours,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
            
            #cv2.drawContours(img, contours, -1, (0,255,0), 3)
            cv2.imshow('original image', img)
            cv2.imshow('gray_img', gray_img)
            cv2.imshow('enhanced_img', canny)
            
        #pothole_location = []
        '''
        pothole = Marker()

        c = contours[0]
        M = cv2.moments(c)

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        
        #pothole_location.append(self.depth_img[cY, cX])
            
        camera_coords = self.camera_model.projectPixelTo3dRay((cX, cY)) #project the image coords (x,y) into 3D ray in camera coords 
        camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
        camera_coords = [x*self.depth_img[cY, cX] for x in camera_coords] # multiply the vector by depth

        pothole.header.frame_id = "depth_link"
        pothole.type = Marker.SPHERE
        pothole.action = 0
        pothole.pose.position.x = camera_coords[0]
        pothole.pose.position.y = camera_coords[1]
        pothole.pose.position.z = camera_coords[2]
        pothole.pose.orientation.x = 0.0
        pothole.pose.orientation.y = 0.0
        pothole.pose.orientation.z = 0.0
        pothole.pose.orientation.w = 1.0
        pothole.scale.x = 0.02
        pothole.scale.y = 0.02
        pothole.scale.z = 0.02
        pothole.color.r = 0.0
        pothole.color.g = 1.0
        pothole.color.b = 0.0
        pothole.color.a = 1.0

        self.pothole_location.publish(pothole)
        '''
                
        #self.pothole_location.publish(self.all_potholes)
        #print('Current number of potholes:')
        #print(self.n_id)
        cv2.waitKey(1) 

    def locate_holes(self, contours):

        for c in contours:

            pothole = Marker()

            M = cv2.moments(c)

            if M["m00"] == 0:
                return

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            #pothole_location.append(self.depth_img[cY, cX])
             
            camera_coords = self.camera_model.projectPixelTo3dRay((cX, cY)) #project the image coords (x,y) into 3D ray in camera coords 
            camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
            camera_coords = [x*self.depth_img[cY, cX] for x in camera_coords] # multiply the vector by depth

            test_pothole = Pose()
            test_pothole.position.x = camera_coords[0]
            test_pothole.position.y = camera_coords[1]
            test_pothole.position.z = camera_coords[2]
            test_pothole.orientation.x = 0.0
            test_pothole.orientation.y = 0.0
            test_pothole.orientation.z = 0.0
            test_pothole.orientation.w = 1.0

            self.is_pothole_new(test_pothole)
            '''
            pothole.header.frame_id = "depth_link"
            pothole.id = self.n_id
            pothole.type = Marker.SPHERE
            pothole.action = 0
            pothole.pose.position.x = camera_coords[0]
            pothole.pose.position.y = camera_coords[1]
            pothole.pose.position.z = camera_coords[2]
            pothole.pose.orientation.x = 0.0
            pothole.pose.orientation.y = 0.0
            pothole.pose.orientation.z = 0.0
            pothole.pose.orientation.w = 1.0
            pothole.scale.x = 0.05
            pothole.scale.y = 0.05
            pothole.scale.z = 0.05
            pothole.color.r = 0.0
            pothole.color.g = 1.0
            pothole.color.b = 0.0
            pothole.color.a = 1.0
            '''

            

    def is_pothole_new(self, pothole):

        depth_to_odom = self.tf_buffer.lookup_transform('odom', 'depth_link', rclpy.time.Time())
        pose_to_check = do_transform_pose(pothole, depth_to_odom)

        if len(self.all_potholes.markers) == 0:
            new_pothole = Marker()
            new_pothole.header.frame_id = "odom"
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
            return

        #print(pose_to_check.position.x)
        thresh = 0.2
        check_potholes = deepcopy(self.all_potholes.markers)
        for marker in check_potholes:
            print('loop entered')
            #current_marker = do_transform_pose(marker.pose, depth_to_odom)

            if marker.pose.position.x - thresh <= pose_to_check.position.x <= marker.pose.position.x + thresh \
            and marker.pose.position.y - thresh <= pose_to_check.position.y <= marker.pose.position.y + thresh:
                print('Skip this pothole')
                print('This pothole:')
                print(marker.id)
                print(pose_to_check.position.x)
                print(pose_to_check.position.y)
                print('____________')
                return
            
        new_pothole = Marker()
        new_pothole.header.frame_id = "odom"
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

        print('New pothole:')
        print(pose_to_check.position.x)
        print(pose_to_check.position.y)

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