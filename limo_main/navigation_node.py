# This code has three different parts:
# The first part is the TFListener node where it waits 
# The second part is the navigation node, which is activated once a message about the robot location is received
# The node generates waypoints for the robot to navigate through potholes for both types of maps.
# The points ideally cover the whole map. Some issues could arise if the robot deviates from the point it is 
# supposed to reach. 
# The last part is to generate a report about the detected potholes once the map is covered
# The report also generates an image of the pothole locations

########################################################################################################################
## IMPORTANT: For the navigation to work properly, the nav2 controller parameter should be a rotation shim controller ##
########################################################################################################################

import cv2
import os
import pathlib
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
from copy import deepcopy
from rclpy.node import Node
import tf2_ros
from visualization_msgs.msg import MarkerArray
from reportlab.pdfgen import canvas

# A typical TFListener node:
class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None

# A node to write a report about the potholesthat is only activated after the navigation is completed
class report_generator(Node):
    def __init__(self):
        super().__init__('generate_report')
        self.map_type = 0
        self.get_potholes = self.create_subscription(MarkerArray, '/limo/pothole_location', self.draw_pothole_map, 10)

    def draw_pothole_map(self, data): # This function generates the map of the detected potholes
        # The detected potholes are drawn as sky blue squares
        # The potholes are drawn over the provided world maps (stored in the "backgrounds" folder)

        map_deviation_x = 76 # deviation of robot initial position in pixels
        map_deviation_y = 10

        # Get the path (this only work properly when the terminal is in ros2_ws directory):
        this_path = pathlib.Path('navigation_node.py')
        self.parent = this_path.parent.absolute()
        if self.map_type == 0:
            image_path = self.parent.joinpath('src/limo_main/backgrounds/background_potholes_simple.png')
        else:
            image_path = self.parent.joinpath('src/limo_main/backgrounds/background_potholes.png')
        
        map = cv2.imread(str(image_path)) # Get the map
        self.pothole_map = deepcopy(map)
        
        for pothole in data.markers: # Draw the markersas sky blue squares of size 22*22 pixels
            x = pothole.pose.position.x
            y = pothole.pose.position.y
            abs_x = int(((x/0.02) + map_deviation_x)*11.6578947) # This ratio is based on the size of the pgm map vs gazebo map (background)
            abs_y = int(((-y/0.02) + map_deviation_y)*11.6578947)
            size = 11
            self.pothole_map[abs_y-size:abs_y+size, abs_x-size:abs_x+size, 0] = 255
            self.pothole_map[abs_y-size:abs_y+size, abs_x-size:abs_x+size, 1] = 225
            self.pothole_map[abs_y-size:abs_y+size, abs_x-size:abs_x+size, 2] = 0
        cv2.imwrite(str(self.parent.joinpath('src/limo_main/report/potholes.png')), self.pothole_map) # Save the potholes map image
        self.write_report(data) # Write the report

    def write_report(self, markers): # This function generates the report using reportlab library
        # For more information about how it is used, please check here: https://www.geeksforgeeks.org/creating-pdf-documents-with-python/

        pothole_resize = cv2.resize(self.pothole_map, None, fx= 0.3, fy= 0.3, interpolation= cv2.INTER_LINEAR)
        cv2.imwrite('potholes_for_report.png', pothole_resize)
        report_img = 'potholes_for_report.png'
        fileName = self.parent.joinpath('src/limo_main/report/PotholeReport.pdf')
        documentTitle = 'PotholeReport'
        title = 'Detailed analysis of detected potholes'
        headline_1 = '1- The total number of potholes is ' + str(len(markers.markers))
        headline_2 = '2- Pothole locations (in mm) relative to the "map" coordinate frame:'
        headline_3 = '3- The map of the actual vs detected potholes (sky blue squares):'
        text = []
        pdf = canvas.Canvas(str(fileName))
        pdf.setTitle(documentTitle)
        pdf.setFont('Helvetica', 20)
        pdf.drawCentredString(300, 800, title)
        pdf.setFont('Helvetica', 14)
        pdf.drawString(30, 760, headline_1)
        pdf.drawString(30, 730, headline_2)
        text = pdf.beginText(30, 700)
        text.setFont('Helvetica', 10)
        for pothole in markers.markers:
            pothole_num = pothole.id + 1
            x = pothole.pose.position.x*100
            y = pothole.pose.position.x*100
            text_to_add = '- Location of pothole #' + str(pothole_num) + ' is approximately: x: ' + str(int(x)) + ', y: ' + str(int(y))
            text.textLines(str(text_to_add))
        pdf.drawText(text)
        pdf.showPage()
        pdf.setFont('Helvetica', 14)
        pdf.drawString(30, 800, headline_3)
        pdf.drawImage(report_img, 30, 500)
        pdf.save()
        os.remove(str(report_img))

# A typical function to convert normal location to a Pose message
def pose_from_xytheta(x, y, theta):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    q = quaternion_from_euler(0, 0, theta)
    pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return pose

# The following function generates the waypoints for the robot to follow.
# The waypoints are generated based on the available map.
def generate_waypoints(current_pose):
    
    # The following points are the corners of the map, plus the road in the middle.
    # These points were determined experimentally.
    points_to_cover = np.array([[1.15, 0.0],
              [1.15, -1.05],
              [-0.35, -1.05],
              [-1.15, -1.05],
              [-1.15, 0.0],
              [-0.35, 0.0]
              ])
    
    start_point = find_closest_point_cw(current_pose, points_to_cover) # A function to find the closest point to the robot

    # Here the waypoints are determined based on the starting point such that the full map is covered
    if start_point < len(points_to_cover):
        choose_points = np.roll(points_to_cover, -start_point, axis=0)
    else:
        choose_points = points_to_cover

    outer_loop = np.append(choose_points, [choose_points[0]], axis = 0) # These are the waypoints for the outer road

    # These are the coordinates the robot has to follow to cover the inner road (the road in the middle)
    inner_loop = np.array([[1.15, -1.05],
                  [-0.35, -1.05],
                  [-0.35, 0.0],
                  [1.15, 0.0]
              ])
    
    # The following loop ensures the robot always ends up at the top right corner before adding the inner loop coordinates.
    # This ensures the robot stays as much as possible on the asphalt path.
    counter = 1
    while sum(outer_loop[-1] == [1.15, 0.0]) != 2: 
        outer_loop = np.append(outer_loop, [outer_loop[counter]], axis = 0)
        counter += 1
    
    final_points = np.append(outer_loop, inner_loop, axis=0)

    return final_points

# The following function locates the closest waypoint for the robot in the direction of the arrows on the map street
def find_closest_point_cw(current_pose, points_to_cover):
    closest = np.argmin(np.linalg.norm(current_pose - points_to_cover, axis = 1)) # Find the closest point to the robot

    # Pick the closest point and the one next to it in the direction of travel.
    if closest == 2:
        points_to_compare = np.array([points_to_cover[closest], points_to_cover[-1]])
    elif closest + 1 == len(points_to_cover):
        points_to_compare = np.array([points_to_cover[closest], points_to_cover[0]])
    else:
        points_to_compare = np.array([points_to_cover[closest], points_to_cover[closest+1]])

    # Now compare between the current location of the robot and these two points to determine which one the robot should follow.
    # Example: if the closest point is a point that is in the opposite direction of travel, the comparison here will pick the next 
    # point, which is further away from the robot, as the first point in the navigation waypoints.
    determine_from = current_pose - points_to_compare
    comparison_axis = np.argmin(np.abs(points_to_compare[0] - points_to_compare[1]))

    if np.abs(determine_from[0][comparison_axis]) > 0.1:
        start_point = closest
    else:
        start_point = closest + 1
    return start_point

# A typical function for a waypoint follower. The major difference here is that it gets activated
# once the tf transformation is received.
# This allows the robot to navigate and cover the whole map regardless of its starting point.
# Clearly it depends on the accuracy of the initial location of the robot.
# This code should only be run after the robot localizes its starting point.
def main():

    rclpy.init()
    tf_listener = TFListener()

    while rclpy.ok():
        transform = tf_listener.get_tf_transform('map', 'base_link')
        if transform:
            
            navigator = BasicNavigator()

            robot_x = transform.transform.translation.x # Get the location of the robot
            robot_y = transform.transform.translation.y

            initial_points = generate_waypoints(np.array([robot_x, robot_y])) # Generate the major waypoints
            count = 1
            # Here add minor interpolated points. This helps the robot to follow the path in a better way.
            for pt in initial_points:
                if count == len(initial_points):
                    break
                elif count == 1:
                    points = np.linspace(pt, initial_points[count], 4)
                else:
                    to_add = np.linspace(pt, initial_points[count], 4)
                    points = np.append(points, to_add[1:], axis=0)
                count+=1

            # The following lines generate the way points with respect to the map coordinate frame
            all_waypoints = []
            single_waypoint = PoseStamped()
            single_waypoint.header.frame_id = 'map'
            single_waypoint.header.stamp = navigator.get_clock().now().to_msg()
            single_waypoint.pose.orientation.z = 1.0
            single_waypoint.pose.orientation.w = 0.0
            for point in points: # The direction of the robot is determined based on the location of the point
                direction = 0.0
                if point[0] == 1.15 and point[1] == -1.05:
                    direction = -np.pi
                elif point[0] == -1.15 and point[1] == 0.0:
                    continue
                elif point[0] == 1.15:
                    direction = -np.pi/2
                elif point[0] == -1.15:
                    direction = np.pi/2
                elif point[1] == -1.05:
                    direction = -np.pi
                elif point[0] == -0.35 and point[1] != 0.0:
                    direction = np.pi/2

                single_waypoint.pose = pose_from_xytheta(point[0], point[1], direction) # Get the pose of the waypoint
                all_waypoints.append(deepcopy(single_waypoint)) # Append to all waypoints

            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            initial_pose.pose = pose_from_xytheta(robot_x, robot_y, 0.0)
            initial_pose.pose.orientation = transform.transform.rotation
            navigator.setInitialPose(initial_pose) # First possition of the robot

            print('The robot location is')
            print(robot_x)
            print(robot_y)
            # Wait for navigation to fully activate, since autostarting nav2
            navigator.waitUntilNav2Active()

            navigator.followWaypoints(all_waypoints)

            i = 0
            while not navigator.isTaskComplete(): # The current waypoint is printed as a feedback.
                i = i + 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(all_waypoints)))

            # Status of the goal point:
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

            navigator.destroy_node()

            break

        rclpy.spin_once(tf_listener)

    tf_listener.destroy_node()
    rclpy.shutdown()
    
    # Now write the report:
    rclpy.init()
    generate_report = report_generator()
    rclpy.spin_once(generate_report)
    generate_report.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()