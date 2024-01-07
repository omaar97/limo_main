# This code has three different parts:
# The first part is the TFListener node where it waits 
# The second part is the navigation node, which is activated once a message about the robot location is received
# The node generates waypoints for the robot to navigate through potholes for both types of maps.
# The points ideally cover the whole map. Some issues could arise if the robot deviates from the point it is 
# supposed to reach.
# The last part is to generate a report about the detected potholes once the map is covered
# The report also generates an image of the pothole locations

import cv2
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
import reportlab

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
        self.get_potholes = self.create_subscription(MarkerArray, '/limo/pothole_location', self.draw_pothole_map, 10)

    def write_report(self, markers):

        fileName = 'pothole_report.pdf'
        documentTitle = 'PotholeReport'
        title = 'Detailed analysis of detected potholes'
        subTitle = 'The largest thing now!!'
        text = []
        image = 'image.jpg'
        
        for pothole in data.markers:
            x = pothole.pose.position.x
            y = pothole.pose.position.x
            text.append()
        
        print("Number of pothole generated:")
        print(len(data.markers))
        print('Done!')

    def draw_pothole_map(self, data):
        map_deviation_x = 76 # deviation of robot initial position in pixels
        map_deviation_y = 10
        this_path = pathlib.Path('navigation_node.py')
        parent = this_path.parent.absolute()
        image_path = parent.joinpath('maps/potholes_20mm.pgm')
        map = cv2.imread(str(image_path), -1)
        pothole_map = deepcopy(map)
        for pothole in data.markers:
            x = pothole.pose.position.x
            y = pothole.pose.position.x
            abs_x = int(x/0.02) + map_deviation_x
            abs_y = int(-y/0.02) + map_deviation_y
            size = 1
            pothole_map[abs_y - size:abs_y + size, abs_x - size:abs_x + size] = 127
        cv2.imwrite(pothole_map)

        self.write_report(data)

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
    
    points_to_cover = np.array([[1.15, 0.0],
              [1.15, -1.05],
              [-0.35, -1.05],
              [-1.15, -1.05],
              [-1.15, 0.0],
              [-0.35, 0.0]
              ])
    
    start_point = find_closest_point_cw(current_pose, points_to_cover)

    if start_point < len(points_to_cover):
        choose_points = np.roll(points_to_cover, -start_point, axis=0)
    else:
        choose_points = points_to_cover

    outer_loop = np.append(choose_points, [choose_points[0]], axis = 0)

    inner_loop = np.array([[1.15, -1.05],
                  [-0.35, -1.05],
                  [-0.35, 0.0],
                  [1.15, 0.0]
              ])
    counter = 1
    while sum(outer_loop[-1] == [1.15, 0.0]) != 2:
        outer_loop = np.append(outer_loop, [outer_loop[counter]], axis = 0)
        counter += 1
    
    final_points = np.append(outer_loop, inner_loop, axis=0)

    return final_points

# The following function locates the closest waypoint for the robot in the direction of the arrows on the map street
def find_closest_point_cw(current_pose, points_to_cover):
    closest = np.argmin(np.linalg.norm(current_pose - points_to_cover, axis = 1)) # Find the closest point to the robot

    if closest == 2:
        points_to_compare = np.array([points_to_cover[closest], points_to_cover[-1]])
    elif closest + 1 == len(points_to_cover):
        points_to_compare = np.array([points_to_cover[closest], points_to_cover[0]])
    else:
        points_to_compare = np.array([points_to_cover[closest], points_to_cover[closest+1]])

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

            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y

            initial_points = generate_waypoints(np.array([robot_x, robot_y]))
            count = 1
            for pt in initial_points:
                if count == len(initial_points):
                    break
                elif count == 1:
                    points = np.linspace(pt, initial_points[count], 4)
                else:
                    to_add = np.linspace(pt, initial_points[count], 4)
                    points = np.append(points, to_add[1:], axis=0)
                count+=1
            
            print('Closest waypoint is')
            print(points[0])

            all_waypoints = []
            single_waypoint = PoseStamped()
            single_waypoint.header.frame_id = 'map'
            single_waypoint.header.stamp = navigator.get_clock().now().to_msg()
            single_waypoint.pose.orientation.z = 1.0
            single_waypoint.pose.orientation.w = 0.0
            for point in points:
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

                single_waypoint.pose = pose_from_xytheta(point[0], point[1], direction)
                all_waypoints.append(deepcopy(single_waypoint))

            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            initial_pose.pose = pose_from_xytheta(robot_x, robot_y, 0.0)
            initial_pose.pose.orientation = transform.transform.rotation
            navigator.setInitialPose(initial_pose)

            print('The robot location is')
            print(robot_x)
            print(robot_y)
            # Wait for navigation to fully activate, since autostarting nav2
            navigator.waitUntilNav2Active()

            navigator.followWaypoints(all_waypoints)

            i = 0
            while not navigator.isTaskComplete():
                # Do something with the feedback
                i = i + 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(all_waypoints)))

            # Do something depending on the return code
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