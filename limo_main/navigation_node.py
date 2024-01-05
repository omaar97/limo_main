import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
from copy import deepcopy
from rclpy.node import Node
import tf2_ros

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

def pose_from_xytheta(x, y, theta):
    # negative theta: turn clockwise
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    q = quaternion_from_euler(0, 0, theta)
    pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return pose

def find_closest_point_cw(current_pose, points_to_cover):
    closest = np.argmin(np.linalg.norm(current_pose - points_to_cover, axis = 1))

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

def generate_waypoints(current_pose):
    
    points_to_cover = np.array([[1.125, 0.0],
              [1.125, -1.025],
              [-0.325, -1.025],
              [-1.125, -1.025],
              [-1.125, 0.0],
              [-0.325, 0.0]
              ])
    
    start_point = find_closest_point_cw(current_pose, points_to_cover)

    if start_point < len(points_to_cover):
        choose_points = np.roll(points_to_cover, -start_point, axis=0)
    else:
        choose_points = points_to_cover

    outer_loop = np.append(choose_points, [choose_points[0]], axis = 0)

    inner_loop = np.array([[1.125, -1.025],
                  [-0.325, -1.025],
                  [-0.325, 0.0],
                  [1.125, 0.0]
              ])
    counter = 1
    while sum(outer_loop[-1] == [1.125, 0.0]) != 2:
        outer_loop = np.append(outer_loop, [outer_loop[counter]], axis = 0)
        counter += 1
    
    final_points = np.append(outer_loop, inner_loop, axis=0)

    return final_points

def main():

    rclpy.init()
    tf_listener = TFListener()

    while rclpy.ok():
        transform = tf_listener.get_tf_transform('map', 'base_link')
        if transform:
            navigator = BasicNavigator()

            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y


            points = generate_waypoints(np.array([robot_x, robot_y]))

            print('Closest waypoint is')
            print(points[0])

            all_waypoints = []
            single_waypoint = PoseStamped()
            single_waypoint.header.frame_id = 'map'
            single_waypoint.header.stamp = navigator.get_clock().now().to_msg()
            single_waypoint.pose.orientation.z = 1.0
            single_waypoint.pose.orientation.w = 0.0
            for point in points:
                if point[0] == 1.1 and point[1] == 0.0:
                    direction = -np.pi/2
                elif point[0] == -1.1 and point[1] == -1.0:
                    direction = np.pi/2
                elif point[1] == 0.0:
                    direction = 0.0
                else:
                    direction = np.pi

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

            print()

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

if __name__ == '__main__':
    main()