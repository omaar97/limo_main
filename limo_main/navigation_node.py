import cv2
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler


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
        points_to_compare = np.array(points_to_cover[closest], points_to_cover[-1])
    elif closest + 1 == len(points_to_cover):
        points_to_compare = np.array(points_to_cover[closest], points_to_cover[0])
    else:
        points_to_compare = np.array(points_to_cover[closest], points_to_cover[closest+1])

    determine_from = current_pose - points_to_compare
    comparison_axis = np.argmin(np.abs(points_to_compare[0] - points_to_compare[1]))
    
    if np.abs(determine_from[0][comparison_axis]) < 0.1:
        start_point = closest
    else:
        start_point = closest + 1
    return start_point

def generate_waypoints():
    current_pose = [0.0, 0.0]
    points_to_cover = np.array([[1.1, 0.0],
              [1.1, -1.0],
              [-0.3, -1.0],
              [-1.1, -1.0],
              [-1.1, 0.0],
              [-0.3, 0.0]
              ])
    
    start_point = find_closest_point_cw(current_pose, points_to_cover)

    if start_point < len(points_to_cover):
        choose_points = np.roll(points_to_cover, -start_point, axis=0)
    else:
        choose_points = points_to_cover

    outer_loop = np.append(choose_points, [choose_points[0]], axis = 0)

    inner_loop = np.array([[1.1, -1.0],
                  [-0.3, -1.0],
                  [-0.3, 0.0],
                  [1.1, 0.0]
              ])
    counter = 1
    while sum(outer_loop[-1] == [1.1, 0.0]) != 2:
        outer_loop = np.append(outer_loop, [outer_loop[counter]], axis = 0)
        counter += 1
    
    final_points = np.append(outer_loop, inner_loop, axis=0)
    directions = []
    for point in final_points:
        if point[0] == 1.1 and point[1] == 0.0:
            directions.append(-np.pi/2)
        elif point[0] == -1.1 and point[1] == -1.0:
            directions.append(np.pi/2)
        elif point[1] == 0.0:
            directions.append(0.0)
        else:
            directions.append(np.pi)

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose (0,0,0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose = pose_from_xytheta(0.0, 0.0, 0.0)
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose = pose_from_xytheta(0.0, 0.0, 2*np.pi)

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = -3.0
                navigator.goToPose(goal_pose)

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

    exit(0)


if __name__ == '__main__':
    main()