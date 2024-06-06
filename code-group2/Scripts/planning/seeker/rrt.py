#!/usr/bin/env python
import os
import rospy
import tf
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from scipy.ndimage import binary_dilation
from scipy.spatial.transform import Rotation as RO
from actionlib_msgs.msg import GoalStatus
from collections import defaultdict
from std_msgs.msg import String

global frontiers,mapData,global1,global2,global3,globalmaps,count, map_array, costmap_array 
class setup:
    def __init__(self):
        global frontiers,mapData,global1,global2,global3,globalmaps,count, map_array, costmap_array
        # Subscribers' callbacks for the map and frontiers
        mapData=OccupancyGrid()
        frontiers=[]
        global1=OccupancyGrid()
        global2=OccupancyGrid()
        global3=OccupancyGrid()
        globalmaps=[]
        count = 0
        # fetching all parameters for the setup
        rospy.init_node('killall', anonymous=False)
        self.n_robots = rospy.get_param('~n_robots',1)
        self.namespace_init_count = rospy.get_param('namespace_init_count',1)
        self.namespace = rospy.get_param('~namespace','')
        self.map_topic= rospy.get_param('~map_topic','/map')
        rateHz = rospy.get_param('~rate',100)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.mapCallBack)
        # Subscribe to the costmap topic to get the costmap and ensure the robot stays within the costmap
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)
        self.delay_after_assignement=rospy.get_param('~delay_after_assignement',1)
        self.rate = rospy.Rate(rateHz)

    def callBack(self, data):
        global frontiers
        frontiers=[]
        for point in data.points:
            frontiers.append(np.array([point.x,point.y]))

    def mapCallBack(self, data):
        global mapData, map_array
        mapData=data
        map_array = np.asarray(data.data, dtype=np.int8).reshape((data.info.height, data.info.width))
        map_array = np.ma.array(map_array, mask=mapData==-1, fill_value=-1)

    def costmap_callback(self, data):
        global costmap_array
        # Convert the OccupancyGrid message to a numpy array
        costmap_array = np.array(data.data,dtype=np.int8).reshape((data.info.height, data.info.width))



class robot:
    # Initialize the MoveBaseGoal, start and end poses
    goal = MoveBaseGoal()
    start = PoseStamped()
    end = PoseStamped()

    # Initialize the robot parameters and the planning
    def __init__(self, name):
        global visited_counts, map_array
        self.assigned_point = []
        self.name = name
        self.global_frame = rospy.get_param('~global_frame', '/map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.plan_service = rospy.get_param(
            '~plan_service', '/move_base_node/NavfnROS/make_plan')
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0), rospy.Duration(10.0))
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0
        self.position = np.array([trans[0], trans[1]])
        self.assigned_point = self.position
        self.client = actionlib.SimpleActionClient(
            self.name+'/move_base', MoveBaseAction)
        self.client.wait_for_server()
        robot.goal.target_pose.header.frame_id = self.global_frame
        robot.goal.target_pose.header.stamp = rospy.Time.now()
        rospy.wait_for_service(self.name+self.plan_service)
        self.make_plan = rospy.ServiceProxy(
            self.name+self.plan_service, GetPlan)
        robot.start.header.frame_id = self.global_frame
        robot.end.header.frame_id = self.global_frame
        self.is_standing_still = False
        # Adjust as needed
        self.velocity_threshold = 0.05  
        self.orange_cord_x = 0.0
        self.orange_cord_y = 0.0
        # Initialize the visited counts
        visited_counts = np.zeros_like(map_array)  

    # Send the goal to the robot
    def sendGoal(self, point_x, point_y):
        robot.goal.target_pose.pose.position.x = point_x
        robot.goal.target_pose.pose.position.y = point_y
        robot.goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(robot.goal)
        point = np.array([int(point_x), int(point_y)])
        self.assigned_point = point
        # Increment the visited count of the assigned point
        visited_counts[point] += 1  

    # Get a random goal
    def get_random_goal(self, current_position, axis = 1):
        print("getting random goal")
        global visited_counts, map_array, costmap_array
        # Create boolean arrays where True values correspond to cells that are not -1 and not 100
        not_minus_one = map_array != -1
        not_hundred = map_array != 100
        # Perform element-wise logical AND operation between map_array and costmap_array
        map_and_costmap = np.logical_and(not_minus_one, not_hundred)
        # Perform element-wise logical AND operation between the two boolean arrays
        map_and_costmap = np.logical_and(map_and_costmap, costmap_array)

        # Create a mask that is True for cells that are walls
        wall_mask = map_array == 100

        # Create a mask that is True for cells that are walls or within a radius of 4 cells from a wall
        # So the robot does not crash into walls
        expanded_wall_mask = binary_dilation(wall_mask, iterations=4)

        # Exclude the cells that are within the expanded wall mask from the map_and_costmap array
        map_and_costmap = np.logical_and(map_and_costmap, ~expanded_wall_mask)

        # Get the cells that are known to be free from the map and costmap
        known_cells = np.argwhere(map_and_costmap)

        # If no cell has been visited yet, return a random cell
        if np.all(visited_counts == 0):
            if len(known_cells) > 0:
                goal_index = np.random.choice(len(known_cells))
                goal = known_cells[goal_index]
            else:
                return None
        else:
            # Get the indices of the cells that have been visited the least
            least_visited = np.argwhere(visited_counts == np.min(visited_counts))

            # Convert arrays to sets of tuples
            least_visited_set = set(map(tuple, least_visited))
            known_cells_set = set(map(tuple, known_cells))

            # Find the intersection
            least_visited_and_known = np.array(list(least_visited_set & known_cells_set))

            # If all cells are unknown, just return a random cell
            if len(least_visited) == 0:
                goal_index = np.random.choice(len(known_cells))
                goal = known_cells[goal_index]
            else:
                # Choose a random cell from the least visited cells
                goal_index = np.random.choice(len(least_visited_and_known))
                goal = least_visited_and_known[goal_index]

        # Return the coordinates in (x, y) order
        return goal[1], goal[0]  
    
    def get_robot_pose(self):
        listener = tf.TransformListener()

        # Wait for the transform to become available
        listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))

        # Get the latest transform
        (trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))

        return trans, rot
    




class RRT:
    # Initialize the RRT class for RRT planning (Rapidly exploring random tree)
    def __init__(self):
        global failed_goals
        failed_goals = []
        self.current_position = Point()
        self.goal = Point()

    # Initialize the subscriber for the odometry
    def odom_callback(self, data):
        position = data.pose.pose.position
        self.current_position = position

    # Function to convert numpy array coordinate to occupancy map coordinate (for going from array to map coordinate)
    def numpy_to_occupancy_map_coord(self, numpy_coord, rot, map_resolution, map_origin, map_size):
        # Flip the row index to match the occupancy map's coordinate system
        numpy_coord_flipped = np.flip(numpy_coord)
        
        # Convert numpy array coordinate to occupancy map coordinate
        map_coord = (numpy_coord_flipped * map_resolution) + map_origin

        # Convert the quaternion to a rotation matrix
        r = RO.from_quat(rot)
        rotation_matrix = r.as_dcm()

        # Extract the 2D rotation matrix
        rotation_matrix_2d = rotation_matrix[:2, :2]

        # Rotate the coordinates
        map_coord_rotated = np.dot(rotation_matrix_2d, map_coord)

        # # Clip the coordinates to stay within the map bounds
        # map_coord_rotated_clipped = np.clip(map_coord_rotated, [0, 0], map_size * map_resolution)

        return map_coord_rotated

    # Function to convert occupancy map coordinate to numpy array coordinate (for going from map coordinate to array)
    def occupancy_map_to_numpy_coord(self, map_coord_rotated, map_origin, map_resolution, rot):
        # Convert the quaternion to a rotation matrix
        r = RO.from_quat(rot)
        rotation_matrix = r.as_dcm()

        # Extract the 2D rotation matrix
        rotation_matrix_2d = rotation_matrix[:2, :2]

        # Compute the inverse rotation matrix
        rotation_matrix_2d_inv = np.linalg.inv(rotation_matrix_2d)

        # Rotate the coordinates back
        map_coord = np.dot(rotation_matrix_2d_inv, map_coord_rotated)

        # Convert occupancy map coordinate to numpy array coordinate
        numpy_coord_flipped = (map_coord - map_origin) / map_resolution

        # Flip the row index to match the numpy's coordinate system
        numpy_coord = np.flip(numpy_coord_flipped)

        return numpy_coord

    # Function to calculate the Euclidean distance between two points
    def distance(self, point1, point2):
        return ((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2) ** 0.5
    
    # Old function to find frontiers also working
    # def find_frontiers(self, robot_x, robot_y):
    #     global failed_goals, map_array
    #     height, width = map_array.shape
    #     directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Right, Left, Down, Up
    #     frontiers = defaultdict(list)

    #     wall_mask = map_array == 100
    #     expanded_wall_mask = binary_dilation(wall_mask, iterations=4)
    #     known_cells = list(zip(*np.where(np.logical_and(map_array == 0, ~expanded_wall_mask))))

    #     for y, x in known_cells:
    #         for dy, dx in directions:
    #             ny, nx = y + dy, x + dx
    #             if (0 <= ny < height) and (0 <= nx < width) and (map_array[ny, nx] == -1):  # If the neighbor is unknown
    #                 # Check if there are at least 4 cells with a value of 0 in the vicinity
    #                 vicinity = [(ny + dy, nx + dx) for dy, dx in directions]
    #                 if sum(1 for vy, vx in vicinity if 0 <= vy < height and 0 <= vx < width and map_array[vy, vx] == 0) >= 6:
    #                     frontiers[y].append(Point(x, y, 0))
    #                     break

    #     if frontiers:
    #         # Filter out lines that are too short
    #         long_enough_lines = {y: [point for point in points if not any(np.array_equal(np.array([point.x, point.y]), failed_goal) for failed_goal in failed_goals)] for y, points in frontiers.items() if len(points) >= 1}
    #         # Remove lines that became too short after removing failed goals
    #         long_enough_lines = {y: points for y, points in long_enough_lines.items() if len(points) >= 1}
    #         print("Frontiers:")
    #         print(long_enough_lines)
    #         if long_enough_lines:
    #             # Find the furthest line
    #             furthest_line = max(long_enough_lines.keys(), key=lambda y: self.distance(Point(robot_x, robot_y, 0), Point(robot_x, y, 0)))

    #             # Find the middle frontier in the furthest line
    #             furthest_frontier = long_enough_lines[furthest_line][len(long_enough_lines[furthest_line]) // 2]

    #             frontier_array = np.array([furthest_frontier.x, furthest_frontier.y])

    #             # Check if the furthest frontier is within the known area
    #             if map_array[furthest_frontier.x, furthest_frontier.y] == 0:
    #                 frontier_array = np.array([furthest_frontier.x, furthest_frontier.y])
    #                 return frontier_array

    #     return None

    # function to check if a position is valid
    def is_valid_position(self, y, x, map_array, height, width):
        for dy in range(-5, 6):
            for dx in range(-5, 6):
                ny, nx = y + dy, x + dx
                if not (0 <= ny < height and 0 <= nx < width):
                    return False
                if (dy ** 2 + dx ** 2 <= 25 and map_array[ny, nx] != 0) or (dy ** 2 + dx ** 2 > 25 and map_array[ny, nx] == -1):
                    return True
        return False

    # function to find frontiers on the edge of the map
    def find_frontiers(self):
        global map_array, costmap_array, failed_goals
        # Perform element-wise logical AND operation between the two boolean arrays
        map_and_costmap = np.logical_and(map_array, costmap_array)

        # Create a mask that is True for cells that are walls
        wall_mask = map_array == 100

        # Create a mask that is True for cells that are walls or within a radius of 4 cells from a wall
        expanded_wall_mask = binary_dilation(wall_mask, iterations=4)

        # Exclude the cells that are within the expanded wall mask from the map_and_costmap array
        map_and_costmap = np.logical_and(map_and_costmap, ~expanded_wall_mask)
        height, width = map_and_costmap.shape
        for y in range(height):
            for x in range(width):
                if any(np.linalg.norm(np.array([x, y]) - np.array([failed_goal[0], failed_goal[1]])) <= 4 for failed_goal in failed_goals):
                    continue
                if self.is_valid_position(y, x, map_and_costmap, height, width):
                    # Return the valid position as the goal
                    return np.array([x, y])  
        # Return None if no valid position is found
        return None  



# older function for random goal
#     def get_random_goal(self):
#         height, width = map_array.shape
#         directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Right, Left, Down, Up
#         frontiers = []

#         # Get the known cells
#         known_cells = list(zip(*np.where(map_array == 0)))

#         for y, x in known_cells:
#             # Check the neighbors
#             for dy, dx in directions:
#                 ny, nx = y + dy, x + dx
#                 if (0 <= ny < height) and (0 <= nx < width) and (map_array[ny, nx] == -1):  # If the neighbor is unknown
#                     frontiers.append((x, y))  # Append the coordinates in (x, y) order
#                     break

#         # If no cell has been visited yet, return a random cell
#         if np.all(self.visited_counts == 0):
#             if frontiers:
#                 goal = random.choice(frontiers)
#             else:
#                 return None
#         else:
#             # Get the indices of the cells that have been visited the least
#             least_visited = np.argwhere(self.visited_counts == np.min(self.visited_counts))

#             # Convert arrays to sets of tuples
#             least_visited_set = set(map(tuple, least_visited))
#             frontiers_set = set(frontiers)

#             # Find the intersection
#             least_visited_and_frontiers = np.array(list(least_visited_set & frontiers_set))

#             # If all cells are unknown, just return a random cell
#             if len(least_visited_and_frontiers) == 0:
#                 if frontiers:
#                     goal = random.choice(frontiers)
#                 else:
#                     return None
#             else:
#                 # Choose a random cell from the least visited cells
#                 goal = random.choice(least_visited_and_frontiers)

#         return goal

# Setup the environment
S = setup()
# if there are more robots add them to the list
robots=[]
if len(S.namespace)>0:
    for i in range(0,S.n_robots):
        robots.append(robot(S.namespace+str(i+S.namespace_init_count)))
elif len(S.namespace)==0:
        robots.append(robot(S.namespace))
# pause_execution is a global variable that is used to pause the execution of the main loop
# when the other robot is found
pause_execution = False

# Callback function for the pause topic
def callback(data):
    global pause_execution
    # Assuming the message is a standard String message and we pause when data.data is 'pause'
    if data.data == 'pause':
        pause_execution = True
    elif data.data == 'resume':
        pause_execution = False

# Main function
def main():
    # Initialize the global variables
    global failed_goals

    # Initialize the subscriber for the pausing of the robot
    rospy.Subscriber('/pause_topic', String, callback)

    # Kill all nodes that are not needed
    nodes = os.popen("rosnode list").readlines()
    for i in range(len(nodes)):
        nodes[i] = nodes[i].replace("\n","")

    for node in nodes:
        if node == "/filter" or node == "/assigner" or node =="/global_frame" or node == "/global_detector" or node == "/local_detector":
            os.system("rosnode kill "+ node)

    # Initialize the RRT class
    R = RRT()

    # Main loop automatically repeated for the robot
    while not rospy.is_shutdown():
        # Check if the execution is paused
        if pause_execution:
            continue
        # Check if the goal is still active for every robot 
        # This code is written for both the Air challenge and the possibility of using multiple robots
        for robot in robots:
            # If the goal is still active, skip this iteration
            if robots[0].client.get_state() == GoalStatus.ACTIVE or robots[0].client.get_state() == GoalStatus.PENDING:
                print("Goal is still active")
                continue
        # If no goal is active, continue with the code
        random = False
        # Get the robot's current position and rotation
        trans, rot = robots[0].get_robot_pose()
        robot_x = trans[0]
        robot_y = trans[1]

        # Print the robot's coordinates
        print("Robot coordinates:")
        print(robot_x, robot_y)
        # Test the conversion functions 
        # This is to test if the robot functions correctly
        print("coords to numpy:")
        test_coords = R.occupancy_map_to_numpy_coord(np.array([robot_x, robot_y]), np.array([mapData.info.origin.position.x, mapData.info.origin.position.y]), mapData.info.resolution, rot)
        print(test_coords)
        print("numpy to coords:")
        test_coords = R.numpy_to_occupancy_map_coord(test_coords, rot, mapData.info.resolution, np.array([mapData.info.origin.position.x, mapData.info.origin.position.y], dtype=np.float64), np.array([mapData.info.width, mapData.info.height], dtype=np.int32))
        print(test_coords)
        
        # Find the frontiers
        goal = R.find_frontiers()
        # If all frontiers are visited, get a random goal
        if goal is None:
            random = True
            current_position = np.array([robot_x, robot_y])
            goal_x, goal_y = robots[0].get_random_goal(current_position)
            goal = np.array([goal_x, goal_y])

        # Get the origin and resolution of the map so the goal can be converted to the occupancy map coordinate system
        map_origin = np.array([mapData.info.origin.position.x, mapData.info.origin.position.y])
        map_resolution = mapData.info.resolution
        coord = R.numpy_to_occupancy_map_coord(goal, rot, mapData.info.resolution, np.array([mapData.info.origin.position.x, mapData.info.origin.position.y], dtype=np.float64), np.array([mapData.info.width, mapData.info.height], dtype=np.int32))
        print("Goal coordinates:")
        print(coord)

        # Check if the goal is within the map bounds or extremely far from the robot
        # The lidar has some noise and a goal further than 20 meters is not reliable
        if coord[0] > 20 or coord[1] > 20:
            failed_goals.append(goal)
            continue

        # Send the goal to the robot
        robots[0].sendGoal(coord[0], coord[1])
        # Wait for the goal to finish
        robots[0].client.wait_for_result()

        # Check if the goal succeeded
        if robots[0].client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
            # To ensure that the robot does not go to the same goal again
            failed_goals.append(goal)
        else:
            rospy.loginfo("Goal failed!")
            # To ensure that the robot does not go to the same goal again
            failed_goals.append(goal)

        # Wait for a while before sending the next goal
        rospy.sleep(S.delay_after_assignement)

        S.rate.sleep()

if __name__ == '__main__':
    main()

