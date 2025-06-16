#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path, Odometry
from geometry_msgs.msg import Pose, Point, PoseStamped, Twist
import rclpy.time
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time

import numpy as np
from queue import Queue
import time
from copy import copy
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import heapq

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



# Import other python packages that you think necessary


"""
To-dos
1. subscribe to /map topic and extract Occupancy grid
2. process the Occupancy grid
3. frontier detection
4. select frontier goal
5. navigate to the selected goal (+ collision avoidance)
6. repeat 3-5 until all the unknown areas are eliminated
"""


class Task1(Node):
    """
    Environment mapping task.
    """
    def __init__(self):
        super().__init__('task1_node')
        self.timer = self.create_timer(0.1, self.timer_cb)
        # Fill in the initialization member variables that you need
        
        # create a tf2 listener to extract robot's current location
        # https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.create_timer(self.fetch_robot_position, 0.5)
        # time.sleep(1.0)
        
        # subscriber
        self.subscriber1 = self.create_subscription(OccupancyGrid, '/map', self.dataProcess, 10)
        self.subscriber2 = self.create_subscription(Odometry, '/odom', self.fetch_robot_position_odom, 10)
        # self.subscriber3 = self.create_subscription(LaserScan, '/scan', self.scan, 10)
        
        # publisher
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # self.create_timer(0.1, self.fetch_robot_position)
        
        self.map_info = None
        # self.map_resolution = None
        # self.map_width = None
        # self.map_height = None
        # self.map_origin = None
        self.occupancy_grid_2d = None   #[y][x]
        self.x_idx = None
        self.y_idx = None
        # self.x = None
        # self.y = None
        # self.yaw = None
        self.path = None
        self.prevIdx = 0
        self.prevHeadingError = 0
        self.prevTime = 0
        self.prev_heading = 0
        self.prevgoal = (0, 0)
        self.inflated = False
        self.initial = 0
        self.samegoal = 0
        self.scan_data = None
        
        
        
    # def fetch_robot_position(self):
    #     # return (x, y)
        
    #     try:
    #         t = self.tf_buffer.lookup_transform(
    #             target_frame = 'map',
    #             source_frame = 'base_link',
    #             time = rclpy.time.Time(),
    #             timeout = rclpy.duration.Duration(seconds=1.0))
    #         robot_x = t.transform.translation.x
    #         robot_y = t.transform.translation.y
            
    #         qx = t.transform.rotation.x
    #         qy = t.transform.rotation.y
    #         qz = t.transform.rotation.z
    #         qw = t.transform.rotation.w
            
    #         yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
            
    #         # self.get_logger().info(f'CURRENT POSITION x = {robot_x}, y = {robot_y}, yaw = {yaw * 180 / np.pi}')
            
    #         # self.x = robot_x
    #         # self.y = robot_y
    #         # self.yaw = yaw
            
    #         return robot_x, robot_y, yaw
        
    #     except TransformException as ex:
    #         self.get_logger().info(f'TF failed: {ex}')
    #         return None, None, None
        
    def fetch_robot_position_odom(self, msg):
        # self.get_logger().info('Received new position')
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        
        # self.get_logger().info(f'CURRENT POSITION x = {self.x}, y = {self.y}, yaw = {self.yaw * 180 / np.pi}')
           
    
        
    def quaternion_to_yaw(self, qx, qy, qz, qw): 
        q_length = np.sqrt( qx ** 2 + qy ** 2 + qz ** 2 + qw ** 2) 
        qx_norm = qx / q_length
        qy_norm = qy / q_length
        qz_norm = qz / q_length
        qw_norm = qw / q_length
        numerator = 2 * (qw_norm * qz_norm + qx_norm * qy_norm)
        denominator = 1 - 2 * (qy_norm ** 2 + qz_norm ** 2)
        yaw = np.arctan2( numerator ,denominator)
        return yaw

    
    def scan(self, msg):
        if msg is not None:
            self.scan_data = msg


    def timer_cb(self):
        self.get_logger().info('Task1 node is alive.', throttle_duration_sec=1)
        # Feel free to delete this line, and write your algorithm in this callback function

    # Define function(s) that complete the (automatic) mapping task
    
    def dataProcess(self, msg):
        # self.get_logger().info('Received new occupancy grid')
        self.inflated = False
        
        self.map_info = msg.info
        # map_resolution = self.map_info.resolution
        # map_width = self.map_info.width
        # map_height = self.map_info.height
        # map_origin = self.map_info.origin
        occupancy_grid = msg.data   # remember to convert to world frame when navigating

        self.occupancy_grid_2d = np.array(occupancy_grid, dtype=int).reshape((self.map_info.height, self.map_info.width))
        # path = Path()
        # path.header.stamp = Time()
        # path.header.frame_id = 'map'
        # self.path_pub.publish(path)
        # cmd_vel = Twist()
        # cmd_vel.linear.x = 0.0
        # cmd_vel.angular.z = 0.4
        # self.cmd_vel_pub.publish(cmd_vel)
        # status = 0

    # def inflate_map(self, size):
    #     start = time.time()
    #     original = self.occupancy_grid_2d.copy()
        
    #     for y in range(self.map_info.height):
    #         for x in range(self.map_info.width):
                
    #             if original[y][x] == 100:
    #             # if self.occupancy_grid_2d[y][x] > 0:
    #                 # self.occupancy_grid_2d[y][x] = 100
    #                 neighbors = self.adjacentNeighbors(x, y, size=size)
                    
    #                 for x_n, y_n in neighbors:
    #                     if original[y_n][x_n] != -1:
    #                         self.occupancy_grid_2d[y_n][x_n] = 100
                            
                            
                            
    #             ###################################
    #             if original[y][x] == 0:
    #                 neighbors = self.adjacentNeighbors(x, y, size = 2)
                    
    #                 for x_n, y_n in neighbors:
    #                     if original[y_n][x_n] == -1:
    #                         self.occupancy_grid_2d[y][x] = -1
    #                         break
                    
                
    #     self.inflated = True
    #     self.get_logger().info(f'time for inflating map : {time.time()- start}')
    #     # self.get_logger().info(f'inflated map : {self.occupancy_grid_2d}')
        
        
    def inflate_map(self, size):
        start = time.time()
        original = self.occupancy_grid_2d.copy()
        width = self.map_info.width
        height = self.map_info.height
        grid = self.occupancy_grid_2d
        # First pass: inflate obstacles (100)
        for y in range(height):
            for x in range(width):
                if original[y][x] == 100:
                    for dx in range(-size, size + 1):
                        for dy in range(-size, size + 1):
                            nx, ny = x + dx, y + dy
                            if (
                                0 <= nx < width and 0 <= ny < height
                                and original[ny][nx] != -1
                            ):
                                grid[ny][nx] = 100
        # Second pass: mark unknown if surrounded by unknown
        for y in range(height):
            for x in range(width):
                if original[y][x] == 0:
                    for dx in range(-2, 3):
                        for dy in range(-2, 3):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < width and 0 <= ny < height:
                                if original[ny][nx] == -1:
                                    grid[y][x] = -1
                                    break
        self.inflated = True
        self.get_logger().info(f'time for inflating map : {time.time() - start:.4f} seconds')


        
    def wfd(self):
        start = time.time()
        # frontier selection (Wavefront frontier detection)
        # https://arxiv.org/abs/1806.03581
        # https://arxiv.org/pdf/1806.03581
        # robot_x, robot_y = self.fetch_robot_position()
        
        # x_occup_idx = round((robot_x - self.map_info.origin.position.x) / self.map_info.resolution)
        # y_occup_idx = round((robot_y - self.map_info.origin.position.y) / self.map_info.resolution)
        x_occup_idx = self.x_idx
        y_occup_idx = self.y_idx
        
        
        # UNLABELED = 0
        # MAP_OPEN_LIST = 1
        # MAP_CLOSE_LIST = 2
        # FRONTIER_OPEN_LIST = 3
        # FRONTIER_CLOSE_LIST = 4
        labeled_occupancy_grid_2d = np.zeros((self.map_info.height, self.map_info.width), dtype=int)        #[y][x]
        
        # (x, y)
        outter_queue = Queue()
        frontier_list = list()
        
        # enqueue current robot position and mark as "MAP_OPEN_LIST"
        outter_queue.put((x_occup_idx, y_occup_idx))
        labeled_occupancy_grid_2d[y_occup_idx][x_occup_idx] = 1
        
        while not outter_queue.empty():
            x, y = outter_queue.get()
            
            if labeled_occupancy_grid_2d[y][x] == 2:
                continue
            
            if self.isFrontier(x, y):
                # (x, y)
                inner_queue = Queue()
                frontier_list_temp = list()
                
                inner_queue.put((x, y))
                labeled_occupancy_grid_2d[y][x] = 3
                
                while not inner_queue.empty():
                    x_i, y_i = inner_queue.get()
                    
                    if labeled_occupancy_grid_2d[y_i][x_i] == 2 or labeled_occupancy_grid_2d[y_i][x_i] == 4:
                        continue
                    
                    if self.isFrontier(x_i, y_i):
                        frontier_list_temp.append((x_i, y_i))
                        
                        neighbors = self.adjacentNeighbors(x_i, y_i)
                        if len(neighbors) > 0:
                            for x_ii, y_ii in neighbors:
                                
                                # [MAP_OPEN_LIST, FRONTIER_OPEN_LIST, FRONTIER_CLOSE_LIST]
                                if labeled_occupancy_grid_2d[y_ii][x_ii] not in [1, 3, 4]:
                                    inner_queue.put((x_ii, y_ii))
                                    labeled_occupancy_grid_2d[y_ii][x_ii] = 3

                    labeled_occupancy_grid_2d[y_i][x_i] = 4

                if len(frontier_list_temp) > 0:
                    frontier_list.append(frontier_list_temp)
                    for x_f, y_f in frontier_list_temp:
                        labeled_occupancy_grid_2d[y_f][x_f] = 2
            
            neighbors = self.adjacentNeighbors(x ,y)
            if len(neighbors) > 0:
                for x_n, y_n in neighbors:
                    if labeled_occupancy_grid_2d[y_n][x_n] not in [1,2] and self.freeNeighbor(x_n, y_n):
                        outter_queue.put((x_n, y_n))
                        labeled_occupancy_grid_2d[y_n][x_n] = 1
                        
            labeled_occupancy_grid_2d[y][x] = 2
        
        self.get_logger().info(f'time for detecting frontiers : {time.time()- start}')
        return frontier_list
                                           
        
    def isFrontier(self, x, y):
        # (x, y)
        if self.occupancy_grid_2d[y][x] != 0:
            return False
    
        neighbors = self.adjacentNeighbors(x, y)
                
        if len(neighbors) > 0:
            for x, y in neighbors:
                if self.occupancy_grid_2d[y][x] == -1:
                    return True 
        
        return False
    
    def adjacentNeighbors(self, x, y, size = 1):
        # (x, y)
        # directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        
        directions = [(dx, dy) for dx in range(-size, size + 1) for dy in range(-size, size + 1) if not (dx == 0 and dy ==0)]
        neighbors = list()
        
        for dx, dy in directions:
            if (x + dx) >= 0 and (x + dx) < self.map_info.width and (y + dy) >= 0 and (y + dy) < self.map_info.height:
                neighbors.append((x + dx, y + dy))
        
        return neighbors
    
    def freeNeighbor(self, x, y):
        neighbors = self.adjacentNeighbors(x, y)
        if len(neighbors) > 0:
            for x_n, y_n in neighbors:
                if self.occupancy_grid_2d[y_n][x_n] == 0:
                    return True
    
        return False
    
    # def frontierSelect(self, frontiers, robot_x, robot_y, alpha=1.0, beta=0.5, gamma=0.8):
    #     # dist = np.inf
    #     # goal = None
        
    #     # for f in frontiers:
    #     #     mid = len(f) // 2
    #     #     x = sorted([x for x, y in f])
    #     #     y = sorted([y for x, y in f])
            
    #     #     d = np.sqrt((self.x_idx - x[mid]) ** 2 + (self.y_idx - y[mid]) ** 2)
            
    #     #     if d < dist:
    #     #         goal = (x[mid], y[mid])
    #     #         dist = d
        
    #     dist = np.inf
    #     goal = None
        
    #     for f in frontiers:
    #         cx = sum(x for x, y in f) / len(f)
    #         cy = sum(y for x, y in f) / len(f)
            
    #         x, y = max(f, key=lambda pt: (pt[0] - cx)**2 + (pt[1] - cy)**2)
            
    #         # x = cx
    #         # y = cy
            

    #         if self.prevgoal is not None:
    #             prev_dist = np.sqrt((x - self.prevgoal[0]) ** 2 + (y - self.prevgoal[1]) ** 2 )
    #         else:
    #             prev_dist = 0
            
    #         d = np.sqrt((x - robot_x)** 2 + (y - robot_y)** 2)
            
    #         info_gain = 0
            
    #         for dx in [-1, 0, 1]:
    #             for dy in [-1, 0, 1]:
    #                 nx = x + dx
    #                 ny = y + dy
    #                 if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
    #                     if self.occupancy_grid_2d[ny][nx] == -1:
    #                         info_gain += 1
            
    #         score = alpha * d + beta * prev_dist - gamma * info_gain
            
    #         if score < dist:
    #             goal = (x, y)
    #             dist = score                
                    
    #     return goal
    
    
    ######################################################
    def frontierSelect(self, frontiers, robot_x, robot_y, alpha=1.5, gamma=1.6):
        start = time.time()
        dist = np.inf
        goal = None
        info = 0
        for frontier in frontiers:
            if len(frontier) <= 4:
                continue
            
            for fx, fy in frontier: 
                x, y = self.convertFrames((fx, fy))
                # self.get_logger().info(f'x, y ={fx, fy}, prevgoal = {self.prevgoal}')
                
                
                
                d = np.sqrt((x - robot_x)** 2 + (y - robot_y)** 2)
                
                info_gain = 0
                
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        nx = fx + dx
                        ny = fy + dy
                        if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
                            if self.occupancy_grid_2d[ny][nx] == -1:
                                info_gain += 1
                # self.get_logger().info(f'dist = {alpha * d}, info = {gamma * info_gain}')
                score = alpha * d - gamma * info_gain
                
                if score < dist:
                    goal = (fx, fy)
                    dist = score
                    info = info_gain
                   
        self.get_logger().info(f'time for selecting frontier goal : {time.time()- start}')    
        return goal, info
    
    

    # broken
    def farthestFrontier(self, frontiers, robot_x, robot_y, alpha=10, beta=2.0):
        best_score = 0
        best_goal = None
        
        for f in frontiers:
 
            for fx, fy in f:

                # Distance from robot
                x, y = self.convertFrames((fx, fy))
                dist = np.sqrt((x - robot_x) ** 2 + (y - robot_y) ** 2)

                # Info gain: count unknown neighbors around frontier
                info_gain = 0

                for dx in range(-3, 4):
                    for dy in range(-3, 4):
                        nx, ny = fx + dx, fy + dy
                        if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
        
                            if self.occupancy_grid_2d[ny][nx] == -1:  # unknown
                                info_gain += 100

            # Combined score
            # score = alpha * dist + beta * info_gain
            score = info_gain

            if score > best_score:
                best_score = score
                # Now find a valid free cell in this frontier closest to the centroid
                best_goal = (fx, fy)

                    
        return best_goal
        
    
        
    
    def get_path_idx(self, x, y):
        idx = -1
        # dist = np.inf
        idxRadius = 0.2
        
        for i in range(self.prevIdx, len(self.path.poses)):
            if (np.sqrt((x - self.path.poses[i].pose.position.x) ** 2 + (y - self.path.poses[i].pose.position.y) ** 2)) >= idxRadius:
                idx = i
                self.prevIdx = i
                break
            
        if idx == -1:
            idx = len(self.path.poses) - 1

        return idx

    def path_follower(self, x, y, yaw, current_goal):
        status = False
        # kp_speed = 1.8
        # kp_heading = 1.2
        kp_speed = 1.5
        kp_heading = 1.0
        
        # current_time = time.time()
        current_time = self.get_clock().now().nanoseconds * 1e-9
        while current_time == self.prevTime:
            rclpy.spin_once(self, timeout_sec=0.1)
            current_time = self.get_clock().now().nanoseconds

        
        error = np.sqrt(( x - current_goal.pose.position.x) ** 2 + ( y - current_goal.pose.position.y) ** 2)

        if error < 0.2:
            status = True
        
        speed = kp_speed * error
        
        heading_difference_vehicl_goal = np.arctan2(current_goal.pose.position.y - y , current_goal.pose.position.x - x )
        heading_error = heading_difference_vehicl_goal - yaw
        
        if heading_error > np.pi:
            heading_error = -2 * np.pi + heading_error
        elif heading_error < -np.pi:
            heading_error = 2 * np.pi + heading_error
        
        # self.get_logger().info(f'heading: {yaw * 180 / np.pi}, heading_error: {heading_error * 180 / np.pi }')
        
        # self.get_logger().info(f'current time: {current_time}, previous time: {self.prevTime}')
            
        # heading = kp_heading * heading_error + kd_heading* (heading_error - self.prevHeadingError) / (current_time - self.prevTime)
        heading = kp_heading * heading_error
        
        self.prevTime = current_time
        self.prevHeadingError = heading_error
        
        return speed, heading, status

    def move_robot(self, speed, heading):
        cmd_vel = Twist()
        
        if abs(self.prev_heading) >= (np.pi / 4) or abs(heading) >= (np.pi / 4):
            speed = 0.0
            self.prev_heading = heading
            heading = heading / 1
        
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        
        # self.get_logger().info(f'OUTPUT speed: {speed}, heading: {heading * 180 / np.pi}')
        
        self.cmd_vel_pub.publish(cmd_vel)
        
    def convertFrames(self, node):
        # convert from node frame into Rviz frame
        
        x = round(node[0] * self.map_info.resolution + self.map_info.origin.position.x, 2)
        y = round(node[1] * self.map_info.resolution + self.map_info.origin.position.y, 2)
            
        return (x, y)
    
    def enableLaserScan(self):
        self.scan_data = None
        # https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
        self.subscriber3 = self.create_subscription(LaserScan, '/scan', self.scan, 10)
    
    def wallFollower(self, duration = 3):
        start_time = time.time()
        self.enableLaserScan()
        kp = 0.5

        
        while rclpy.ok() and time.time() < start_time + duration:
            rclpy.spin_once(self, timeout_sec= 0.05)
            speed = 0.15
            heading = 0.0
            if self.scan_data is None:
                continue
            
            range = np.array(self.scan_data.ranges)
            min_index = np.nanargmin(range)
            min_angle = (self.scan_data.angle_min + min_index * self.scan_data.angle_increment) * 180 / np.pi
            # range = np.clip(range, 0, 3.0)

            min_range = range[min_index]
            
            front_sector = np.concatenate((range[-15:], range[:16]))
            front_sector = np.clip(front_sector, 0, np.inf)
            front = np.mean(front_sector)
            right = np.mean(range[189:351])
            left = np.mean(range[9:171])
            
            # if front < 0.3:
            #     self.get_logger().info(f'front is too close')
            #     speed = -0.2
            #     cmd_vel = Twist()
            #     cmd_vel.linear.x = speed
            #     self.cmd_vel_pub.publish(cmd_vel)
            #     continue
            

            if min_range <= 0.75:
                # self.get_logger().info(f'obstacle detected in direction = {min_range}')
                cmd_vel = Twist()
                # cmd_vel.linear.x = 0.05
                # cmd_vel.angular.z = -0.5
                if (min_angle <= 45 or min_angle >= 315) and (min_range <= 0.4):
                    self.get_logger().info(f'front is being too close')
                    cmd_vel.linear.x = -0.6
                    cmd_vel.linear.x = -0.2
                    if left > right:
                        cmd_vel.angular.z = -0.1
                    else:
                        cmd_vel.angular.z = 0.1
                    self.cmd_vel_pub.publish(cmd_vel)
                    time.sleep(0.3)
                    continue
                self.get_logger().info(f'obstacle detected in direction = {min_range}')
                if 0<= min_angle <= 90:
                    cmd_vel.linear.x = 0.2
                    # cmd_vel.angular.z = 0.3 *  (min_index - 180) * np.pi / 180
                    cmd_vel.angular.z = 0.65*  (min_angle - 90) * np.pi / 180
                elif 90< min_angle <= 180:
                    cmd_vel.linear.x = 0.2
                    cmd_vel.angular.z = 0.65 *  (min_angle - 90) * np.pi / 180
                elif 180< min_angle <= 270:
                    cmd_vel.linear.x = 0.2
                    cmd_vel.angular.z = 0.7 *  (min_angle - 270) * np.pi / 180
                elif 270 < min_angle <= 360:
                    cmd_vel.linear.x = 0.2
                    cmd_vel.angular.z = 0.7 *  (min_angle - 270) * np.pi / 180
                
                self.cmd_vel_pub.publish(cmd_vel)
                # time.sleep(1)
                continue
            

            
            # if front is np.inf:
            #     front = 3
            # if right is np.inf:
            #     right = 3
            # if left is np.inf:
            #     left = 3
                
            
            elif right > 2 and left > 2 and front > 2:
                self.get_logger().info(f'no angle adjustment')
                side = 0
                side_check = 1
                if right > left:
                    heading = -0.1
                elif right < left:
                    heading = 0.1
                else:
                    heading = 0.0
                speed = 0.3
            

                
              
            # else:
            #     # self.get_logger().info(f'controling angle')
            #     if right > left:
                    
            #         min_index = np.nanargmin(range[0:181])
            #         self.get_logger().info(f'controling angle - left, {min_index}')
            #         heading = kp * (min_index - 90) * np.pi / 180
            #         # if 10 < min_index < 170:
            #         #     heading = kp * (min_index - 90) * np.pi / 180
            #         #     # heading = -0.3
            #         #     # speed = 0.0
            #         # elif 0 <= min_index <= 10:
            #         #     heading = kp * (0 - min_index) * np.pi / 180
            #         # elif 170 <= min_index <= 180:
            #         #     heading = kp * (180 - min_index) * np.pi / 180
            #             # heading = 0.3
            #             # speed = 0.0
            #         # side = -1
            #         # side_check = left
            #         # heading = kp * (min_index - 90) * np.pi / 180
                    
            #     elif right < left:
                    
            #         min_index = np.nanargmin(range[181:len(range)])
            #         self.get_logger().info(f'controling angle - right, {min_index}')
            #         heading = kp * (min_index - 270) * np.pi / 180
            #         # if  190 < min_index < 350:
            #         #     heading = kp * (min_index - 270) * np.pi / 180
            #         #     # heading = -0.3
            #         #     # speed = 0.0
            #         # elif 181 <= min_index <= 190:
            #         #     heading = kp * (180 - min_index) * np.pi / 180
            #         # elif 350 <= min_index <= len(range):
            #         #     heading = kp * (360 - min_index) * np.pi / 180
            #         #     # heading = 0.3
            #         #     # speed = 0.0
            #         # # side = 1
            #         # # side_check = right
            #     else:
            #         heading = 0.0
                    
            # heading = min(max(heading, np.pi * 45/ 180),-np.pi * 45/ 180)
                
            cmd_vel = Twist()
            cmd_vel.linear.x = speed
            cmd_vel.angular.z = heading
            self.cmd_vel_pub.publish(cmd_vel)
            
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info("Wall following complete.")
        self.destroy_subscription(self.subscriber3)
                
    # def wallFollower(self, duration = 3):
    #     start_time = time.time()
    #     self.enableLaserScan()
    #     kp = 0.3
    #     front_threshold = 0.3
        
    #     while rclpy.ok() and time.time() <= start_time + duration:
    #         rclpy.spin_once(self, timeout_sec=0.05)
    #         if self.scan_data is None:
    #             continue
            
    #         range_data = np.array(self.scan_data.ranges)
    #         num_range = len(range_data)
            
    #         right = range_data[179:351]
    #         left = range_data[9:171]
    #         front = np.concatenate((range_data[-10:], range_data[:11]))
    #         right_dist = np.mean(right[np.isfinite(right)]) if np.any(np.isfinite(right)) else 3.0
    #         left_dist = np.mean(left[np.isfinite(left)]) if np.any(np.isfinite(left)) else 3.0
    #         front_dist = np.min(front[np.isfinite(front)]) if np.any(np.isfinite(front)) else 3.0
            
    #         error = left_dist - right_dist
    #         cmd_vel = Twist()
            
    #         if front_dist < front_threshold:
    #             cmd_vel.linear.x = 0.0
                
    #             if left_dist > right_dist:
    #                 cmd_vel.angular.z = 0.3
    #             else:
    #                 cmd_vel.angular.z = -0.3
    #         else:
    #             angular_correction = kp * error
    #             angular_correction = min(max(angular_correction, 0.4), -0.4)
    #             cmd_vel.linear.x = 0.2 
    #             cmd_vel.angular.z = angular_correction
    #             self.get_logger().info(f"Centering correction: {angular_correction * 180 / np.pi}, L: {left_dist:.2f}, R: {right_dist:.2f}")
                
    #         self.cmd_vel_pub.publish(cmd_vel)
        
    #     cmd_vel = Twist()
    #     cmd_vel.linear.x = 0.0
    #     cmd_vel.angular.z = 0.0
    #     self.cmd_vel_pub.publish(cmd_vel)
    #     self.get_logger().info("Wall following complete.")
            
        
    def run(self):
        # status:
        # 0 : Reaches the goal point, need to select a new frontier goal
        # 1 : Navigate to the selected frontier goal point 
        
        status = 0
        # prev_heading = 0
        # prev_time = 0
        times = 180
        start_time = time.time()
        invalid = False
        
        while rclpy.ok():
            """
            1. spin 360
            2. get the occucpancy grid 
            3. calculate each row's medains from the frontier matrix
            4. choose the cloestest one
            5. navigate to the goal
            6. repeat 1-5 until there isn't any unknown area
            """
            rclpy.spin_once(self, timeout_sec= 1.0)
            rclpy.spin_once(self, timeout_sec= 1.0)
            
            if self.map_info is not None:
                
                            
                if self.initial < times:
                
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.25
                    cmd_vel.angular.z = 2.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    self.initial += 1
                    # self.get_logger().info("BOOM!!!")
                    continue
                
                elif self.initial == times:
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    self.initial += 1

                    self.map_info = None
                    self.occupancy_grid_2d = None
                    time.sleep(2)
                    continue
                
                # x, y, yaw = self.fetch_robot_position()
                # # time.sleep(1)
                # while x is None or y is None:
                #     rclpy.spin_once(self)
                #     x, y, yaw = self.fetch_robot_position()
                
                # self.get_logger().info(f'width: {self.map_info.width}, height: {self.map_info.height}')
                
                x = self.x
                y = self.y
                yaw = self.yaw
                
                
                # self.get_logger().info(f'CURRENT POSITION x = {self.x}, y = {self.y}, yaw = {self.yaw * 180 / np.pi}')
                
            
                if status == 0:
                    
                    
                    self.x_idx = round((x - self.map_info.origin.position.x) / self.map_info.resolution)
                    self.y_idx = round((y - self.map_info.origin.position.y) / self.map_info.resolution)
                    
                    # (x,y)
                    if not self.inflated:
                        ####################
                        self.inflate_map(size = 5)
                        # self.get_logger().info(str(type(self.occupancy_grid_2d)))

                    
                    # fig, ax = plt.subplots(dpi=100)
                    # plt.imshow(self.occupancy_grid_2d)
                    # plt.colorbar()
                    # plt.show()
                    
                    frontier_list = self.wfd()
                    
                    # if len(frontier_list) == 0:
                    #     end_time = time.time()
                    #     self.get_logger().info(f'Mapping Completed, time spent: {end_time - start_time}')
                    #     break
                    
                    # self.get_logger().info(f'occupancy grid = {self.occupancy_grid_2d}')
                    # self.get_logger().info(f'frontier list = {frontier_list}')
                    
                    if invalid:
                        self.get_logger().info('Invalid section')
                        goal, info = self.frontierSelect(frontier_list, x ,y, alpha = 1, gamma = 0.5)
                        # self.wallFollower(duration=4)
                        invalid = False
                        # cmd_vel = Twist()
                        # cmd_vel.linear.x = 0.0
                        # cmd_vel.angular.z = 0.0
                        # self.cmd_vel_pub.publish(cmd_vel)
                        
                        
                        # self.get_logger().info('Change to wall follower')
                        # self.wallFollower(duration = 8)
                        
                        
                        # invalid = False
                        # status = 0
                        # self.map_info = None
                        # self.occupancy_grid_2d = None
                        # goal = None
                        # time.sleep(1)
                        # continue
                        
                        
                        
                        
                    else:
                        goal, info = self.frontierSelect(frontier_list, x ,y)
                        # goal = self.farthestFrontier(frontier_list, x ,y)
                    
                        self.get_logger().info(f'Valid info gain = {info}')
                    
                    if goal is None:
                        self.get_logger().info(f'Goal is None')
                        # invalid = True
                        # cmd_vel = Twist()
                        # cmd_vel.linear.x = 0.0
                        # cmd_vel.angular.z = np.pi / 10
                        # self.cmd_vel_pub.publish(cmd_vel)
                        goal_x, goal_y = self.prevgoal
                        # continue
                    else:
                        goal_x, goal_y = self.convertFrames(goal)
                    
                    if np.sqrt((goal_x -self.prevgoal[0]) ** 2 + (goal_y - self.prevgoal[1]) ** 2) < 0.3 or info <= 8:
                    # if info <= 24:
                        """
                        call wall follower and travel for 3 seconds.
                        return and 
                        """
                        cmd_vel = Twist()
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.0
                        self.cmd_vel_pub.publish(cmd_vel)
                        
                        
                        self.get_logger().info('Change to wall follower')
                        self.wallFollower(duration = 10)
                        
                        
                        status = 0
                        self.map_info = None
                        self.occupancy_grid_2d = None
                        goal = None
                        invalid = False

                        time.sleep(3)
                        continue
                    
                    
                    # if goal is None:
                        
                    #     cmd_vel = Twist()
                    #     cmd_vel.linear.x = 0.0
                    #     cmd_vel.angular.z = np.pi / 10
                    #     self.cmd_vel_pub.publish(cmd_vel)
                        
                    else:
                            
                        # if np.sqrt((goal_x - self.prevgoal[0]) ** 2 + (goal_y - self.prevgoal[1]) ** 2) < 0.23:

                        #     self.get_logger().info(f'Force finding the farthest valid goal')
                        #     goal = self.farthestFrontier(frontier_list, x ,y)
                        #     a = Astar(self.map_info, self.occupancy_grid_2d, (self.x_idx, self.y_idx), goal)
                        #     self.path = a.solve()
                        #     self.path_pub.publish(self.path)
                        #     status = 1
                        #     self.prevIdx = 0
                        #     self.prevgoal = self.convertFrames(goal)
                        
                        
                        # else:
                            self.get_logger().info(f'Found a valid goal')

                            cmd_vel = Twist()
                            cmd_vel.linear.x = 0.0
                            cmd_vel.angular.z = 0.0
                            self.cmd_vel_pub.publish(cmd_vel)
                            self.prevgoal = self.convertFrames(goal)
                            
                            
                            
                            
                            # fig, ax = plt.subplots(dpi=100)
                            
                            # cmap = mcolors.ListedColormap(['gray', 'white', 'black'])
                            # bounds = [-1.5, -0.5, 50, 150]
                            # norm = mcolors.BoundaryNorm(bounds, cmap.N)
                            # im = ax.imshow(self.occupancy_grid_2d, cmap=cmap, norm=norm)
                            # cbar = plt.colorbar(im, ticks=[-1, 0, 100])
                            # cbar.ax.set_yticklabels(['Unknown (-1)', 'Free (0)', 'Occupied (100)'])
                            
                            # plt.imshow(self.occupancy_grid_2d)
                            # plt.colorbar()
                            # plt.show()
                            
                            start_astar = time.time()
                            a = Astar(self.map_info, self.occupancy_grid_2d, (self.x_idx, self.y_idx), goal)
                            start_path = time.time()
                            self.get_logger().info(f'time for initializing Astar: {start_path - start_astar}')
                            path = a.solve()
                            self.get_logger().info(f'time for calculating path: {time.time() - start_path}')
                            
                            # caught exception
                            if len(path.poses) == 1:
                                self.get_logger().info('Invalid path, recalulating')
                                self.get_logger().info('Invalid path, recalulating')
                                self.path_pub.publish(path)
                                cmd_vel = Twist()
                                cmd_vel.linear.x = 0.0
                                cmd_vel.angular.z = 0.0
                                self.cmd_vel_pub.publish(cmd_vel)
                                status = 0
                                self.map_info = None
                                self.occupancy_grid_2d = None
                                self.path = None
                                None
                                invalid = True
                                time.sleep(1)
                                continue
                                                        
                            else:
                                self.path = path
                                self.path_pub.publish(self.path)
                                # self.get_logger().info(f'path: {self.path.poses}')
                                status = 1
                                self.prevIdx = 0
                                # prev_time = self.get_clock().now().nanoseconds*1e-9

                        
                else:
                    # x = self.x
                    # y = self.y
                    # yaw = self.yaw
                

                    s1 = False
                    idx = self.get_path_idx(x, y)
                    
                    if not self.path or idx >= len(self.path.poses):
                        self.get_logger().warn(f"Invalid path access: idx {idx}, path size {len(self.path.poses)}")
                        continue  # or handle safely

                    current_goal = self.path.poses[idx]
                    # self.get_logger().info(f'goal = {current_goal.pose.position}, idx = {idx}')
                    
                    if idx == len(self.path.poses) - 1:
                        s1 = True
                        # self.get_logger().info("STATUS CHANGED!!!")
                    
                    speed, heading, s2 = self.path_follower(x, y, yaw, current_goal)
                    
                    if s1 and s2:
                        status = 0
                        self.get_logger().info("STATUS CHANGED!!!")
                        cmd_vel = Twist()
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.3
                        self.cmd_vel_pub.publish(cmd_vel)

                        # timeout = time.time() + 1
                        # while time.time() < timeout:
                        #     rclpy.spin_once(self, timeout_sec=0.1)
                        #     self.map_info = None
                        #     self.occupancy_grid_2d = None
                        
                        self.map_info = None
                        self.occupancy_grid_2d = None
                        time.sleep(1.5)

                    else:
                        # self.get_logger().info(f'speed = {speed}, heading = {heading}')
                        self.move_robot(speed, heading)

            
            # time.sleep(1.0)
            

    
class Astar():
    def __init__(self, map_info, occupancy_grid_2d, start, goal):
        self.map_info = map_info
        self.occupancy_grid_2d = occupancy_grid_2d   #[y][x]
        
        self.start = start  #(x, y)
        self.goal = goal    #(x, y) rviz
        
        # self.q = Queue_astar()
        self.dist = np.full((map_info.height, map_info.width), np.inf)     #[y][x]
        # self.h = np.zeros((map_info.height, map_info.width))                #[y][x]
        self.h = np.full((map_info.height, map_info.width), np.inf) 
        self.via = np.zeros((map_info.height, map_info.width), dtype = object)              #[y][x]
        
        goal_node_x = round( (goal[0] - self.map_info.origin.position.x) / self.map_info.resolution)
        goal_node_y = round( (goal[1] - self.map_info.origin.position.y) / self.map_info.resolution)

        
        for y in range(map_info.height):
            for x in range(map_info.width):
                if self.occupancy_grid_2d[y][x] == 0:
                    # robot_x, robot_y = self.convertFrames((x, y))
                    h_score = np.sqrt((goal_node_x - x) ** 2 + (goal_node_y - y) ** 2)
                    

                    # for dy in range(-inflate_size, inflate_size + 1):


                    #         if 0 <= ny < map_info.height and 0 <= nx < map_info.width:
                    #             if self.occupancy_grid_2d[ny][nx] == -1:  # unknown
                    #                 h_score = 1000
                    #                 break  # increase once and break to avoid overcounting
                    
                    # inflate_size = 2
                    # for xn, yn in self.adjacentNeighbors((x, y), size = inflate_size):
                    #     if self.occupancy_grid_2d[yn][xn] == -1:
                    #         h_score = np.inf
                    #         break

                    self.h[y][x] = h_score
                    
        ####################################
        
        # fig, ax = plt.subplots(dpi=100)
    
        # # 1. Mask inf values
        # masked_h = np.ma.masked_where(np.isinf(self.h), self.h)

        # # 2. Define a colormap for normal (non-inf) values
        # finite_cmap = plt.cm.viridis  # or 'plasma', 'inferno', etc.

        # # 3. Plot non-inf values
        # finite_im = ax.imshow(masked_h, cmap=finite_cmap)

        # # 4. Overlay inf cells as solid color (e.g. black)
        # inf_mask = np.isinf(self.h).astype(float)
        # ax.imshow(np.ma.masked_where(inf_mask == 0, inf_mask), cmap='gray', alpha=1.0)

        # # 5. Add colorbar for finite values
        # plt.colorbar(finite_im, ax=ax)

        # plt.title("Heuristic Map (h) with inf in Gray")
        # plt.show()


        
    def __get_f_score(self, node):
        return self.dist[node[1]][node[0]] + self.h[node[1]][node[0]]
    
    def __get_g_score(self, start, end):
        return abs(start[0] - end[0]) + abs(start[1] - end[1])
    
    def adjacentNeighbors(self, node, size = 1):
        # (x, y)
        x = node[0]
        y = node[1]
        # directions = [(-1, 0), (0, -1), (0, 1), (1, 0)]
        directions = [(dx, dy) for dx in range(-size, size + 1) for dy in range(-size, size + 1) if not (dx == 0 and dy ==0)]
        
        neighbors = list()
        
        for dx, dy in directions:
            if (x + dx) >= 0 and (x + dx) < self.map_info.width and (y + dy) >= 0 and (y + dy) < self.map_info.height:
                
                neighbors.append((x + dx, y + dy))
        
        return neighbors
        
    # def solve(self):
    #     # start = time.time()
    #     sn = self.start
    #     en = self.goal
        
    #     # if self.occupancy_grid_2d[sn[1]][sn[0]] != 0 or self.occupancy_grid_2d[en[1]][en[0]] != 0:
            
    #     #     # self.get_logger().info("Start or goal is in an obstacle or unknown space!")
    #     #     path = Path()
    #     #     path.header.stamp = Time()
    #     #     path.header.frame_id = 'map'
    #     #     return path
        
        
    #     self.dist[sn[1]][sn[0]] = 0
    #     self.q.push(sn)
        
    #     while len(self.q) > 0:
    #         self.q.sort(key = self.__get_f_score)
    #         u = self.q.pop()

    #         if u == en:
    #             break
    #         children = self.adjacentNeighbors(u, size = 1)
            
    #         for child in children:
                
    #             if self.occupancy_grid_2d[child[1]][child[0]] != 0:
    #                 continue
                
    #             g_score = self.dist[u[1]][u[0]] + self.__get_g_score(u, child)
                
    #             if g_score < self.dist[child[1]][child[0]]:
    #                 self.dist[child[1]][child[0]] = g_score
    #                 self.via[child[1]][child[0]] = u

    #                 if child in self.q.queue:
    #                     self.q.queue.remove(child)
    #                 self.q.push(child)
        
    #     path = self.reconstruct_path()
    #     return path
    
    def solve(self):
        sn, en = self.start, self.goal
        self.dist[sn[1]][sn[0]] = 0
        open_heap = []
        heapq.heappush(open_heap, (self.__get_f_score(sn), sn))
        visited = set()
        
        while open_heap:
            _, u = heapq.heappop(open_heap)
            if u == en:
                break
            if u in visited:
                continue
            visited.add(u)

            for child in self.adjacentNeighbors(u):
                if self.occupancy_grid_2d[child[1]][child[0]] != 0:
                    continue

                tentative_g = self.dist[u[1]][u[0]] + self.__get_g_score(u, child)
                if tentative_g < self.dist[child[1]][child[0]]:
                    self.dist[child[1]][child[0]] = tentative_g
                    self.via[child[1]][child[0]] = u
                    heapq.heappush(open_heap, (self.__get_f_score(child), child))

        return self.reconstruct_path()
        
    
        
    def reconstruct_path(self):
        path = Path()
        
        current = self.goal
        
        path.header.stamp = Time()
        path.header.frame_id = 'map'
        
        while current != 0:
            currentNode = PoseStamped()
            x, y = self.convertFrames(current)
            currentNode.pose.position.x = x
            currentNode.pose.position.y = y
            currentNode.pose.position.z = 0.0

            currentNode.pose.orientation.x=0.0 
            currentNode.pose.orientation.y=0.0
            currentNode.pose.orientation.z=0.0
            currentNode.pose.orientation.w=1.0 
            currentNode.header.stamp=Time()      # change accordingly
            currentNode.header.frame_id='map'
            path.poses.append(currentNode)
            current = self.via[current[1]][current[0]]
            
        path.poses.reverse() 
        
        return path
    
    def convertFrames(self, node):
        # convert from node frame into Rviz frame
        
        x = round(node[0] * self.map_info.resolution + self.map_info.origin.position.x, 2)
        y = round(node[1] * self.map_info.resolution + self.map_info.origin.position.y, 2)
            
        return x, y


# class Queue_astar():
#     def __init__(self, init_queue = []):
#         self.queue = copy(init_queue)
#         self.start = 0
#         self.end = len(self.queue)-1
    
#     def __len__(self):
#         numel = len(self.queue)
#         return numel
    
#     def __repr__(self):
#         q = self.queue
#         tmpstr = ""
#         for i in range(len(self.queue)):
#             flag = False
#             if(i == self.start):
#                 tmpstr += "<"
#                 flag = True
#             if(i == self.end):
#                 tmpstr += ">"
#                 flag = True
            
#             if(flag):
#                 tmpstr += '| ' + str(q[i]) + '|\n'
#             else:
#                 tmpstr += ' | ' + str(q[i]) + '|\n'
            
#         return tmpstr
    
#     def __call__(self):
#         return self.queue
    
#     def initialize_queue(self,init_queue = []):
#         self.queue = copy(init_queue)
    
#     def sort(self,key=str.lower):
#         self.queue = sorted(self.queue,key=key)
        
#     def push(self,data):
#         self.queue.append(data)
#         self.end += 1
    
#     def pop(self):
#         p = self.queue.pop(self.start)
#         self.end = len(self.queue)-1
#         return p



def main(args=None):
    rclpy.init(args=args)
    task1 = Task1()

    try:
        task1.run()
    except KeyboardInterrupt:
        pass
    finally:
        task1.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



# ###
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path, Odometry
from geometry_msgs.msg import Pose, Point, PoseStamped, Twist
import rclpy.time
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time

import numpy as np
from queue import Queue
import time
from copy import copy
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import heapq

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



# Import other python packages that you think necessary


"""
To-dos
1. subscribe to /map topic and extract Occupancy grid
2. process the Occupancy grid
3. frontier detection
4. select frontier goal
5. navigate to the selected goal (+ collision avoidance)
6. repeat 3-5 until all the unknown areas are eliminated
"""


class Task1(Node):
    """
    Environment mapping task.
    """
    def __init__(self):
        super().__init__('task1_node')
        self.timer = self.create_timer(0.1, self.timer_cb)
        # Fill in the initialization member variables that you need
        
        # create a tf2 listener to extract robot's current location
        # https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.create_timer(self.fetch_robot_position, 0.5)
        # time.sleep(1.0)
        
        # subscriber
        self.subscriber1 = self.create_subscription(OccupancyGrid, '/map', self.dataProcess, 10)
        self.subscriber2 = self.create_subscription(Odometry, '/odom', self.fetch_robot_position_odom, 10)
        self.subscriber3 = self.create_subscription(LaserScan, '/scan', self.scan, 10)
        
        # publisher
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # self.create_timer(0.1, self.fetch_robot_position)
        
        self.map_info = None
        # self.map_resolution = None
        # self.map_width = None
        # self.map_height = None
        # self.map_origin = None
        self.occupancy_grid_2d = None   #[y][x]
        self.x_idx = None
        self.y_idx = None
        # self.x = None
        # self.y = None
        # self.yaw = None
        self.path = None
        self.prevIdx = 0
        self.prevHeadingError = 0
        self.prevTime = 0
        self.prev_heading = 0
        self.prevgoal = (0, 0)
        self.inflated = False
        self.initial = 0
        self.samegoal = 0
        self.scan_data = None
        
        
        
    # def fetch_robot_position(self):
    #     # return (x, y)
        
    #     try:
    #         t = self.tf_buffer.lookup_transform(
    #             target_frame = 'map',
    #             source_frame = 'base_link',
    #             time = rclpy.time.Time(),
    #             timeout = rclpy.duration.Duration(seconds=1.0))
    #         robot_x = t.transform.translation.x
    #         robot_y = t.transform.translation.y
            
    #         qx = t.transform.rotation.x
    #         qy = t.transform.rotation.y
    #         qz = t.transform.rotation.z
    #         qw = t.transform.rotation.w
            
    #         yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
            
    #         # self.get_logger().info(f'CURRENT POSITION x = {robot_x}, y = {robot_y}, yaw = {yaw * 180 / np.pi}')
            
    #         # self.x = robot_x
    #         # self.y = robot_y
    #         # self.yaw = yaw
            
    #         return robot_x, robot_y, yaw
        
    #     except TransformException as ex:
    #         self.get_logger().info(f'TF failed: {ex}')
    #         return None, None, None
        
    def fetch_robot_position_odom(self, msg):
        # self.get_logger().info('Received new position')
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        
        # self.get_logger().info(f'CURRENT POSITION x = {self.x}, y = {self.y}, yaw = {self.yaw * 180 / np.pi}')
           
    
        
    def quaternion_to_yaw(self, qx, qy, qz, qw): 
        q_length = np.sqrt( qx ** 2 + qy ** 2 + qz ** 2 + qw ** 2) 
        qx_norm = qx / q_length
        qy_norm = qy / q_length
        qz_norm = qz / q_length
        qw_norm = qw / q_length
        numerator = 2 * (qw_norm * qz_norm + qx_norm * qy_norm)
        denominator = 1 - 2 * (qy_norm ** 2 + qz_norm ** 2)
        yaw = np.arctan2( numerator ,denominator)
        return yaw

    
    def scan(self, msg):
        if msg is not None:
            self.scan_data = msg
            
            # length = len(msg.ranges)
            # self.get_logger().info(f'length: {length}, angle {msg.angle_min, msg.angle_max}')
            # self.get_logger().info(f'dist {msg.range_min, msg.range_max}')    (0.2, 5.0)
            # self.get_logger().info(f'{msg.angle_increment}')


    def timer_cb(self):
        self.get_logger().info('Task1 node is alive.', throttle_duration_sec=1)
        # Feel free to delete this line, and write your algorithm in this callback function

    # Define function(s) that complete the (automatic) mapping task
    
    def dataProcess(self, msg):
        # self.get_logger().info('Received new occupancy grid')
        self.inflated = False
        
        self.map_info = msg.info
        # map_resolution = self.map_info.resolution
        # map_width = self.map_info.width
        # map_height = self.map_info.height
        # map_origin = self.map_info.origin
        occupancy_grid = msg.data   # remember to convert to world frame when navigating

        self.occupancy_grid_2d = np.array(occupancy_grid, dtype=int).reshape((self.map_info.height, self.map_info.width))
        # path = Path()
        # path.header.stamp = Time()
        # path.header.frame_id = 'map'
        # self.path_pub.publish(path)
        # cmd_vel = Twist()
        # cmd_vel.linear.x = 0.0
        # cmd_vel.angular.z = 0.4
        # self.cmd_vel_pub.publish(cmd_vel)
        # status = 0

    # def inflate_map(self, size):
    #     start = time.time()
    #     original = self.occupancy_grid_2d.copy()
        
    #     for y in range(self.map_info.height):
    #         for x in range(self.map_info.width):
                
    #             if original[y][x] == 100:
    #             # if self.occupancy_grid_2d[y][x] > 0:
    #                 # self.occupancy_grid_2d[y][x] = 100
    #                 neighbors = self.adjacentNeighbors(x, y, size=size)
                    
    #                 for x_n, y_n in neighbors:
    #                     if original[y_n][x_n] != -1:
    #                         self.occupancy_grid_2d[y_n][x_n] = 100
                            
                            
                            
    #             ###################################
    #             if original[y][x] == 0:
    #                 neighbors = self.adjacentNeighbors(x, y, size = 2)
                    
    #                 for x_n, y_n in neighbors:
    #                     if original[y_n][x_n] == -1:
    #                         self.occupancy_grid_2d[y][x] = -1
    #                         break
                    
                
    #     self.inflated = True
    #     self.get_logger().info(f'time for inflating map : {time.time()- start}')
    #     # self.get_logger().info(f'inflated map : {self.occupancy_grid_2d}')
        
        
    def inflate_map(self, size):
        start = time.time()
        original = self.occupancy_grid_2d.copy()
        width = self.map_info.width
        height = self.map_info.height
        grid = self.occupancy_grid_2d
        # First pass: inflate obstacles (100)
        for y in range(height):
            for x in range(width):
                if original[y][x] == 100:
                    for dx in range(-size, size + 1):
                        for dy in range(-size, size + 1):
                            nx, ny = x + dx, y + dy
                            if (
                                0 <= nx < width and 0 <= ny < height
                                and original[ny][nx] != -1
                            ):
                                grid[ny][nx] = 100
        # Second pass: mark unknown if surrounded by unknown
        # for y in range(height):
        #     for x in range(width):
        #         if original[y][x] == 0:
        #             for dx in range(-2, 3):
        #                 for dy in range(-2, 3):
        #                     nx, ny = x + dx, y + dy
        #                     if 0 <= nx < width and 0 <= ny < height:
        #                         if original[ny][nx] == -1:
        #                             grid[y][x] = -1
        #                             break
        self.inflated = True
        self.get_logger().info(f'time for inflating map : {time.time() - start:.4f} seconds')


        
    def wfd(self):
        start = time.time()
        # frontier selection (Wavefront frontier detection)
        # https://arxiv.org/abs/1806.03581
        # https://arxiv.org/pdf/1806.03581
        # robot_x, robot_y = self.fetch_robot_position()
        
        # x_occup_idx = round((robot_x - self.map_info.origin.position.x) / self.map_info.resolution)
        # y_occup_idx = round((robot_y - self.map_info.origin.position.y) / self.map_info.resolution)
        x_occup_idx = self.x_idx
        y_occup_idx = self.y_idx
        
        
        # UNLABELED = 0
        # MAP_OPEN_LIST = 1
        # MAP_CLOSE_LIST = 2
        # FRONTIER_OPEN_LIST = 3
        # FRONTIER_CLOSE_LIST = 4
        labeled_occupancy_grid_2d = np.zeros((self.map_info.height, self.map_info.width), dtype=int)        #[y][x]
        
        # (x, y)
        outter_queue = Queue()
        frontier_list = list()
        
        # enqueue current robot position and mark as "MAP_OPEN_LIST"
        outter_queue.put((x_occup_idx, y_occup_idx))
        labeled_occupancy_grid_2d[y_occup_idx][x_occup_idx] = 1
        
        while not outter_queue.empty():
            x, y = outter_queue.get()
            
            if labeled_occupancy_grid_2d[y][x] == 2:
                continue
            
            if self.isFrontier(x, y):
                # (x, y)
                inner_queue = Queue()
                frontier_list_temp = list()
                
                inner_queue.put((x, y))
                labeled_occupancy_grid_2d[y][x] = 3
                
                while not inner_queue.empty():
                    x_i, y_i = inner_queue.get()
                    
                    if labeled_occupancy_grid_2d[y_i][x_i] == 2 or labeled_occupancy_grid_2d[y_i][x_i] == 4:
                        continue
                    
                    if self.isFrontier(x_i, y_i):
                        frontier_list_temp.append((x_i, y_i))
                        
                        neighbors = self.adjacentNeighbors(x_i, y_i)
                        if len(neighbors) > 0:
                            for x_ii, y_ii in neighbors:
                                
                                # [MAP_OPEN_LIST, FRONTIER_OPEN_LIST, FRONTIER_CLOSE_LIST]
                                if labeled_occupancy_grid_2d[y_ii][x_ii] not in [1, 3, 4]:
                                    inner_queue.put((x_ii, y_ii))
                                    labeled_occupancy_grid_2d[y_ii][x_ii] = 3

                    labeled_occupancy_grid_2d[y_i][x_i] = 4

                if len(frontier_list_temp) > 0:
                    frontier_list.append(frontier_list_temp)
                    for x_f, y_f in frontier_list_temp:
                        labeled_occupancy_grid_2d[y_f][x_f] = 2
            
            neighbors = self.adjacentNeighbors(x ,y)
            if len(neighbors) > 0:
                for x_n, y_n in neighbors:
                    if labeled_occupancy_grid_2d[y_n][x_n] not in [1,2] and self.freeNeighbor(x_n, y_n):
                        outter_queue.put((x_n, y_n))
                        labeled_occupancy_grid_2d[y_n][x_n] = 1
                        
            labeled_occupancy_grid_2d[y][x] = 2
        
        self.get_logger().info(f'time for detecting frontiers : {time.time()- start}')
        return frontier_list
                                           
        
    def isFrontier(self, x, y):
        # (x, y)
        if self.occupancy_grid_2d[y][x] != 0:
            return False
    
        neighbors = self.adjacentNeighbors(x, y)
                
        if len(neighbors) > 0:
            for x, y in neighbors:
                if self.occupancy_grid_2d[y][x] == -1:
                    return True 
        
        return False
    
    def adjacentNeighbors(self, x, y, size = 1):
        # (x, y)
        # directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        
        directions = [(dx, dy) for dx in range(-size, size + 1) for dy in range(-size, size + 1) if not (dx == 0 and dy ==0)]
        neighbors = list()
        
        for dx, dy in directions:
            if (x + dx) >= 0 and (x + dx) < self.map_info.width and (y + dy) >= 0 and (y + dy) < self.map_info.height:
                neighbors.append((x + dx, y + dy))
        
        return neighbors
    
    def freeNeighbor(self, x, y):
        neighbors = self.adjacentNeighbors(x, y)
        if len(neighbors) > 0:
            for x_n, y_n in neighbors:
                if self.occupancy_grid_2d[y_n][x_n] == 0:
                    return True
    
        return False
    
    
    ######################################################
    def frontierSelect(self, frontiers, robot_x, robot_y, alpha=1.0, gamma=1.5):
        start = time.time()
        dist = np.inf
        goal = None
        info = 0
        for frontier in frontiers:
            if len(frontier) <= 4:
                continue
            
            for fx, fy in frontier: 
                x, y = self.convertFrames((fx, fy))
                # self.get_logger().info(f'x, y ={fx, fy}, prevgoal = {self.prevgoal}')
                
                
                
                d = np.sqrt((x - robot_x)** 2 + (y - robot_y)** 2)
                
                info_gain = 0
                
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        nx = fx + dx
                        ny = fy + dy
                        if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
                            if self.occupancy_grid_2d[ny][nx] == -1:
                                info_gain += 1
                # self.get_logger().info(f'dist = {alpha * d}, info = {gamma * info_gain}')
                score = alpha * d - gamma * info_gain
                
                if score < dist:
                    goal = (fx, fy)
                    dist = score
                    info = info_gain
                   
        self.get_logger().info(f'time for selecting frontier goal : {time.time()- start}')    
        return goal, info
    
    

    # broken
    def farthestFrontier(self, frontiers, robot_x, robot_y, alpha=10, beta=2.0):
        best_score = 0
        best_goal = None
        
        for f in frontiers:
 
            for fx, fy in f:

                # Distance from robot
                x, y = self.convertFrames((fx, fy))
                dist = np.sqrt((x - robot_x) ** 2 + (y - robot_y) ** 2)

                # Info gain: count unknown neighbors around frontier
                info_gain = 0

                for dx in range(-3, 4):
                    for dy in range(-3, 4):
                        nx, ny = fx + dx, fy + dy
                        if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
        
                            if self.occupancy_grid_2d[ny][nx] == -1:  # unknown
                                info_gain += 100

            # Combined score
            # score = alpha * dist + beta * info_gain
            score = info_gain

            if score > best_score:
                best_score = score
                # Now find a valid free cell in this frontier closest to the centroid
                best_goal = (fx, fy)

                    
        return best_goal
        
    
        
    
    def get_path_idx(self, x, y):
        idx = -1
        # dist = np.inf
        idxRadius = 0.2
        
        for i in range(self.prevIdx, len(self.path.poses)):
            if (np.sqrt((x - self.path.poses[i].pose.position.x) ** 2 + (y - self.path.poses[i].pose.position.y) ** 2)) >= idxRadius:
                idx = i
                self.prevIdx = i
                break
            
        if idx == -1:
            idx = len(self.path.poses) - 1

        return idx

    def path_follower(self, x, y, yaw, current_goal):
        status = False
        # kp_speed = 1.8
        # kp_heading = 1.2
        kp_speed = 1.5
        kp_heading = 1.0
        
        # current_time = time.time()
        current_time = self.get_clock().now().nanoseconds * 1e-9
        while current_time == self.prevTime:
            rclpy.spin_once(self, timeout_sec=0.1)
            current_time = self.get_clock().now().nanoseconds

        
        error = np.sqrt(( x - current_goal.pose.position.x) ** 2 + ( y - current_goal.pose.position.y) ** 2)

        if error < 0.2:
            status = True
        
        speed = kp_speed * error
        
        heading_difference_vehicl_goal = np.arctan2(current_goal.pose.position.y - y , current_goal.pose.position.x - x )
        heading_error = heading_difference_vehicl_goal - yaw
        
        if heading_error > np.pi:
            heading_error = -2 * np.pi + heading_error
        elif heading_error < -np.pi:
            heading_error = 2 * np.pi + heading_error
        
        # self.get_logger().info(f'heading: {yaw * 180 / np.pi}, heading_error: {heading_error * 180 / np.pi }')
        
        # self.get_logger().info(f'current time: {current_time}, previous time: {self.prevTime}')
            
        # heading = kp_heading * heading_error + kd_heading* (heading_error - self.prevHeadingError) / (current_time - self.prevTime)
        heading = kp_heading * heading_error
        
        self.prevTime = current_time
        self.prevHeadingError = heading_error
        
        return speed, heading, status

    def move_robot(self, speed, heading):
        cmd_vel = Twist()
        
        if abs(self.prev_heading) >= (np.pi / 4) or abs(heading) >= (np.pi / 4):
            speed = 0.0
            self.prev_heading = heading
            heading = heading / 1
        
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        
        # self.get_logger().info(f'OUTPUT speed: {speed}, heading: {heading * 180 / np.pi}')
        
        self.cmd_vel_pub.publish(cmd_vel)
        
    def convertFrames(self, node):
        # convert from node frame into Rviz frame
        
        x = round(node[0] * self.map_info.resolution + self.map_info.origin.position.x, 2)
        y = round(node[1] * self.map_info.resolution + self.map_info.origin.position.y, 2)
            
        return (x, y)
    
    def enableLaserScan(self):
        self.scan_data = None
        # https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
        self.subscriber3 = self.create_subscription(LaserScan, '/scan', self.scan, 10)
    
    def wallFollower(self, duration = 3):
        start_time = time.time()
        # self.enableLaserScan()
        kp = 0.5

        
        while rclpy.ok() and time.time() < start_time + duration:
            rclpy.spin_once(self, timeout_sec= 0.05)
            speed = 0.15
            heading = 0.0
            if self.scan_data is None:
                continue
            
            range = np.array(self.scan_data.ranges)
            min_index = np.nanargmin(range)
            min_angle = (self.scan_data.angle_min + min_index * self.scan_data.angle_increment) * 180 / np.pi
            range = np.clip(range, 0, 3.0)
            
            min_range = range[min_index]
            
            front_sector = np.concatenate((range[-15:], range[:16]))
            # front_sector = np.clip(front_sector, 0, np.inf)
            front = np.mean(front_sector)
            right = np.mean(range[189:351])
            left = np.mean(range[9:171])
            
            # if front < 0.3:
            #     self.get_logger().info(f'front is too close')
            #     speed = -0.2
            #     cmd_vel = Twist()
            #     cmd_vel.linear.x = speed
            #     self.cmd_vel_pub.publish(cmd_vel)
            #     continue
            

            if min_range <= 1.0:
                # self.get_logger().info(f'obstacle detected in direction = {min_range}')
                cmd_vel = Twist()
                # cmd_vel.linear.x = 0.05
                # cmd_vel.angular.z = -0.5
                if (min_angle <= 60 or min_angle >= 300) and (min_range <= 0.4):
                    self.get_logger().info(f'front is being too close')
                    cmd_vel.linear.x = -0.15
                    if left > right:
                        cmd_vel.angular.z = -0.2
                    else:
                        cmd_vel.angular.z = 0.2
                    self.cmd_vel_pub.publish(cmd_vel)
                    time.sleep(0.3)
                    continue
                self.get_logger().info(f'obstacle detected in = {min_range}')
                if 0<= min_angle <= 90:
                    cmd_vel.linear.x = 0.2
                    # cmd_vel.angular.z = 0.3 *  (min_index - 180) * np.pi / 180
                    cmd_vel.angular.z = 0.65*  (min_angle - 90) * np.pi / 180
                elif 90< min_angle <= 180:
                    cmd_vel.linear.x = 0.2
                    cmd_vel.angular.z = 0.65 *  (min_angle - 90) * np.pi / 180
                elif 180< min_angle <= 270:
                    cmd_vel.linear.x = 0.2
                    cmd_vel.angular.z = 0.7 *  (min_angle - 270) * np.pi / 180
                elif 270 < min_angle <= 360:
                    cmd_vel.linear.x = 0.2
                    cmd_vel.angular.z = 0.7 *  (min_angle - 270) * np.pi / 180
                
                self.cmd_vel_pub.publish(cmd_vel)
                # time.sleep(1)
                continue
            

            
            elif right > 2 and left > 2 and front > 2:
                self.get_logger().info(f'no angle adjustment')
                side = 0
                side_check = 1
                if right > left:
                    heading = -0.1
                elif right < left:
                    heading = 0.1
                else:
                    heading = 0.0
                speed = 0.3
                         
            cmd_vel = Twist()
            cmd_vel.linear.x = speed
            cmd_vel.angular.z = heading
            self.cmd_vel_pub.publish(cmd_vel)
            
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info("Wall following complete.")
        # self.destroy_subscription(self.subscriber3)

    def run(self):
        # status:
        # 0 : Reaches the goal point, need to select a new frontier goal
        # 1 : Navigate to the selected frontier goal point 
        
        status = 0
        # prev_heading = 0
        # prev_time = 0
        times = 180
        start_time = time.time()
        invalid = False
        
        while rclpy.ok():
            """
            1. spin 360
            2. get the occucpancy grid 
            3. calculate each row's medains from the frontier matrix
            4. choose the cloestest one
            5. navigate to the goal
            6. repeat 1-5 until there isn't any unknown area
            """
            rclpy.spin_once(self, timeout_sec= 1.0)
            rclpy.spin_once(self, timeout_sec= 1.0)
            
            if self.map_info is not None:
                
                            
                if self.initial < times:
                
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.25
                    cmd_vel.angular.z = 2.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    self.initial += 1
                    # self.get_logger().info("BOOM!!!")
                    continue
                
                elif self.initial == times:
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    self.initial += 1

                    self.map_info = None
                    self.occupancy_grid_2d = None
                    time.sleep(2)
                    continue
                
                
                x = self.x
                y = self.y
                yaw = self.yaw
                
                
                # self.get_logger().info(f'CURRENT POSITION x = {self.x}, y = {self.y}, yaw = {self.yaw * 180 / np.pi}')
                
            
                if status == 0:
                    
                    
                    self.x_idx = round((x - self.map_info.origin.position.x) / self.map_info.resolution)
                    self.y_idx = round((y - self.map_info.origin.position.y) / self.map_info.resolution)
                    
                    # (x,y)
                    if not self.inflated:
                        ####################
                        self.inflate_map(size = 5)
                        # self.get_logger().info(str(type(self.occupancy_grid_2d)))

                    
                    
                    frontier_list = self.wfd()
                    
                    
                    if invalid:
                        self.get_logger().info('Invalid')
                        goal, info = self.frontierSelect(frontier_list, x ,y, alpha = 1, gamma = 0.4)
                        # self.wallFollower(duration=2)
                        invalid = False
                        
                    else:
                        goal, info = self.frontierSelect(frontier_list, x ,y)
                        # goal = self.farthestFrontier(frontier_list, x ,y)
                    
                        self.get_logger().info(f'Valid info gain = {info}')
                    
                    if goal is None:
                        self.get_logger().info(f'Goal is None')
                        goal_x, goal_y = self.prevgoal
                        
                    else:
                        goal_x, goal_y = self.convertFrames(goal)
                    
                    # if np.sqrt((goal_x -self.prevgoal[0]) ** 2 + (goal_y - self.prevgoal[1]) ** 2) < 0.3 or info <= 8:
                    if info <= 6:
                        """
                        call wall follower and travel for 3 seconds.
                        return and 
                        """
                        cmd_vel = Twist()
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.0
                        self.cmd_vel_pub.publish(cmd_vel)
                        
                        
                        self.get_logger().info('Change to wall follower')
                        self.wallFollower(duration = 10)
                        
                        
                        status = 0
                        self.map_info = None
                        self.occupancy_grid_2d = None
                        goal = None
                        invalid = False

                        time.sleep(3)
                        continue
                    
                        
                    else:
                
                        self.get_logger().info(f'Found a valid goal')
                        cmd_vel = Twist()
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.0
                        self.cmd_vel_pub.publish(cmd_vel)
                        self.prevgoal = self.convertFrames(goal)
                        
                        
                        
                        start_astar = time.time()
                        a = Astar(self.map_info, self.occupancy_grid_2d, (self.x_idx, self.y_idx), goal)
                        start_path = time.time()
                        self.get_logger().info(f'time for initializing Astar: {start_path - start_astar}')
                        path = a.solve()
                        self.get_logger().info(f'time for calculating path: {time.time() - start_path}')
                        
                        # caught exception
                        if len(path.poses) == 1:
                            self.get_logger().info('Invalid path, recalulating')
                            self.get_logger().info('Invalid path, recalulating')
                            self.path_pub.publish(path)
                            cmd_vel = Twist()
                            cmd_vel.linear.x = 0.0
                            cmd_vel.angular.z = 0.0
                            self.cmd_vel_pub.publish(cmd_vel)
                            status = 0
                            self.map_info = None
                            self.occupancy_grid_2d = None
                            self.path = None
                            invalid = True
                            time.sleep(1)
                            continue
                                                    
                        else:
                            self.path = path
                            self.path_pub.publish(self.path)
                            # self.get_logger().info(f'path: {self.path.poses}')
                            status = 1
                            self.prevIdx = 0
                            # prev_time = self.get_clock().now().nanoseconds*1e-9

                        
                else:
                    # x = self.x
                    # y = self.y
                    # yaw = self.yaw

                    range = np.array(self.scan_data.ranges)
                    min_index = np.nanargmin(range)
                    min_range = range[min_index]
                    # self.get_logger().info(f'min_range = {min_range}')
                    
                    if min_range <= 0.22:
                        cmd_vel = Twist()
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.0
                        self.cmd_vel_pub.publish(cmd_vel)
                        self.map_info = None
                        self.occupancy_grid_2d = None
                        self.path = None
                        invalid = True
                        status = 0
                        self.get_logger().info(f'Too close to obstacles')
                        self.wallFollower()
                        time.sleep(0.5)
                        continue

                    s1 = False
                    idx = self.get_path_idx(x, y)
                    
                    if not self.path or idx >= len(self.path.poses):
                        self.get_logger().warn(f"Invalid path access: idx {idx}, path size {len(self.path.poses)}")
                        continue  # or handle safely

                    current_goal = self.path.poses[idx]
                    # self.get_logger().info(f'goal = {current_goal.pose.position}, idx = {idx}')
                    
                    if idx == len(self.path.poses) - 1:
                        s1 = True
                        # self.get_logger().info("STATUS CHANGED!!!")
                    
                    speed, heading, s2 = self.path_follower(x, y, yaw, current_goal)
                    
                    if s1 and s2:
                        status = 0
                        self.get_logger().info("STATUS CHANGED!!!")
                        cmd_vel = Twist()
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.3
                        self.cmd_vel_pub.publish(cmd_vel)
                        
                        self.map_info = None
                        self.occupancy_grid_2d = None
                        time.sleep(1.5)

                    else:
                        self.move_robot(speed, heading)

            
            # time.sleep(1.0)
            

    
class Astar():
    def __init__(self, map_info, occupancy_grid_2d, start, goal):
        self.map_info = map_info
        self.occupancy_grid_2d = occupancy_grid_2d   #[y][x]
        
        self.start = start  #(x, y)
        self.goal = goal    #(x, y) rviz
        
        # self.q = Queue_astar()
        self.dist = np.full((map_info.height, map_info.width), np.inf)     #[y][x]
        # self.h = np.zeros((map_info.height, map_info.width))                #[y][x]
        self.h = np.full((map_info.height, map_info.width), np.inf) 
        self.via = np.zeros((map_info.height, map_info.width), dtype = object)              #[y][x]
        
        goal_node_x = round( (goal[0] - self.map_info.origin.position.x) / self.map_info.resolution)
        goal_node_y = round( (goal[1] - self.map_info.origin.position.y) / self.map_info.resolution)

        
        for y in range(map_info.height):
            for x in range(map_info.width):
                if self.occupancy_grid_2d[y][x] == 0:
                    h_score = np.sqrt((goal_node_x - x) ** 2 + (goal_node_y - y) ** 2)
                    self.h[y][x] = h_score



        
    def __get_f_score(self, node):
        return self.dist[node[1]][node[0]] + self.h[node[1]][node[0]]
    
    def __get_g_score(self, start, end):
        return abs(start[0] - end[0]) + abs(start[1] - end[1])
    
    def adjacentNeighbors(self, node, size = 1):
        # (x, y)
        x = node[0]
        y = node[1]
        # directions = [(-1, 0), (0, -1), (0, 1), (1, 0)]
        directions = [(dx, dy) for dx in range(-size, size + 1) for dy in range(-size, size + 1) if not (dx == 0 and dy ==0)]
        
        neighbors = list()
        
        for dx, dy in directions:
            if (x + dx) >= 0 and (x + dx) < self.map_info.width and (y + dy) >= 0 and (y + dy) < self.map_info.height:
                
                neighbors.append((x + dx, y + dy))
        
        return neighbors
    
    def solve(self):
        sn, en = self.start, self.goal
        self.dist[sn[1]][sn[0]] = 0
        open_heap = []
        heapq.heappush(open_heap, (self.__get_f_score(sn), sn))
        visited = set()
        
        while open_heap:
            _, u = heapq.heappop(open_heap)
            if u == en:
                break
            if u in visited:
                continue
            visited.add(u)

            for child in self.adjacentNeighbors(u):
                if self.occupancy_grid_2d[child[1]][child[0]] != 0:
                    continue

                tentative_g = self.dist[u[1]][u[0]] + self.__get_g_score(u, child)
                if tentative_g < self.dist[child[1]][child[0]]:
                    self.dist[child[1]][child[0]] = tentative_g
                    self.via[child[1]][child[0]] = u
                    heapq.heappush(open_heap, (self.__get_f_score(child), child))

        return self.reconstruct_path()
        
    
        
    def reconstruct_path(self):
        path = Path()
        
        current = self.goal
        
        path.header.stamp = Time()
        path.header.frame_id = 'map'
        
        while current != 0:
            currentNode = PoseStamped()
            x, y = self.convertFrames(current)
            currentNode.pose.position.x = x
            currentNode.pose.position.y = y
            currentNode.pose.position.z = 0.0

            currentNode.pose.orientation.x=0.0 
            currentNode.pose.orientation.y=0.0
            currentNode.pose.orientation.z=0.0
            currentNode.pose.orientation.w=1.0 
            currentNode.header.stamp=Time()      # change accordingly
            currentNode.header.frame_id='map'
            path.poses.append(currentNode)
            current = self.via[current[1]][current[0]]
            
        path.poses.reverse() 
        
        return path
    
    def convertFrames(self, node):
        # convert from node frame into Rviz frame
        
        x = round(node[0] * self.map_info.resolution + self.map_info.origin.position.x, 2)
        y = round(node[1] * self.map_info.resolution + self.map_info.origin.position.y, 2)
            
        return x, y


def main(args=None):
    rclpy.init(args=args)
    task1 = Task1()

    try:
        task1.run()
    except KeyboardInterrupt:
        pass
    finally:
        task1.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
