#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Import other python packages that you think necessary
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Pose
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time
import heapq
import numpy as np
import os
import yaml
from PIL import Image, ImageOps
import pandas as pd
from copy import copy
import matplotlib.pyplot as plt
import time
from ament_index_python.packages import get_package_share_directory

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from scipy.ndimage import binary_dilation


# for both static map and dynamic map, 0 means occupied and 1 means free

class Task2(Node):
    """
    Environment localization and navigation task.
    """
    def __init__(self):
        super().__init__('task2_node')
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.map_info = None
        # self.enableOccupanctGrid()
        # self.map()
        # Fill in the initialization member variables that you need
        self.static_map = None
        # self.static_map = Map('/home/albert/sim_ws/src/turtlebot3_gazebo/maps/map')
        # self.static_map.inflate_map(pad = 2)    # p is the inflated size
        
        qos_map = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        
        # self.subscriber1 = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        # self.subscriber2 = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)
        # self.subscriber3 = self.create_subscription(LaserScan, '/scan', self.scan, 10)
        self.subscriber4 = self.create_subscription(OccupancyGrid, '/map', self.occupancyGrid, qos_map)

        self.path_pub = self.create_publisher(Path, '/global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # variables
        self.dynamic_grid = None
        self.goal_pose = Pose()
        self.ttbot_pose = Pose()
        self.scan_data = None
        self.prev_idx = 0

    def timer_cb(self):
        self.get_logger().info('Task2 node is alive.', throttle_duration_sec=1)
        # Feel free to delete this line, and write your algorithm in this callback function

    # Define function(s) that complete the (automatic) mapping task
    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        # store it as Pose object
        self.get_logger().info('received goal pose')
        self.goal_pose = data.pose
        # self.get_logger().info(
            # 'goal_pose: {:.4f}, {:.4f}'.format(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))
        # print('goal_pose: {:.4f}, {:.4f}'.format(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))

    def __ttbot_pose_cbk(self, data):
        """! Callback to catch the position of the vehicle.
        @param  data    PoseWithCovarianceStamped object from amcl.
        @return None.
        """
        self.get_logger().info('received ttbot pose')
        self.ttbot_pose = data.pose.pose
        # self.yaw = self.quaternion_to_yaw(self.ttbot_pose.orientation)
        self.get_logger().info(f'current pose: {self.ttbot_pose}')
        # self.get_logger().info(
            # 'ttbot_pose: {:.4f}, {:.4f}'.format(self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y))
        # print('ttbot_pose: {:.4f}, {:.4f}'.format(self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y))
    
    def enableOccupanctGrid(self):
        self.subscriber4 = self.create_subscription(OccupancyGrid, '/map', self.occupancyGrid, 10)
    
    def disableOccupancyGrid(self):
        self.destroy_subscription(self.subscriber4)
        
    def occupancyGrid(self, msg):
        self.get_logger().info('received map info')
        # if msg is not None:
        self.map_info = msg.info
        self.get_logger().info(f'height, width = {self.map_info.height}, {self.map_info.width}')
        
        # while self.map_info is None:
        #     if msg is None:
        #         continue
        #     self.map_info = msg.info
        #     self.get_logger().info(f'height, width = {self.map_info.height}, {self.map_info.width}')
            
        #     self.static_map = Map('/home/albert/sim_ws/src/turtlebot3_gazebo/maps/map')
        #     self.static_map.inflate_map(pad = 2) 
            

            
    def scan(self, msg):
        self.get_logger().info('received scan data')
        if self.static_map is None:
            return
        
        self.dynamic_grid = np.ones_like(self.static_map.get_grid())
        
        if self.ttbot_pose is None:
            return
        # pad = 2 -> 5 * 5 kernel (inflate obstacle)
        pad = 4
        h, w = self.dynamic_grid.shape
        yaw = self.quaternion_to_yaw(self.ttbot_pose.orientation)
        
        for i in range(len(msg.ranges)):
            # if the detected range is less than range_max -> mark as an obstacle
            if msg.ranges[i] < msg.range_max:
                angle = (msg.angle_min + i * msg.angle_increment)

                # (dx, dy) in robot frame
                # dx = -msg.ranges[i] * np.sin(angle)
                # dy = msg.ranges[i] * np.cos(angle)

                # (x, y) in inertial frame (Rviz)
                x = self.ttbot_pose.position.x + msg.ranges[i] * np.cos(angle + yaw) - 0.064 * np.cos(yaw)
                y = self.ttbot_pose.position.y + msg.ranges[i] * np.sin(angle + yaw) - 0.064 * np.sin(yaw)

                x_grid, y_grid = self.convertFrames(x, y, args='RvizToNode')
                self.dynamic_grid[y_grid][x_grid] = 0

        # inflate obstacles
        kernel = np.ones((2*pad + 1, 2*pad + 1), dtype=bool)
        obs = (self.dynamic_grid == 0)
        inflated = binary_dilation(obs, structure=kernel)
        self.dynamic_grid = np.where(inflated, 0, 1).astype(np.uint8)
        # self.dynamic_grid = inflated.astype(np.uint8)
        # self.static_map.show_map(grid=self.dynamic_grid, title='dynamic')

    
    def a_star_path_planner(self, start_pose, goal_pose, grid):     # Node frame
        """! A Start path planner.
        @param  start_pose    PoseStamped object containing the start of the path to be created.
        @param  end_pose      PoseStamped object containing the end of the path to be created.
        @return path          Path object containing the sequence of waypoints of the created path.
        """
        start_pose_x, start_pose_y = self.convertFrames(start_pose.position.x, start_pose.position.y, 'RvizToNode')
        goal_pose_x, goal_pose_y = self.convertFrames(goal_pose.position.x, goal_pose.position.y, 'RvizToNode')
        aStar = Astar(grid, (start_pose_x, start_pose_y), (goal_pose_x, goal_pose_y), self.convertFrames)
        path_Rviz, path_Node = aStar.solve()
        
        return path_Rviz, path_Node
    
    def get_path_idx(self, path_Rviz, vehicle_pose):
        idx = -1
        idxRadius = 0.3
        
        for i in range(self.prev_idx, len(path_Rviz.poses)):
            if (np.sqrt((vehicle_pose.position.x - path_Rviz.poses[i].pose.position.x) ** 2+(vehicle_pose.position.y - path_Rviz.poses[i].pose.position.y) ** 2)) >= idxRadius:
                idx = i
                self.prev_idx = i
                break
        
        if idx == -1:
            idx = len(path_Rviz.poses) - 1
            
        return idx
        
    def path_follower(self, vehicle_pose, current_goal_pose):
        # pid variables
        kp_speed = 0.8
        kp_heading = 1.0
        
        error_speed = np.sqrt(( vehicle_pose.position.x - current_goal_pose.position.x) ** 2 + ( vehicle_pose.position.y - current_goal_pose.position.y) ** 2)
        speed = kp_speed * error_speed
        
        yaw = self.quaternion_to_yaw(vehicle_pose.orientation)
        
        heading_difference_vehicl_goal = np.arctan2(current_goal_pose.position.y - vehicle_pose.position.y , current_goal_pose.position.x - vehicle_pose.position.x )
        error_heading = heading_difference_vehicl_goal - yaw
        
        if error_heading > np.pi:
            error_heading = -2 * np.pi + error_heading
        elif error_heading < -np.pi:
            error_heading = 2 * np.pi + error_heading    
        
        heading = kp_heading * error_heading
        
        return speed, heading
    
        
    def move_ttbot(self, speed, heading, prev_heading):
        cmd_vel = Twist()
        
        if abs(prev_heading) >= (np.pi / 18) or abs(heading) >= (np.pi / 18):
            speed = 0.0
            prev_heading = heading
            heading = heading
        
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)
            
    def convertFrames(self, x, y, args = None):
        # Rviz
        xmin, xmax, ymin, ymax = self.static_map.limits
        res = self.static_map.map_df.resolution[0]
        # grid
        width = self.static_map.im.size[0]
        height = self.static_map.im.size[1]
        
        if args =='RvizToNode':
            x = 0 + ((x - xmin) / (width * res)) * width
            x = max(min(round(x), width - 1), 0)
            y = 0 + (1 - (y - ymin) / (height * res)) * height
            y = max(min(round(y), height - 1), 0)
            return x, y
        
        elif args == 'NodeToRviz':
            x = xmin + (x / width) * width * res
            x = max(min(round(x ,2), xmax), xmin)
            y = ymin + (1 - (y / height)) * height * res
            y = max(min(round(y, 2), ymax), ymin)
            return x, y
        
        else:
            self.get_logger().info('Didnt specify source frame and target frame')
            return None, None
            
    
    
    def quaternion_to_yaw(self, q): 
        q_length = np.sqrt( q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2) 
        qx_norm = q.x / q_length
        qy_norm = q.y / q_length
        qz_norm = q.z / q_length
        qw_norm = q.w / q_length
        numerator = 2 * (qw_norm * qz_norm + qx_norm * qy_norm)
        denominator = 1 - 2 * (qy_norm ** 2 + qz_norm ** 2)
        yaw = np.arctan2( numerator ,denominator)
        return yaw

    def run(self):
        have_plan = False
        prev_heading = 0
        # prev_time = 0
        
        # self.subscriber1 = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        # self.subscriber2 = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)
        # self.subscriber3 = self.create_subscription(LaserScan, '/scan', self.scan, 10)
        # self.subscriber4 = self.create_subscription(OccupancyGrid, '/map', self.occupancyGrid, 10)
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.map_info is None:
                self.get_logger().info('map info is None')
                continue
            else:
                # self.static_map = Map('/home/albert/sim_ws/src/turtlebot3_gazebo/maps/map', self.map_info)
                self.static_map = Map(map_name='map', map_info=self.map_info)
                # self.static_map.show_map()
                self.static_map.inflate_map(pad = 7)
                # self.static_map.show_map()
                self.subscriber1 = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
                self.subscriber2 = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)
                self.subscriber3 = self.create_subscription(LaserScan, '/scan', self.scan, 10)
                self.disableOccupancyGrid()
                break
        
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec= 0.1)
            

            # self.get_logger().info(f'static map size{self.static_map.im.size[0], self.static_map.im.size[1]}')
            # self.get_logger().info(f'limits: {self.static_map.limits}')
            
            if (self.goal_pose.position.x == 0.0 and self.goal_pose.position.y == 0.0 and self.goal_pose.position.z == 0.0):
                continue
            
            if not have_plan or len(self.path_Rviz.poses) <= 2:
                # merge dynamic and static grids
                grid = self.static_map.get_grid().copy()
                grid[self.dynamic_grid == 0] = 0
                # grid = 1 - grid
                # self.static_map.show_map(grid=grid, title ='merged')
                
                
                # self.path should be in the Rviz frame
                self.path_Rviz, self.path_Node = self.a_star_path_planner(self.ttbot_pose, self.goal_pose, grid)
                
                # show path
                # self.get_logger().info(f'Path : {self.path_Node}')
                self.static_map.show_map(path = self.path_Node, grid=grid, title = 'merged')
                
                # convert self.path into Rviz frame and also store the path (node frame) in self.path_node_frame
                # for i in self.path.poses:
                #     (i.pose.position.x, i.pose.position.y) = self.convertFrames(i.pose.position.x, i.pose.position.y, 'NodeToRviz')
                self.path_pub.publish(self.path_Rviz)
                
                have_plan = True
            
            else:
                idx = self.get_path_idx(self.path_Rviz, self.ttbot_pose)
                x, y = self.path_Node[idx]
                
                if self.dynamic_grid[y][x] == 0:
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    have_plan = False
                    self.prev_idx = 0
                    continue
                
                current_goal = self.path_Rviz.poses[idx].pose
                speed, heading = self.path_follower(self.ttbot_pose, current_goal)
                self.move_ttbot(speed, heading, prev_heading)
            
            
class Map():
    def __init__(self, map_name, map_info):
        package_dir = get_package_share_directory('turtlebot3_gazebo')
        map_path = os.path.join(package_dir, 'maps', map_name)
        self.map_info = map_info
        self.im, self.map_df, self.limits = self.__open_map(map_path)
        self.grid = self.__get_obstacle_map(self.im, self.map_df)
        


    def __open_map(self, map_name):
        f = open(map_name + '.yaml', 'r')
        map_df = pd.json_normalize(yaml.safe_load(f))
        map_dir = os.path.dirname(map_name+'.pgm')
        image_path = os.path.join(map_dir, map_df.image[0])
        im = Image.open(image_path)
        # size = 200, 200
        # im.thumbnail(size)
        im = ImageOps.grayscale(im)
        
        # im.size[0], im.size[1] = self.map_info.width, self.map_info.height
        xmin = map_df.origin[0][0]
        xmax = map_df.origin[0][0] + im.size[0] * map_df.resolution[0]
        ymin = map_df.origin[0][1]
        ymax = map_df.origin[0][1] + im.size[1] * map_df.resolution[0]

        return im, map_df, [xmin, xmax, ymin, ymax]
    
    def __get_obstacle_map(self, im, map_df):
        img_array = np.reshape(list(im.getdata()),(im.size[1],im.size[0]))
        up_thresh = map_df.occupied_thresh[0] * 255

        # cells above the occupied threshold become 1, else 0
        grid = np.zeros_like(img_array)
        grid = (img_array > up_thresh).astype(np.uint8)

        return grid
        
    def inflate_map(self, pad = 3):
        # pad = 2 -> 5 * 5 kernel
        
        obs = (self.grid == 0)

        # build a square structuring element
        kernel = np.ones((2*pad+1, 2*pad+1), dtype=bool)

        # dilate the obstacle mask
        dilated_obs = binary_dilation(obs, structure=kernel)

        # back to 0/1: obstacles → 0, free → 1
        self.grid = np.where(dilated_obs, 0, 1)

        
    def get_grid(self):
        return self.grid
    
    def draw_path(self, path):
        grid = self.get_grid()
        path_array = grid.astype(float)

        for idx in path:
            # allow either ("i,j") strings or (i,j) tuples
            i, j = idx[0], idx[1]
            path_array[j][i] = 0.5

        return path_array
    
    def show_map(self, path=None, grid=None, figsize=(6,6), cmap='gray', origin='upper', title = 'inflated static map'):
        
        img = self.get_grid().copy() if grid is None else grid

        if path is not None:
            # draw the path into img at intensity 0.5
            overlay = self.draw_path(path)
            # wherever overlay==0.5, override img
            img = np.where(overlay==0.5, 0.5, img)
            plt_title = f"{title} + Path" 
        else:
            plt_title = title
        
        plt.figure(figsize=figsize)
        plt.imshow(img, cmap=cmap, origin=origin, interpolation='nearest')
        plt.title(title)
        plt.axis('off')
        plt.show()

class Astar():
    def __init__(self, grid, start, goal, convertFrames):
        self.grid = grid
        self.start = start  #(x, y) Node frame
        self.goal = goal    #(x, y) Node frame
        self.convertFrames = convertFrames
        
        self.height, self.width = grid.shape
        
        self.dist = np.full((self.height, self.width), np.inf)
        self.h = np.zeros((self.height, self.width))
        self.via = np.zeros((self.height, self.width), dtype = object)
        
        for y in range(self.height):
            for x in range(self.width):
                if grid[y][x] != 1:
                    self.h[y][x] = np.sqrt((goal[0] - x) ** 2 + (goal[1] - y) ** 2)
    
    def __get_f_score(self, node):
        return self.dist[node[1]][node[0]] + self.h[node[1]][node[0]]
    
    def __get_g_score(self, start, end):
        return abs(start[0] - end[0]) + abs(start[1] - end[1])
    
    def adjacentNeighbors(self, node, size = 1):
        x = node[0]
        y = node[1]
        directions = [(dx, dy) for dx in range(-size, size + 1) for dy in range(-size, size + 1) if not (dx == 0 and dy ==0)]
        neighbors = list()
        
        for dx, dy in directions:
            if 0 <= (x + dx) < self.width and 0 <= (y + dy) < self.height:
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
                if self.grid[child[1]][child[0]] == 0:
                    continue

                tentative_g = self.dist[u[1]][u[0]] + self.__get_g_score(u, child)
                
                if tentative_g < self.dist[child[1]][child[0]]:
                    self.dist[child[1]][child[0]] = tentative_g
                    self.via[child[1]][child[0]] = u
                    heapq.heappush(open_heap, (self.__get_f_score(child), child))

        return self.reconstruct_path()

    def reconstruct_path(self):
        path_Rviz = Path()
        path_Node = []
        current = self.goal
        
        path_Rviz.header.stamp = Time()
        path_Rviz.header.frame_id = 'map'
        
        while current != 0:
            currentNode = PoseStamped()
            x, y = self.convertFrames(current[0], current[1], 'NodeToRviz')
            currentNode.pose.position.x = x
            currentNode.pose.position.y = y
            currentNode.pose.position.z = 0.0

            currentNode.pose.orientation.x=0.0 
            currentNode.pose.orientation.y=0.0
            currentNode.pose.orientation.z=0.0
            currentNode.pose.orientation.w=1.0 
            currentNode.header.stamp=Time()      # change accordingly
            currentNode.header.frame_id='map'
            path_Rviz.poses.append(currentNode)
            path_Node.append(current)
            current = self.via[current[1]][current[0]]
            
            
        path_Rviz.poses.reverse() 
        path_Node.reverse()
        
        return path_Rviz, path_Node
        
def main(args=None):
    rclpy.init(args=args)

    task2 = Task2()
    try:
        task2.run()
    except KeyboardInterrupt:
        pass
    finally:
        task2.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
