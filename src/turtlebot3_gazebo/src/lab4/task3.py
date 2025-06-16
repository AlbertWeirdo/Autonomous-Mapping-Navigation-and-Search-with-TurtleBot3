#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Pose, Point
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Image as ROSImage
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker
import heapq
import numpy as np
import os
import yaml
from PIL import Image, ImageOps
import pandas as pd
from copy import copy
import matplotlib.pyplot as plt
import time
from collections import deque
from ament_index_python.packages import get_package_share_directory

from scipy.ndimage import binary_dilation


# Import other python packages that you think necessary


class Task3(Node):
    """
    Environment localization and navigation task.
    You can also inherit from Task 2 node if most of the code is duplicated
    """
    def __init__(self):
        super().__init__('task3_node')
        self.timer = self.create_timer(0.1, self.timer_cb)
        # Fill in the initialization member variables that you need
        self.static_map = Map(map_name='map')
        # node frame centers of each rooms
        self.goals_list = [(96, 53), (25, 43), (20, 160), (247, 50), (171, 49), (272, 160)]
        
        
        self.subscriber1 = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)
        self.subscriber2 = self.create_subscription(LaserScan, '/scan', self.scan, 10)
        self.subscriber3 = self.create_subscription(ROSImage, '/camera/image_raw', self.image_sub, 10)
        
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.maker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
         
        # self.publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        
        # global variables
        self.ball_locations = {'red':None, 'blue':None, 'green':None}
        self.dynamic_map = None
        self.laser_scan = None
        self.goal_ball_following = None
        self.arc_detected = False
        self.found_ball = False
        # self.goal = None
        self.ttbot_pose = Pose()
        self.bridge = CvBridge()
        self.image = ROSImage()
        self.color = None
        self.prev_idx = 0
        self.timeout_count = 0
        self.robot_in_obstacles = False
        # self.ball_center_img = None
        


    def timer_cb(self):
        # self.get_logger().info('Task3 node is alive.', throttle_duration_sec=1)
        pass
        # Feel free to delete this line, and write your algorithm in this callback function

    # Define function(s) that complete the (automatic) mapping task
    
    ### subscriptions ###
    def __ttbot_pose_cbk(self, data):
        
        self.ttbot_pose = data.pose.pose
        # self.get_logger().info(f'received ttbot pose, current pose: {data.pose.pose}')
    
    def scan(self, msg):
        
        if self.static_map is None or self.ttbot_pose is None:
            return
        
        # self.get_logger().info('received scan data')
        self.laser_scan = msg
        self.laser_scan_ranges = np.array(msg.ranges, dtype= float)
        self.laser_scan_ranges = np.clip(self.laser_scan_ranges, self.laser_scan.range_min, self.laser_scan.range_max)
                
        self.dynamic_map = np.ones_like(self.static_map.get_grid())

        # pad = 2 -> 5 * 5 kernel (inflate obstacle)
        pad = 3
        h, w = self.dynamic_map.shape
        yaw = self.quaternion_to_yaw(self.ttbot_pose.orientation)
        
        for i in range(len(msg.ranges)):
            # if the detected range is less than range_max -> mark as an obstacle
            if msg.ranges[i] < msg.range_max:
                angle = (msg.angle_min + i * msg.angle_increment)

                # (x, y) in inertial frame (Rviz)
                x = self.ttbot_pose.position.x + msg.ranges[i] * np.cos(angle + yaw) - 0.064 * np.cos(yaw)
                y = self.ttbot_pose.position.y + msg.ranges[i] * np.sin(angle + yaw) - 0.064 * np.sin(yaw)

                x_grid, y_grid = self.convertFrames(x, y, args='RvizToNode')
                self.dynamic_map[y_grid][x_grid] = 0
        # look for arc        
        # dynamic_map_uninfalted = self.dynamic_map.copy()
        # self.arc_searching(msg, dynamic_map_uninfalted)
        
        
        # inflate obstacles
        kernel = np.ones((2*pad + 1, 2*pad + 1), dtype=bool)
        obs = (self.dynamic_map == 0)
        inflated = binary_dilation(obs, structure=kernel)
        self.dynamic_map = np.where(inflated, 0, 1).astype(np.uint8)
            
        # self.static_map.show_map(grid=self.dynamic_map, title='dynamic') 
    
    def image_sub(self, msg):
        self.image_cv2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_hsv = cv2.cvtColor(self.image_cv2, cv2.COLOR_BGR2HSV)
        self.image_ros_width = msg.width
        
        
        # blue ball
        blue_lower_bound = np.array([105, 180, 80])
        blue_upper_bound = np.array([120, 255, 255])
        blue_mask = cv2.inRange(self.image_hsv, blue_lower_bound, blue_upper_bound)
        # masked = cv2.bitwise_and(self.image_cv2, self.image_cv2, mask = blue_mask)
        
        # red
        red_lower_bound1 = np.array([0, 180, 80])
        red_upper_bound1 = np.array([5, 255, 255])
        red_mask1 = cv2.inRange(self.image_hsv, red_lower_bound1, red_upper_bound1)
        red_lower_bound2 = np.array([175, 180, 80])
        red_upper_bound2 = np.array([180, 255, 255])
        red_mask2 = cv2.inRange(self.image_hsv, red_lower_bound2, red_upper_bound2)
        # masked = cv2.bitwise_and(self.image_cv2, self.image_cv2, mask = red_mask1 + red_mask2)
        
        # green
        green_lower_bound = np.array([60, 180, 80])
        green_upper_bound = np.array([65, 255, 255])
        green_mask = cv2.inRange(self.image_hsv, green_lower_bound, green_upper_bound)
        # masked = cv2.bitwise_and(self.image_cv2, self.image_cv2, mask = green_mask)
        mask = red_mask1 + red_mask2 + blue_mask + green_mask
        masked = cv2.bitwise_and(self.image_cv2, self.image_cv2, mask = mask)
        
        # cv2.imshow('after detection', masked)
        # cv2.waitKey(25)

        # find contour
        self.find_contour(mask=mask, masked=masked, red_mask=red_mask1+red_mask2, blue_mask=blue_mask, green_mask=green_mask)
            
        
        
        
    
    ### ball searching ###
    # ball's diameter is 0.3m
    def find_contour(self, mask, masked, red_mask, blue_mask, green_mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            # 50000
            # if cv2.contourArea(contour) < 100000:
            #     self.goal_ball_following = self.ball_following(x, y)
            #     # self.mode = 'ball'
            #     self.found_ball = True
            #     return

            # self.stopRobot()
            # self.mode = 'calculate ball center'
            cv2.circle(masked, (int(x),int(y)), int(radius), (255, 255, 255), 2)


            # classify the color
            color_candidates= [c for c, location in self.ball_locations.items() if location is None]
            circle_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.circle(circle_mask, (int(x),int(y)), int(radius), 255, -1)
            
            # cv2.imshow('mask with circle', masked)
            # cv2.imshow('mask with circle', circle_mask)
            # cv2.waitKey(25)
            
            b_channel, g_channel, r_channel = cv2.split(masked)
            bs = cv2.bitwise_and(circle_mask, b_channel)
            gs = cv2.bitwise_and(circle_mask, g_channel)
            rs = cv2.bitwise_and(circle_mask, r_channel)
            
            b_count, g_count, r_count = bs.sum(), gs.sum(), rs.sum()
            
            if r_count > b_count and r_count > g_count:
                color = 'red'    
                count = r_count
            elif g_count > r_count and g_count > b_count:
                color = 'green'
                count = g_count
            else:
                color = 'blue'
                count = b_count
            
            if count < 3000:
                color = None
                # self.get_logger().info('no color detected')
                return
            

            # self.get_logger().info(f'color = {color}, counts = {r_count, g_count, b_count}')
            
            if self.ball_locations[color] is not None:
                continue
            
            if self.ball_locations[color] is None:
                
                if 400 < cv2.contourArea(contour) < 150000:
                    self.goal_ball_following = self.ball_following(x, y)
                    self.found_ball = True
                    return

                elif cv2.contourArea(contour) > 150000:
                    self.timeout_count += 1
                    self.stopRobot()
                    self.mode = 'calculate ball center'
                    self.color = color
                    self.ball_center_img = x
                    if self.timeout_count <= 1:
                        self.current = time.time()
                    # (x_ball, y_ball) = self.determine_ball_center()
                    # self.ball_locations[color] = (x_ball, y_ball)
                    
                    
    def ball_following(self, x, y):
        
        if self.laser_scan is None:
            return None
        image_center = 0.5 * self.image_ros_width
        rad_per_pixel = self.image_ros_width/ 1.02974
        
        # top left corner is the origin of the image
        # center means 0 deg in the laserscan ranges
        # if the ball is too small in the image, we use the same direction of the ball center from the image as the goal to get closer to the ball
        ang_rad = ((image_center - x) / rad_per_pixel)
        if ang_rad < 0:
            ang_rad += 2 * np.pi
        
        idx = round((ang_rad - self.laser_scan.angle_min) / self.laser_scan.angle_increment)
        idx = min(max(idx, 0), len(self.laser_scan_ranges)-1)
        
        yaw = self.quaternion_to_yaw(self.ttbot_pose.orientation)
        dist = self.laser_scan_ranges[idx]
        ang = (self.laser_scan.angle_min + idx * self.laser_scan.angle_increment)
        
        # … your existing code up to dist and ang …
        dist = self.laser_scan_ranges[idx]
        ang   = self.laser_scan.angle_min + idx*self.laser_scan.angle_increment
        yaw   = self.quaternion_to_yaw(self.ttbot_pose.orientation)

        # Try stepping back in 0.1 m increments until we land in free space
        step = 0.1
        while dist > 0:
            # compute candidate in world coords
            gx = self.ttbot_pose.position.x + dist * np.cos(ang + yaw) - 0.064 * np.cos(yaw)
            gy = self.ttbot_pose.position.y + dist * np.sin(ang + yaw) - 0.064 * np.sin(yaw)
            # grid coords
            ix, iy = self.convertFrames(gx, gy, args='RvizToNode')
            # check dynamic_map: 1=free, 0=occupied
            if self.dynamic_map[iy, ix] == 1:
                # build & publish marker as before
                marker = Marker()
                # fill in marker fields
                marker.pose.position.x = gx
                marker.pose.position.y = gy
                self.maker_pub.publish(marker)
                return (ix, iy)
            dist -= step

        # if we’re all the way back to zero without finding free, give up
        return None



    
    # def ball_following(self, x, y):
    #     if self.laser_scan is None:
    #         return None
    #     image_center = 0.5 * self.image_ros_width
    #     rad_per_pixel = self.image_ros_width/ 1.02974
        
    #     # top left corner is the origin of the image
    #     # center means 0 deg in the laserscan ranges
    #     # if the ball is too small in the image, we use the same direction of the ball center from the image as the goal to get closer to the ball
    #     ang_rad = ((image_center - x) / rad_per_pixel)
    #     if ang_rad < 0:
    #         ang_rad += 2 * np.pi
        
    #     idx = round((ang_rad - self.laser_scan.angle_min) / self.laser_scan.angle_increment)
    #     idx = min(max(idx, 0), len(self.laser_scan_ranges)-1)
        
    #     yaw = self.quaternion_to_yaw(self.ttbot_pose.orientation)
    #     dist = self.laser_scan_ranges[idx]
    #     ang = (self.laser_scan.angle_min + idx * self.laser_scan.angle_increment)
    #     goal_x_Rviz = self.ttbot_pose.position.x + dist * np.cos(ang + yaw) - 0.064
    #     goal_y_Rviz = self.ttbot_pose.position.y + dist * np.sin(ang + yaw)
        
        
    #     marker = Marker()
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.header.frame_id = 'map'
    #     marker.id = 0
    #     marker.type = Marker.SPHERE
    #     marker.ns = ''
    #     marker.action = Marker.ADD
    #     marker.pose.position.x = goal_x_Rviz
    #     marker.pose.position.y = goal_y_Rviz
    #     marker.pose.position.z = 0.0
    #     marker.pose.orientation.x = 0.0
    #     marker.pose.orientation.y = 0.0
    #     marker.pose.orientation.z = 0.0
    #     marker.pose.orientation.w = 1.0
    #     marker.scale.x = 0.1  
    #     marker.scale.y = 0.1
    #     marker.scale.z = 0.1
    #     marker.color.r = 1.0
    #     marker.color.g = 1.0
    #     marker.color.b = 1.0
    #     marker.color.a = 1.0  
    #     self.maker_pub.publish(marker)
        
        

        
    #     # return (goal_x_Rviz, goal_y_Rviz)
    #     x_grid, y_grid = self.convertFrames(goal_x_Rviz, goal_y_Rviz, args='RvizToNode')
    #     return (x_grid, y_grid)
    
    def calculate_ball_center(self, x):
        # ball's radius = 0.15 m
        r = 0.15
        
        if self.laser_scan is None:
            return
        # ranges = self.laser_scan.ranges.copy()
        image_center = 0.5 * self.image_ros_width
        rad_per_pixel = self.image_ros_width/ 1.02974
        
        ang_rad = ((image_center - x) / rad_per_pixel)
        if ang_rad < 0:
            ang_rad += 2 * np.pi
        
        idx = round((ang_rad - self.laser_scan.angle_min) / self.laser_scan.angle_increment)
        idx = min(max(idx, 0), len(self.laser_scan_ranges)-1)
        
        arc_start_idx, arc_end_idx = self.arc_searching(idx)    # degree
        arc_start_x, arc_start_y = self.scan_dist_to_rviz(arc_start_idx)
        arc_end_x, arc_end_y = self.scan_dist_to_rviz(arc_end_idx)
        
        # chord distance
        dist_arc = np.sqrt((arc_start_x - arc_end_x) ** 2 + (arc_start_y - arc_end_y) ** 2)
        theta = np.arcsin(dist_arc / (2 * r))   # radian
        
        chord_mid_x, chord_mid_y = 0.5 * (arc_start_x + arc_end_x), 0.5 * (arc_start_y + arc_end_y)
        
        # calculate the othornormal vector
        dx = arc_end_x - arc_start_x
        dy = arc_end_y - arc_start_y
        
        norm = np.sqrt(dx ** 2 + dy ** 2)
        nx, ny = -dy / norm, dx / norm
        
        dist_center_to_mid = 0.15 * np.cos(theta / 2)
        center1_x, center1_y = chord_mid_x + dist_center_to_mid * nx, chord_mid_y + dist_center_to_mid * ny
        center2_x, center2_y = chord_mid_x - dist_center_to_mid * nx, chord_mid_y - dist_center_to_mid * ny
        
        dist_center_to_robot1 = np.sqrt((center1_x - self.ttbot_pose.position.x)**2 + (center1_y - self.ttbot_pose.position.y)**2)
        dist_center_to_robot2 = np.sqrt((center2_x - self.ttbot_pose.position.x)**2 + (center2_y - self.ttbot_pose.position.y)**2)
        
        if dist_center_to_robot1 > dist_center_to_robot2:
            center_x, center_y = center1_x, center1_y
        else:
            center_x, center_y = center2_x, center2_y
        
        self.ball_locations[self.color] = (center_x, center_y)
        self.get_logger().info(f'Color = {self.color}, location = {center_x, center_y}')
        self.publish_answer(self.color, center_x, center_y)
        
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.ns = ''
        marker.action = Marker.ADD
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  
        self.maker_pub.publish(marker)
        
        self.color = None
        self.mode = 'after calculating ball\'s center'
        self.have_plan = False
        self.found_ball = False
        self.goal_ball_following = None
        self.timeout_count = 0
        # pass
    
    def publish_answer(self, color, center_x, center_y):
        topic = '/'+color+'_pos'
        self.publisher_ans = self.create_publisher(Point, topic, 10)
        ans = Point()
        ans.x = float(center_x)
        ans.y = float(center_y)
        ans.z = float(0.1)
        self.publisher_ans.publish(ans)
        self.destroy_publisher(self.publisher_ans)
        
        
    
    def arc_searching(self, idx, threshold = 0.1):
        self.get_logger().info('arc searching')
        
        # use the unclipped one
        ranges = self.laser_scan.ranges
        
        current_dist = ranges[idx]
        
        start, end = idx, idx
        
        # to the left
        for i in range(1, 30):
            idx_temp = (idx + i) % 360
            if abs(ranges[idx_temp] - current_dist) < threshold:
                start = idx_temp
            else:
                break
        
        # to the right
        for i in range(1, 30):
            idx_temp = (idx - i + 360) % 360
            if abs(ranges[idx_temp] - current_dist) < threshold:
                end = idx_temp
            else:
                break
            
        return start, end
        # pass
    
    def scan_dist_to_rviz(self, idx):
        angle = (self.laser_scan.angle_min + idx * self.laser_scan.angle_increment)

        yaw = self.quaternion_to_yaw(self.ttbot_pose.orientation)
        # (x, y) in inertial frame (Rviz)
        x = self.ttbot_pose.position.x + self.laser_scan.ranges[idx] * np.cos(angle + yaw) - 0.064
        y = self.ttbot_pose.position.y + self.laser_scan.ranges[idx] * np.sin(angle + yaw)
        
        return x, y
        
        
    ### path planning ###
    def a_star_path_planner(self, start_pose, goal_pose, grid):
        
        start_pose_x, start_pose_y = self.convertFrames(start_pose.position.x, start_pose.position.y, 'RvizToNode')
        # goal_pose_x, goal_pose_y = self.convertFrames(goal_pose[0], goal_pose[1], 'RvizToNode')
        aStar = Astar(grid, (start_pose_x, start_pose_y), goal_pose, self.convertFrames)
        path_Rviz, path_Node = aStar.solve()
        
        return path_Rviz, path_Node
    
    def get_path_idx(self, path_Rviz, vehicle_pose):
        idx = -1
        idxRadius = 0.30
        reach_goal = False
        
        for i in range(self.prev_idx, len(path_Rviz.poses)):
            if (np.sqrt((vehicle_pose.position.x - path_Rviz.poses[i].pose.position.x) ** 2 + (vehicle_pose.position.y - path_Rviz.poses[i].pose.position.y) ** 2)) >= idxRadius:
                idx = i
                self.prev_idx = i
                break
        
        if idx >= len(path_Rviz.poses) - 1 or idx == -1:
            idx = len(path_Rviz.poses) - 1
            reach_goal = True
            
        return idx, reach_goal
    
    ### robot motions ###
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
            heading = heading / 1.5
        
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)


    ### commonly used tools ###    
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
    
    def stopRobot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def wallFollower(self, duration = 3):
        start_time = time.time()
        # self.enableLaserScan()
        kp = 0.5

        
        while rclpy.ok() and time.time() < start_time + duration:
            rclpy.spin_once(self, timeout_sec= 0.05)
            speed = 0.15
            heading = 0.0
            if self.laser_scan is None:
                continue
            
            range = np.array(self.laser_scan.ranges)
            min_index = np.nanargmin(range)
            min_angle = (self.laser_scan.angle_min + min_index * self.laser_scan.angle_increment) * 180 / np.pi
            # range = np.clip(range, 0, 3.0)

            min_range = range[min_index]
            
            front_sector = np.concatenate((range[-15:], range[:16]))
            front_sector = np.clip(front_sector, 0, np.inf)
            front = np.mean(front_sector)
            right = np.mean(range[189:351])
            left = np.mean(range[9:171])

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
                    time.sleep(0.15)
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
                
            
            elif right > 2 and left > 2 and front > 2:
                self.get_logger().info(f'no angle adjustment')
                # side = 0
                # side_check = 1
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
        self.destroy_subscription(self.subscriber3)
        
    
    def run(self):
        """
        1. use A* to travel to selected goal points
        2. use the same obstacle avoidance from task2
        3. when the obstacle avoidance is triggered, use camera to determine the color (Maybe wait a few seconds to eliminate the delay from /scan)
        4. calculate the center of the obstacle using the dynamic map
        """
        self.mode = 'waypoint'
        self.have_plan = False
        
        
        # self.found_ball = False
        # self.goal_ball_followings
        
        goal_idx = 0
        prev_heading = 0
        self.current = time.time()
        current_goal_ball_following = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # time out when the code first starts or the obstacle avoidance is triggered
            if time.time() < self.current + 3:
                rclpy.spin_once(self, timeout_sec=0.1)
                # self.get_logger().info('sleeping')
                continue
            if self.static_map is None or self.dynamic_map is None or self.ttbot_pose is None:
                continue
            
            
            if self.robot_in_obstacles:
                self.get_logger().info('Wall Following')
                self.wallFollower(duration = 3)
                self.robot_in_obstacles = False
                continue
            
            if self.mode == 'calculate ball center':
                self.get_logger().info('calculating ball\'s center')
                self.stopRobot()
                # time.sleep(2)
                rclpy.spin_once(self, timeout_sec=0.1)
                
                ball_center_img = self.ball_center_img
                self.calculate_ball_center(ball_center_img)
                # self.ball_locations[self.color]= (x, y)
                # self.color = None
                # self.found_ball = False
                continue
            

            # continue
            if self.found_ball and self.mode != 'ball':
                if self.goal_ball_following is None:
                    continue
                
                
                current_goal_ball_following = self.goal_ball_following
                self.get_logger().info('ball following')
                grid = self.static_map.get_grid().copy()
                grid[self.dynamic_map == 0] = 0
                # self.static_map.show_map(grid = grid)
                
                if not self.robot_in_obstacles:
                    x_robot_node, y_robot_node = self.convertFrames(self.ttbot_pose.position.x, self.ttbot_pose.position.y, 'RvizToNode')
                    
                    if grid[y_robot_node][x_robot_node] == 0:
                        self.robot_in_obstacles = True
                        continue
                
                self.path_Rviz, self.path_Node = self.a_star_path_planner(self.ttbot_pose, current_goal_ball_following, grid) 
                # self.static_map.show_map(grid=grid, title='merged', path=self.path_Node)
                self.path_pub.publish(self.path_Rviz)               
                self.mode = 'ball'
                self.have_plan = True
                self.found_ball = False
                self.prev_idx = 0
                continue
            
            if self.have_plan:
                # self.get_logger().info('path following')
                idx, reach_goal = self.get_path_idx(self.path_Rviz, self.ttbot_pose)
                
                if reach_goal:
                    
                    if self.mode == 'waypoint':
                        self.get_logger().info('waypoint goal is reached')
                        self.stopRobot()
                        self.goals_list.pop(goal_idx)
                        self.have_plan = False
                        dist = np.inf
                        for i in range(len(self.goals_list)):
                            goal_temp = self.convertFrames(self.goals_list[i][0], self.goals_list[i][1], 'NodeToRviz')
                            dist_temp = np.sqrt((self.ttbot_pose.position.x - goal_temp[0]) ** 2 + (self.ttbot_pose.position.y - goal_temp[1]) ** 2)
                            if dist_temp < dist:
                                goal_idx = i
                                dist = dist_temp
                        self.prev_idx = 0
                        self.current = time.time()
                        continue

                    else:
                        self.get_logger().info('goal near ball is reached')
                        self.stopRobot()
                        self.mode = 'waypoint'
                        # self.goal_ball_following = None
                        current_goal_ball_following = None
                        self.have_plan = False
                        # self.found_ball = False
                        dist = np.inf
                        for i in range(len(self.goals_list)):
                            goal_temp = self.convertFrames(self.goals_list[i][0], self.goals_list[i][1], 'NodeToRviz')
                            dist_temp = np.sqrt((self.ttbot_pose.position.x - goal_temp[0]) ** 2 + (self.ttbot_pose.position.y - goal_temp[1]) ** 2)
                            if dist_temp < dist:
                                goal_idx = i
                                dist = dist_temp
                        self.prev_idx = 0
                        self.current = time.time()
                        # self.ttbot_pose = None
                        continue
                
                obstacle = False
                for i in range(3):
                    idx_temp = idx + i
                    if idx_temp < len(self.path_Node):
                        x, y = self.path_Node[idx_temp]
                        if self.dynamic_map[y][x] == 0:
                            self.get_logger().info('obstacle avoidance')
                            self.stopRobot()
                            self.have_plan = False
                            self.prev_idx = 0
                            # self.current = time.time()
                            obstacle = True
                            break
                    else:
                        break
                if obstacle:
                    self.stopRobot()
                    self.current = time.time()
                    continue
                
                current_goal = self.path_Rviz.poses[idx].pose
                speed, heading = self.path_follower(self.ttbot_pose, current_goal)
                self.move_ttbot(speed, heading, prev_heading)
            
            elif not self.have_plan:
                grid = self.static_map.get_grid().copy()
                grid[self.dynamic_map == 0] = 0
                
                if not self.robot_in_obstacles:
                    x_robot_node, y_robot_node = self.convertFrames(self.ttbot_pose.position.x, self.ttbot_pose.position.y, 'RvizToNode')
                    
                    if grid[y_robot_node][x_robot_node] == 0:
                        self.robot_in_obstacles = True
                        continue
                
                
                    
                if self.mode == 'waypoint':
                    self.get_logger().info('waypoint path planning')
                    goal = self.goals_list[goal_idx]
                elif self.mode == 'ball':
                    self.get_logger().info('ball path planning')
                    # goal = self.goal_ball_following 
                    goal = current_goal_ball_following
                elif self.mode == 'after calculating ball\'s center':
                    self.get_logger().info('after calculating ball\'s center')
                    dist = np.inf
                    
                    for i in range(len(self.goals_list)):
                        goal_temp = self.convertFrames(self.goals_list[i][0], self.goals_list[i][1], 'NodeToRviz')
                        dist_temp = np.sqrt((self.ttbot_pose.position.x - goal_temp[0]) ** 2 + (self.ttbot_pose.position.y - goal_temp[1]) ** 2)
                        if dist_temp < dist:
                            goal_idx = i
                            dist = dist_temp
                            
                    goal = self.goals_list[goal_idx]
                    self.mode = 'waypoint'
                    # self.prev_idx = 0

                
                

                
                
                self.path_Rviz, self.path_Node = self.a_star_path_planner(self.ttbot_pose, goal, grid)
                # self.static_map.show_map(grid=grid, title='merged', path=self.path_Node)    
                if len(self.path_Node) == 1:
                    self.current = time.time()
                    continue
                self.path_pub.publish(self.path_Rviz)
                self.have_plan = True
                self.prev_idx = 0
                continue
            
            if all(loc is not None for loc in self.ball_locations.values()):
                self.get_logger().info('Found all three balls')
                break

            
            # continue
            
            # if self.arc_detected and not camera_searching:
            #     self.stopRobot()
            #     camera_searching = True
            
            # if camera_searching:
            #     self.ball_searching()
            #     camera_searching = False
            #     self.arc_detected = False
                

            # elif not have_plan and not camera_searching:
            #     # for computer vision
            #     # for path planning
                
            #     goal = self.goals[goal_idx]
            #     # 0 means occupied, 1 means free space
            #     grid = self.static_map.get_grid().copy()
            #     grid[self.dynamic_map == 0] = 0
            #     # self.static_map.show_map(grid=grid)
                
            #     self.path_Rviz, self.path_Node = self.a_star_path_planning(self.ttbot_pose, goal, grid)
            #     self.path_pub.publish(self.path_Rviz)
                
            #     have_plan = True
            
            # elif have_plan and not camera_searching:
            #     idx, reach_goal = self.get_path_idx(self.path_Rviz, self.ttbot_pose)
            #     if reach_goal:
            #         self.stopRobot()
                    
            #         goal_idx += 1
            #         have_plan = False
            #         camera_searching = True
            #         self.prev_idx = 0
                
            #     x, y = self.path_Node[idx]
                
            #     if self.dynamic_map[y][x] == 0:
            #         self.stopRobot()
                    
            #         have_plan = False
            #         self.prev_idx = 0
            #         camera_searching = True
            #         continue
                
            #     current_goal = self.path_Rviz.poses[idx].pose
            #     speed, heading = self.path_follower(self.ttbot_pose, current_goal)
            #     self.move_ttbot(speed, heading, prev_heading)

                
            # if goal_idx >= len(self.goals):
            #     self.get_logger().info('finished searching')
            #     break
            
            
            
            
            
    
class Map():
    def __init__(self, map_name):
        package_dir = get_package_share_directory('turtlebot3_gazebo')
        map_path = os.path.join(package_dir, 'maps', map_name)
        
        self.im, self.map_df, self.limits = self.__open_map(map_path)
        self.grid = self.__get_obstacle_map(self.im, self.map_df)
        self.inflate_map(pad=4)
        # self.show_map()ballDetected
        
    def __open_map(self, map_name):
        f = open(map_name + '.yaml', 'r')
        map_df = pd.json_normalize(yaml.safe_load(f))
        map_dir = os.path.dirname(map_name+'.pgm')
        image_path = os.path.join(map_dir, map_df.image[0])
        im = Image.open(image_path)
        im = ImageOps.grayscale(im)
        
        xmin = map_df.origin[0][0]
        xmax = map_df.origin[0][0] + im.size[0] * map_df.resolution[0]
        ymin = map_df.origin[0][1]
        ymax = map_df.origin[0][1] + im.size[1] * map_df.resolution[0]

        return im, map_df, [xmin, xmax, ymin, ymax]
    
    def __get_obstacle_map(self, im, map_df):
        img_array = np.reshape(list(im.getdata()),(im.size[1],im.size[0]))
        up_thresh = map_df.occupied_thresh[0] * 255

        # cells above the occupied threshold become 1, else 0
        # 1: occupied, 0: free space 
        grid = np.zeros_like(img_array)
        grid = (img_array > up_thresh).astype(np.uint8)

        return grid
    
    def inflate_map(self, pad = 5):
        # pad = 2 -> 5 * 5 kernel
        
        obs = (self.grid == 0)

        # build a square structuring element
        kernel = np.ones((2*pad+1, 2*pad+1), dtype=bool)

        # dilate the obstacle mask
        dilated_obs = binary_dilation(obs, structure=kernel)

        # back to 0/1: obstacles -> 0, free -> 1
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

    task3 = Task3()

    try:
        task3.run()
    except KeyboardInterrupt:
        pass
    finally:
        task3.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
