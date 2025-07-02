import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
from enum import Enum

class State(Enum):
    GO_TO_WAYPOINT = 0     
    GO_TO_FINAL_GOAL = 1    
    GOAL_REACHED = 2     

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
       
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

       
        self.timer = self.create_timer(0.1, self.control_loop)

      
        self.waypoint_x = -0.925741   
        self.waypoint_y = -0.523239
        self.final_target_x = 2.0      
        self.final_target_y = 0.595133
       
        self.position = None
        self.yaw = 0.0
        self.latest_scan = None
        self.state = State.GO_TO_WAYPOINT
        
       
        self.scan_info_received = False
        self.num_scan_points = 0
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0

       
        self.max_linear_speed = 0.22
        self.max_angular_speed = 0.8
        self.distance_tolerance = 0.2

        self.gap_threshold = 0.7 
        self.min_gap_width = 3

        self.get_logger().info(f'Aşama 1 Hedefi (Ara Nokta): ({self.waypoint_x}, {self.waypoint_y})')

    def scan_callback(self, msg):
        self.latest_scan = msg
        if not self.scan_info_received:
            self.num_scan_points = len(msg.ranges)
            self.scan_angle_min = msg.angle_min
            self.scan_angle_increment = msg.angle_increment
            self.scan_info_received = True

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y),
            1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        )

    def find_best_gap(self, target_angle):
        if self.latest_scan is None: return None
        ranges = np.array(self.latest_scan.ranges); ranges[np.isinf(ranges) | np.isnan(ranges)] = 0
        gaps = []; in_gap = False; current_gap_start = 0
        for i, dist in enumerate(ranges):
            if dist > self.gap_threshold:
                if not in_gap: in_gap = True; current_gap_start = i
            else:
                if in_gap:
                    in_gap = False; gap_end = i - 1
                    if (gap_end - current_gap_start) >= self.min_gap_width: gaps.append((current_gap_start, gap_end))
        if in_gap:
            gap_end = len(ranges) - 1
            if (gap_end - current_gap_start) >= self.min_gap_width: gaps.append((current_gap_start, gap_end))
        if not gaps: return None
        best_gap = None; best_score = -float('inf')
        for gap in gaps:
            mid_idx = (gap[0] + gap[1]) // 2; gap_angle = self.scan_angle_min + mid_idx * self.scan_angle_increment
            angle_diff = abs(target_angle - gap_angle); score = 1.0 / (angle_diff + 0.1)
            if score > best_score: best_score = score; best_gap = gap
        if best_gap:
            mid_idx = (best_gap[0] + best_gap[1]) // 2
            return self.scan_angle_min + mid_idx * self.scan_angle_increment
        return None

    def control_loop(self):
        if self.position is None or not self.scan_info_received:
            return

        twist_msg = Twist()
        
        if self.state == State.GO_TO_WAYPOINT:
            distance_to_waypoint = math.sqrt((self.waypoint_x - self.position.x)**2 + (self.waypoint_y - self.position.y)**2)
            if distance_to_waypoint < self.distance_tolerance:
                self.get_logger().info('Ara nokta aşıldı! Ana hedefe yöneliniyor.')
                self.state = State.GO_TO_FINAL_GOAL
                return

            angle_to_waypoint = math.atan2(self.waypoint_y - self.position.y, self.waypoint_x - self.position.x)
            angle_diff = angle_to_waypoint - self.yaw
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            twist_msg.angular.z = self.max_angular_speed * 0.5 * angle_diff
            twist_msg.linear.x = self.max_linear_speed
        
        elif self.state == State.GO_TO_FINAL_GOAL:
            distance_to_target = math.sqrt((self.final_target_x - self.position.x)**2 + (self.final_target_y - self.position.y)**2)
            if distance_to_target < self.distance_tolerance:
                self.state = State.GOAL_REACHED
                return

            angle_to_goal = math.atan2(self.final_target_y - self.position.y, self.final_target_x - self.position.x)
            robot_centric_angle_to_goal = angle_to_goal - self.yaw
            robot_centric_angle_to_goal = math.atan2(math.sin(robot_centric_angle_to_goal), math.cos(robot_centric_angle_to_goal))
            best_gap_angle = self.find_best_gap(robot_centric_angle_to_goal)
            final_angle = 0.0
            if best_gap_angle is not None:
                final_angle = (robot_centric_angle_to_goal + best_gap_angle) / 2.0
            else:
                twist_msg.linear.x = -0.1
                twist_msg.angular.z = self.max_angular_speed * 0.8
                self.cmd_vel_pub.publish(twist_msg)
                return
            
            angle_diff = final_angle
            twist_msg.angular.z = self.max_angular_speed * angle_diff
            if abs(angle_diff) < 0.2: twist_msg.linear.x = self.max_linear_speed
            elif abs(angle_diff) < 0.8: twist_msg.linear.x = self.max_linear_speed * 0.5
            else: twist_msg.linear.x = 0.0

        elif self.state == State.GOAL_REACHED:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.INFO:
                self.get_logger().info('HEDEFE ULAŞILDI! GÖREV TAMAMLANDI.')
        
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
