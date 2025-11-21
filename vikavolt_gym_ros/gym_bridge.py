import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

import gymnasium as gym
import gymnasium_env
import numpy as np
from transforms3d import euler

class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        self.declare_parameter('ego_odom_topic')
        self.declare_parameter('ego_drive_topic')
        self.declare_parameter('collision_topic')
        self.declare_parameter('map_path', "")
        self.declare_parameter('num_agent', 1)
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('mass', 1.0)
        self.declare_parameter('starting_position', [0.0, 0.0, 0.0])
        self.declare_parameter('starting_orientation', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('kb_teleop', True)
        self.declare_parameter('gate_1_position', [5.0, 0.0, 1.0])
        self.declare_parameter('gate_1_orientation')
        self.declare_parameter('gate_2_position', [10.0, 0.0, 2.0])
        self.declare_parameter('gate_2_orientation')
        self.declare_parameter('gate_3_position', [15.0, 3.0, 5.0])
        self.declare_parameter('gate_3_orientation')

        dt = self.get_parameter('dt').value
        mass = self.get_parameter('mass').value
        #init_position = np.array([sx, sy, sz])
        start_pos = self.get_parameter(f'starting_position').get_parameter_value().double_array_value
        init_position = np.array(start_pos, dtype=float)
        init_orientation = self.get_parameter(f'starting_orientation').get_parameter_value().double_array_value
        self.collision_flag = False
 
        self.gate_positions = []
        for i in range(1, 4):
            pos = self.get_parameter(f'gate_{i}_position').get_parameter_value().double_array_value
            quat = self.get_parameter(f'gate_{i}_orientation').get_parameter_value().double_array_value
            self.gate_positions.append(pos)
            #orientations.append(quat)
        self.get_logger().info(f"Gates loaded: {self.gate_positions}")
        # env backend
        self.env = gym.make("gymnasium_env/Vikavolt-v0", mass=mass, init_position=init_position, gate_positions=self.gate_positions, dt=dt)     

        self.has_opp = False
        self.obs, info = self.env.reset(seed=42)
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        ego_odom_topic = self.get_parameter('ego_odom_topic').value
        collision_topic = self.get_parameter('collision_topic').value

        # sim physical step timer
        self.drive_timer = self.create_timer(1, self.drive_timer_callback)
        # topic publishing timer
        self.timer = self.create_timer(1, self.timer_callback)

        # transform broadcaster
        #self.br = TransformBroadcaster(self)

        # publishers
  #      self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        print("odom topic: ", ego_odom_topic)
        self.get_logger().info(f"odom_topic: {ego_odom_topic}")
        self.get_logger().info(f"collision_topic: {collision_topic}")
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_drive_published = False

        # subscribers
        self.collision_sub = self.create_subscription(
            String,
            collision_topic,
            self.collision_callback,
            10)

        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped,
            ego_drive_topic,
            self.drive_callback,
            10)
       # self.ego_reset_sub = self.create_subscription(
       #     PoseWithCovarianceStamped,
       #     '/initialpose',
       #     self.ego_reset_callback,
       #     10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)

    def collision_callback(self, collision_msg):
        self.collision_detected = collision_msg.data
        self.get_logger().info(f"collision callback called : {self.collision_detected}")


    def drive_callback(self, drive_msg):
        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        self.ego_drive_published = True

    def ego_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')

    def teleop_callback(self, twist_msg):
        if not self.ego_drive_published:
            self.ego_drive_published = True

        self.ego_requested_speed = twist_msg.linear.x

        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    def drive_timer_callback(self):
        action = np.array([10,10,10,10])
        self.obs, reward, terminated, truncated, info = self.env.step(action)
        self._update_sim_state()

    def timer_callback(self):
        ts = self.get_clock().now().to_msg()
        self.get_logger().info(f"timer called : {ts}")

        # pub tf
        self._publish_odom(ts)

    def _update_sim_state(self):
        self.position = self.obs['pos']
        self.get_logger().info(f"position : {self.position}")
        self.velocity = self.obs['vel']
        self.R = self.obs['R']
        self.prev_action = self.obs['prev_action']
        self.get_logger().info(f"R : {self.R}")
        self.get_logger().info(f"prev_action : {self.prev_action}")
        

    def _publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts

        ego_odom.header.frame_id = 'map'
        ego_odom.pose.pose.position.x = self.position[0]
        ego_odom.pose.pose.position.y = self.position[1]
        ego_odom.pose.pose.position.z = self.position[2]
        ego_quat = euler.euler2quat(0., 0., self.position[2], axes='sxyz')
        ego_odom.pose.pose.orientation.x = ego_quat[1]
        ego_odom.pose.pose.orientation.y = ego_quat[2]
        ego_odom.pose.pose.orientation.z = ego_quat[3]
        ego_odom.pose.pose.orientation.w = ego_quat[0]
        ego_odom.twist.twist.linear.x = self.velocity[0]
        ego_odom.twist.twist.linear.y = self.velocity[1]
        ego_odom.twist.twist.linear.z = self.velocity[2]
        self.ego_odom_pub.publish(ego_odom)


def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
