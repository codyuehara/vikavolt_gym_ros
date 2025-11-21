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
        self.declare_parameter('map_path')
        self.declare_parameter('num_agent')
        self.declare_parameter('mass')
        self.declare_parameter('starting_x')
        self.declare_parameter('starting_y')
        self.declare_parameter('starting_z')
        self.declare_parameter('kb_teleop')
        self.declare_parameter('gate_1_position')
        self.declare_parameter('gate_1_orientation')
        self.declare_parameter('gate_2_position')
        self.declare_parameter('gate_2_orientation')
        self.declare_parameter('gate_3_position')
        self.declare_parameter('gate_3_orientation')

        # check num_agents
        num_agents = self.get_parameter('num_agent').value
        if num_agents < 1 or num_agents > 2:
            raise ValueError('num_agents should be either 1 or 2.')
        elif type(num_agents) != int:
            raise ValueError('num_agents should be an int.')

        mass = self.get_parameter('mass').value
        sx = self.get_parameter('starting_x').value
        sy = self.get_parameter('starting_y').value
        sz = self.get_parameter('starting_z').value
        init_position = np.array([sx, sy, sz])
        #self.velocity = [0.0, 0.0, 0.0]
        self.collision_flag = False
 
        self.gate_positions = []
        for i in range(1, 4):
            pos = self.get_parameter(f'gate_{i}_position').get_parameter_value().double_array_value
            quat = self.get_parameter(f'gate_{i}_orientation').get_parameter_value().double_array_value
            self.gate_positions.append(pos)
            #orientations.append(quat)
        self.get_logger().info(f"Gates loaded: {self.gate_positions}")
        # env backend
        self.env = gym.make("gymnasium_env/Vikavolt-v0", mass=mass, init_position=init_position, gate_positions=self.gate_positions)     

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
        #action = [1,2,3,4]
        action = np.array([10,10,10,10])
        self.obs, reward, terminated, truncated, info = self.env.step(action)
        #elif self.ego_drive_published and self.has_opp and self.opp_drive_published:
        #    self.obs, _, self.done, _ = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed], [self.opp_steer, self.opp_requested_speed]]))
        self._update_sim_state()

    def timer_callback(self):
        ts = self.get_clock().now().to_msg()
        self.get_logger().info(f"timer called : {ts}")

        # pub tf
        self._publish_odom(ts)
  #      self._publish_transforms(ts)
  #      self._publish_laser_transforms(ts)
  #      self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
   #     self.ego_scan = list(self.obs['scans'][0])

        #self.position[0] = self.obs['poses_x'][0]
        #self.position[1] = self.obs['poses_y'][0]
        #self.get_logger().info(f"OBS TYPE: {type(self.obs)}")
        #self.get_logger().info(f"OBS: {self.obs}")
        self.position = self.obs['pos']
        self.get_logger().info(f"position : {self.position}")
        self.velocity = self.obs['vel']
        #self.velocity[1] = self.obs['linear_vels_y'][0]
        #self.velocity[2] = self.obs['ang_vels_z'][0]
        

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
