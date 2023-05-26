from time import time
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from mpc_tracking_controller import MPCDiffDriveControl 

class MPCControl():
    def __init__(self, delta_t, min_error):
        print("Initializing the MPC controller")
        self.pub_velocity = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odometry/filtered', Odometry, self.set_pose)
        self.i = 0
        self.set_q_init = None
        self.q = None 
        self.r = 0.3 # Wheel radius
        self.L = 1.25 # Axle length
        self.D = 0.07 # Distance between the front front whell and rear axle
        self.Ts = delta_t # Sampling time
        self.t = np.arange(0, 10, self.Ts) # Simulation time
        self.end_controller = False
        # self.timer = rospy.Timer(rospy.Duration(self.Ts), self.timer_callback)
        self.odom_pose = None 
        self.error = None
        self.success = False 
        self.min_acceptable_error = min_error 

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_velocity.publish(msg)
        
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2*np.pi)
        mask = np.abs(xwrap)>np.pi
        xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
        return xwrap[0]
    
    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        if(self.set_q_init is None):
            self.set_q_init = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.q = self.set_q_init
        self.odom_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
        self.odom_control = np.array([msg.twist.twist.linear.x, msg.twist.twist.angular.z])
        
    def get_current_pose(self, ):
        return self.odom_pose, self.odom_control 
    
    def mpc_planner_init(self, target_pose):
        self.mpc_solver = MPCDiffDriveControl(self.Ts, 20, 1.0)
        self.target_pose = target_pose
        if(self.q is None):
            print("Still robot current pose is not set")
        else:
            self.mpc_solver.init_regulator(self.q, target_pose)
        
    def move_one_step(self, ):
        u = np.array([-2000, -2000])
        x_pred = np.array([-2000, -2000, 2000])
        if(self.mpc_solver.init_reg):
            u, x_pred = self.mpc_solver.update(self.odom_pose)
            v = u[0]
            w = u[1]
            self.send_vel(v, w)
        elif(self.q is not None):
            self.mpc_solver.init_regulator(self.q, self.target_pose)
        
        if(self.odom_pose is not None):
            self.error = np.linalg.norm(self.odom_pose-self.target_pose)
            print("Error: ", self.error, " current state : " , self.odom_pose, "target state: ", self.target_pose)
            if(self.error< self.min_acceptable_error):
                self.success = True 
        return u, x_pred 
        
    # def timer_callback(self, msg):
    #     self.mpc_planner()
    #     return