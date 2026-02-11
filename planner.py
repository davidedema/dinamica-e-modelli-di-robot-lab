import numpy as np
from scipy.spatial.transform import Rotation as R
from pymycobot import MyCobot280
from utils import compute_fk, compute_control
import time
class TrajectoryPlanner():
    def __init__(self):
        # Flags and buffers
        self.q_ref = None                       # To store the initial joint positions
        self.index = 0                          # Index for publishing trajectory points
        self.trajectory = None                  # Placeholder for trajectory

        self.deltaT = 0.05                      # Control loop time step (s)
        self.lmb = 1e-2                         # Damping factor for Jacobian pseudo-inverse

        self.index = 0                          # trajectory index (if using trajectories)
        self.speed = 10.0                       # joint velocity for publishing (dummy constant speed)

        self.pos_d_seq = []
        self.q_d_seq = []
        self.or_d_seq = []
        self.mc = MyCobot280('/dev/ttyACM0')
        self.send_initial_position()
        self.read_first_pose()

    def send_initial_position(self):
        initial_q = [0, 0, 0, 0, 0, 0]  # Example initial joint angles (radians)
        self.mc.sync_send_angles(initial_q, int(self.speed), )
        time.sleep(2)  # Wait for the robot to reach the initial position

    def read_first_pose(self):
        self.q0 = self.mc.get_radians()

        if self.q0 == -1:
            print("Failed to read initial joint positions. Check connection.")
            return
        self.q_ref = np.array(self.q0)
        print("Initial joint state received. Computing trajectory...")
        self.compute_trajectory()

    def compute_trajectory(self):
        """
        Reach a fixed position and then draw a circle
        Computes a circular trajectory in the workspace for the end-effector,
        starting from its current pose (computed via forward kinematics).
        """

        # --- Circular Position Trajectory ---
        radius = 0.04 
        n_points = 100
        angles = np.linspace(0, 2 * np.pi, n_points )
        circle_numbers=2

        orient_d = np.array([[0,-1,0],
                             [-1,0,0],
                             [0,0,-1]])
        orient_d = R.from_matrix(orient_d).as_rotvec()

        pos_0 = compute_fk(self.q_ref)[:3,3]

        print(f"Pos_0 {pos_0}")

        pos_init =  np.array([0.22,0,0.033])

        # pos_traj = np.tile(pos_init,(50,1))
        pos_traj = np.linspace(pos_0,pos_init,50)

        approaching = np.vstack((pos_traj,np.tile(pos_init,(50,1))))

        # pos_traj = np.hstack(( np.vstack((pos_traj,np.tile(pos_init,(50,1)))), np.tile(orient_d,(pos_traj.shape[0],1))))

        pos_traj = np.hstack((approaching, np.tile(orient_d,(approaching.shape[0],1))))

        circle_center = pos_traj[-1,:3] + np.array([0,radius,0])
        trajectory_circle = circle_center  + radius*np.vstack((np.sin(angles),-np.cos(angles),np.zeros(n_points))).T
        trajectory_circle = np.hstack((trajectory_circle,np.tile(orient_d,(trajectory_circle.shape[0],1))))

        # --- Full 6D Trajectory: position + orientation ---
        self.raw_traj = np.vstack((pos_traj, np.tile(trajectory_circle,(circle_numbers,1))))

        # Downsample and pad the trajectory to smooth it at the start
        self.downsampled_traj = self.raw_traj[::1].tolist()
        self.downsampled_traj.insert(0, self.raw_traj[0].tolist())
        self.downsampled_traj.insert(0, self.raw_traj[0].tolist())
        self.downsampled_traj.insert(0, self.raw_traj[0].tolist())
        self.downsampled_traj.append(self.raw_traj[-1].tolist())
        self.downsampled_traj = np.array(self.downsampled_traj)

        self.downsampled_traj = self.raw_traj

        self.total_steps = self.downsampled_traj.shape[0]

        print(f"Trajectory computed with {self.total_steps} steps.")
        print(f"Trajectory content: \n{self.downsampled_traj[:,:3]} ")

        np.save('x_trajectory.npy',self.raw_traj)
        self.controller()

    def controller(self):
        """
        Callback triggered on receiving a desired end-effector pose.
        Computes joint commands by inverse kinematics and publishes them.
        """
        self.old_q_d = np.copy(self.q_ref)  
        for i in range(len(self.downsampled_traj)):
            X = self.downsampled_traj[i]
            pos_d = X[0:3]
            quat = R.from_rotvec(X[3:6]).as_quat()
            R_d = R.from_quat([
                quat[0],
                quat[1],
                quat[2],
                quat[3]
            ])
       
            ori_d = R_d.as_matrix()

            # self.get_logger().info(f"pos_d = {pos_d}")
            # self.get_logger().info(f"or_d = {ori_d}")

            Kp_pos = 6
            Kp_ori = 1

            self.q_d = compute_control(self.old_q_d,pos_d,ori_d,np.eye(3)*Kp_pos,Kp_ori,self.deltaT)

            print(self.q_d)
            print(self.speed)
        
            self.mc.send_radians(self.q_d, int(self.speed))
            time.sleep(0.05)
            self.old_q_d = np.copy(self.q_d)
        
        self.send_initial_position()
        
