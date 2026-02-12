import numpy as np
from scipy.spatial.transform import Rotation as R
from pymycobot import MyCobot280
from utils import compute_fk, compute_control, bcolors
import time
import os

ROOT = os.path.dirname(os.path.abspath(__file__))  

class CobotController():
    def __init__(self, simulation=True, serial_port='/dev/ttyACM0'):
        # Flags and buffers
        self.q_ref = None                       # initial joint positions
        self.speed = 10.0                       # joint velocity 
        self.deltaT = 0.05                      # Control loop time step (s)
        self.simulation = simulation
        self.lower_bound = [0.2, -0.15]           # Workspace lower bounds (x, y)
        self.upper_bound = [0.3, 0.15]            # Workspace upper
        if not self.simulation:
            self.mc = MyCobot280(serial_port)       # Initialize MyCobot connection
            print(f"{bcolors.BOLD}{bcolors.OKCYAN}[MODE] REAL ROBOT MODE{bcolors.ENDC}")
            print(f"{bcolors.OKGREEN}[STAT] Connected to MyCobot on {serial_port}{bcolors.ENDC}")
        else:
            from mujoco_sim import MujocoSim
            print(f"{bcolors.BOLD}{bcolors.OKBLUE}[MODE] SIMULATION MODE{bcolors.ENDC}")
            self.mc = MujocoSim(os.path.join(ROOT, 'description', 'scene.xml'))
            self.mc.set_frame_vis("body")

        self.homing_procedure()
        self.read_first_pose()

    def homing_procedure(self):
        initial_q = [0, 0, 0, 0, 0, 0]                          # Home at zero position
        print(f"{bcolors.OKBLUE}[INFO] Homing procedure: moving to initial position {initial_q}{bcolors.ENDC}")
        if not self.simulation:
            self.mc.sync_send_angles(initial_q, int(self.speed))    #
            self.mc.sync_send_angles(initial_q, int(self.speed))    #
            print(f"{bcolors.OKGREEN}[STAT] Homing completed. Robot is at initial position.{bcolors.ENDC}")
        else:
            # interpolate a trajectory from current joint positions to initial_q for smooth homing  
            current_q = self.mc.get_state()[0]
            homing_traj = np.linspace(current_q, initial_q, 50)
            for q in homing_traj:
                self.mc.step(q, self.deltaT)
                time.sleep(self.deltaT)
            print(f"{bcolors.OKGREEN}[STAT] Homing completed. Simulation is at initial position.{bcolors.ENDC}")
        time.sleep(1)

    def read_first_pose(self):
        if not self.simulation:
            self.q0 = self.mc.get_radians()
            if self.q0 == -1:
                print(f"{bcolors.FAIL}[ERROR] Failed to read initial joint positions. Check connection.{bcolors.ENDC}")
                raise RuntimeError("Failed to read initial joint positions from the robot.")
        else:
            self.q0 = self.mc.get_state()[0]  # Get initial joint positions from simulation
        self.q_ref = np.array(self.q0)
        print(f"{bcolors.OKGREEN}[STAT] Joint position received, trajectory can be now computed.{bcolors.ENDC}")

    def compute_trajectory(self, pos):
        """
        Reach a fixed position and then draw a circle
        Computes a circular trajectory in the workspace for the end-effector,
        starting from its current pose (computed via forward kinematics).
        """
        
        if len(pos) != 2:
            print(f"{bcolors.FAIL}[ERROR] Position must be a 2D point (x, y).{bcolors.ENDC}")
            raise ValueError("Position must be a 2D point (x, y).")

        # check if pos is inside the workspace
        if pos[0] < self.lower_bound[0] or pos[0] > self.upper_bound[0]:
            print(f"{bcolors.FAIL}[ERROR] Position is outside the x workspace bounds.{bcolors.ENDC}")
            raise ValueError("Position is outside the x workspace bounds.")
        if pos[1] < self.lower_bound[1] or pos[1] > self.upper_bound[1]:
            print(f"{bcolors.FAIL}[ERROR] Position is outside the y workspace bounds.{bcolors.ENDC}")
            raise ValueError("Position is outside the y workspace bounds.")
        
        orient_d = np.array([[0,-1,0],
                             [-1,0,0],
                             [0,0,-1]])
        orient_d = R.from_matrix(orient_d).as_rotvec()

        pos_0 = compute_fk(self.q_ref)[:3,3]
        print(f"{bcolors.OKBLUE}[INFO] Current end-effector position: {pos_0}{bcolors.ENDC}")
        pos_target = np.array([pos[0], pos[1], 0.033])  # Desired end-effector position (x, y, z)
        pos_traj = np.linspace(pos_0,pos_target,50)
        approaching = np.vstack((pos_traj,np.tile(pos_target,(50,1))))
        pos_traj = np.hstack((approaching, np.tile(orient_d,(approaching.shape[0],1))))
        self.raw_traj = pos_traj

        # Downsample and pad the trajectory to smooth it at the start
        self.downsampled_traj = self.raw_traj[::1].tolist()
        self.downsampled_traj.insert(0, self.raw_traj[0].tolist())
        self.downsampled_traj.insert(0, self.raw_traj[0].tolist())
        self.downsampled_traj.insert(0, self.raw_traj[0].tolist())
        self.downsampled_traj.append(self.raw_traj[-1].tolist())
        self.downsampled_traj = np.array(self.downsampled_traj)

        self.downsampled_traj = self.raw_traj

        self.total_steps = self.downsampled_traj.shape[0]

        print(f"{bcolors.OKBLUE}[INFO] Trajectory computed with {self.total_steps} steps.{bcolors.ENDC}")
        print(f"{bcolors.OKBLUE}[INFO] Total execution time (approx): {self.total_steps * self.deltaT:.2f} seconds.{bcolors.ENDC}")
        print(f"{bcolors.OKGREEN}[STAT] Trajectory can be now executed with the controller.{bcolors.ENDC}")

    def open_gripper(self):
        if not self.simulation:
            self.mc.set_gripper_value(50, 20)
        else:
            print(f"{bcolors.WARNING}[WARN] Gripper not supported in simulation mode.{bcolors.ENDC}")
        
    def close_gripper(self):
        if not self.simulation:
            self.mc.set_gripper_value(10, 20)
        else:
            print(f"{bcolors.WARNING}[WARN] Gripper not supported in simulation mode.{bcolors.ENDC}")

    def _controller(self):
        """
        Computes joint commands by inverse kinematics and execute them.
        """
        if self.downsampled_traj is None:
            raise RuntimeError("Trajectory not computed.")
        self.old_q_d = np.copy(self.q_ref)  
        start = time.time()
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

            Kp_pos = 6
            Kp_ori = 1

            self.q_d = compute_control(self.old_q_d,pos_d,ori_d,np.eye(3)*Kp_pos,Kp_ori,self.deltaT)
            if not self.simulation:
                self.mc.send_radians(self.q_d, int(self.speed))
                self.old_q_d = np.copy(self.q_d)
            else:
                self.mc.step(self.q_d, self.deltaT)
                q_current = self.mc.get_state()[0]
                self.old_q_d = np.copy(q_current)
            time.sleep(self.deltaT)

        print(f"{bcolors.OKGREEN}[STAT] Trajectory execution completed in {time.time() - start:.2f} seconds.{bcolors.ENDC}")
        print(f"{bcolors.OKGREEN}[STAT] Final EE position: {compute_fk(self.q_d)[:3,3]}{bcolors.ENDC}")
        time.sleep(1)  
        self.homing_procedure()
        
    def execute_trajectory(self, n_times):
        for _ in range(n_times):
            self._controller()

    def mount_pen(self):
        if not self.simulation:
            print(f"{bcolors.OKBLUE}[INFO] Mounting pen: opening gripper and waiting for user input...{bcolors.ENDC}")
            self.open_gripper()
            input("Insert the marker in the gripper and press Enter to continue...")
            self.close_gripper()
            print(f"{bcolors.OKGREEN}[STAT] Pen mounted successfully.{bcolors.ENDC}")
        else:
            print(f"{bcolors.WARNING}[WARN] Pen mounting not supported in simulation mode.{bcolors.ENDC}")
    
    def remove_pen(self):
        if not self.simulation:
            print(f"{bcolors.OKBLUE}[INFO] Removing pen: opening gripper and waiting for user input...{bcolors.ENDC}")
            input("Press enter and take the marker out of the gripper to finish...")
            self.open_gripper()
            print(f"{bcolors.OKGREEN}[STAT] Pen removed successfully.{bcolors.ENDC}")
        else:
            print(f"{bcolors.WARNING}[WARN] Pen removal not supported in simulation mode.{bcolors.ENDC}")