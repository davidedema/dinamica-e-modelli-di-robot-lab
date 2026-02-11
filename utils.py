import numpy as np
from scipy.spatial.transform import Rotation as R

# DH parameters in millimeters
d_mm = np.array([131.22, 0, 0, 63.4, 75.05, 45.6])  # Link offsets (along Z axis) in mm
a_mm = np.array([0, -110.4, -96, 0, 0, 0])  # Link lengths (along X axis) in mm
alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])  # Link twists (around X axis)
offset = np.array([0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0])  # Joint angle offsets

# Convert DH parameters to meters (by dividing by 1000)
d = d_mm / 1000  # Link offsets in meters
a = a_mm / 1000  # Link lengths in meters

# def rotation_matrix_to_axis_angle(R_matrix):
#     # Ensure the matrix is a valid rotation matrix (orthogonal and determinant == 1)
#     if np.linalg.norm(R_matrix @ R_matrix.T - np.eye(3)) > 1e-6 or np.abs(np.linalg.det(R_matrix) - 1) > 1e-6:
#         raise ValueError("Input matrix is not a valid rotation matrix")

#     # Step 1: Compute the rotation angle (theta)
#     trace = np.trace(R_matrix)
#     theta = np.arccos((trace - 1) / 2)

#     # Step 2: Compute the axis of rotation (v)
#     if np.abs(theta) < 1e-6:  # If the angle is very small, we can assume no rotation
#         axis = np.array([1, 0, 0])  # Arbitrary axis, since no rotation
#     elif np.abs(theta - np.pi) < 1e-6:  # If the angle is 180 degrees, axis can be any unit vector
#         axis = np.array([1, 0, 0])  # You can choose any axis here
#     else:
#         axis = np.array([
#             R_matrix[2, 1] - R_matrix[1, 2],
#             R_matrix[0, 2] - R_matrix[2, 0],
#             R_matrix[1, 0] - R_matrix[0, 1]
#         ]) / (2 * np.sin(theta))

#     return axis, theta

def compute_A(a, alpha, d, theta):
    """
    Compute the homogeneous transformation matrix A_i for a single joint i
    using the Denavit-Hartenberg parameters and joint angle theta.
    Returns a 4x4 numpy array representing the transform from frame i-1 to i.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])


def compute_fk(joint_angles):
    """
    Compute the forward kinematics of the robot given the current joint angles.
    Multiplies the chain of transformations to get the end-effector pose relative to base frame.
    Returns a 4x4 homogeneous transformation matrix of the end-effector.
    """
    T = np.eye(4)  # initialize transformation as identity
    for i in range(6):
        theta = joint_angles[i] + offset[i]  # apply joint angle offset
        A = compute_A(a[i], alpha[i], d[i], theta)  # transform for joint i
        T = T @ A  # chain transform
    return T


def compute_jacobian(joint_angles):
    """
    Compute the geometric Jacobian matrix for the robot at given joint angles.
    Returns a 6x6 Jacobian matrix composed of:
     - Top 3 rows: linear velocity components (partial derivatives of EE position)
     - Bottom 3 rows: angular velocity components (rotation axes)
    """
    T = np.eye(4)
    Ts = [T]  # store transformations from base to each joint

    # Compute transformation matrices for all joints
    for i in range(6):
        theta = joint_angles[i] + offset[i]
        A = compute_A(a[i], alpha[i], d[i], theta)
        T = T @ A
        Ts.append(T)

    o_n = Ts[6][:3, 3]  # End-effector position (from last transform)
    Jp = []  # Linear velocity part of Jacobian
    Jo = []  # Angular velocity part of Jacobian

    for i in range(6):
        z_i = Ts[i][:3, 2]  # z-axis of the i-th joint frame (rotation axis)
        o_i = Ts[i][:3, 3]  # origin of the i-th joint frame
        Jp_i = np.cross(z_i, o_n - o_i)  # linear velocity Jacobian component
        Jp.append(Jp_i)
        Jo.append(z_i)  # angular velocity Jacobian component

    # Stack linear and angular parts vertically to form full Jacobian
    J = np.vstack((np.array(Jp).T, np.array(Jo).T))
    return J

def compute_control(q,pos_d,orient_d,K_gain,k_rot,dt):
    T = compute_fk(q)
    e = pos_d - T[:3, 3]
    e = K_gain @ e

    J_geom_ = compute_jacobian(q)

    ee_rot = T[:3, :3]

    error_rot = np.array(ee_rot.T @ orient_d)
    
    axis_angle = R.from_matrix(error_rot).as_rotvec()
    
    error_rot_axis = k_rot*(ee_rot @ axis_angle)

    des_vel = np.concatenate([e,error_rot_axis])
    dq =J_geom_.T@np.linalg.pinv(J_geom_@J_geom_.T + np.eye(des_vel.shape[0])*0.00010)@des_vel

    return q + dq*dt