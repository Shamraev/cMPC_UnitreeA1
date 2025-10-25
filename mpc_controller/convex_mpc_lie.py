OSQP = 0
QPOASES = 1

import numpy as np
import math
import scipy.linalg as la
from enum import Enum
from typing import Union, List, Tuple, Optional
import cvxpy as cvx

from mpc_controller.lie_algebra_utils import *

# Formulas are from these papers:
# paper [1]: S. Teng, D. Chen, W. Clark, and M. Ghaffari, "An Error-State Model Predictive Control on Connected Matrix Lie Groups for Legged Robot Control," Jan. 22, 2023. arXiv: 2203.08728.
# paper [2]: J. Sola, J. Deray, and D. Atchuthan, "A micro Lie theory for state estimation in robotics," 2018. arXiv: 1812.01537.


def ihlqr(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray,
          max_iter: int = 1000, tol: float = 1e-5) -> Tuple[np.ndarray, np.ndarray]:
    """Iterative Linear Quadratic Regulator"""
    # Get size of x and u from B
    nx, nu = B.shape
    
    # Initialize P with Q
    P = Q.copy()
    
    # Ricatti iteration
    for ricatti_iter in range(max_iter):
        K = la.inv(R + B.T @ P @ B) @ B.T @ P @ A
        P_k = Q + A.T @ P @ (A - B @ K)
        
        if la.norm(P_k - P) <= tol:
            return P_k, K
        
        P = P_k
    
    raise RuntimeError("ihlqr did not converge")

def GetLocalTrajLie(T_traj, xi_traj, Td0):
    """Transform trajectory to local frame"""
    # from Global CS to Td0
    a, b, N = T_traj.shape
    T_traj_new = np.zeros((a, b, N))
    for k in range(N):
        T = T_traj[:, :, k]
        T_new = la.inv(Td0) @ T  # Reorthogonalisation is not necessary to use here
        T_traj_new[:, :, k] = T_new
    return GetTrajLie(T_traj_new, xi_traj)

def GetLocalXstate(x, Td0):  # x=[[p;q];xi], q - quaternion
    """Transform state to local frame"""
    # from Global CS to Td0
    r = x[:3]
    q = x[3:7]  # quaternion
    v = x[7:10]
    ω = x[10:13]
    xi = np.concatenate([v, ω])
    
    # Transform in SE3 not se3, don't use Adjoints here
    T = np.block([
        [qtoQ(q), r.reshape(-1, 1)],
        [np.zeros(3), 1]
    ])
    T_local = la.inv(Td0) @ T
    r_local = T_local[:3, 3]
    q_local = Qtoq(T_local[:3, :3])
    
    x_local = np.concatenate([r_local, q_local, xi])
    return x_local

def GetSE3_M(x):  # x=[[p;q];xi], q - quaternion
    """Convert state to SE(3) matrix"""
    r = x[:3]
    q = x[3:7]  # quaternion
    v = x[7:10]
    ω = x[10:13]
    
    T = np.block([
        [qtoQ(q), r.reshape(-1, 1)],
        [np.zeros(3), 1]
    ])
    return T

def GetTrajLie(T_traj, xi_traj):  # output: X=[zeta,xi], X in R^12
    """Convert trajectory to Lie algebra representation"""
    a, b, N = T_traj.shape
    assert (a, b) == (4, 4)  # SE3 element is 4x4 matrix
    a_xi, N_xi = xi_traj.shape
    assert N == N_xi  # counts of elements of T_traj and xi_traj are the same
    
    X = np.zeros((12, N))
    for k in range(N):
        T = T_traj[:, :, k]
        zeta = Log_SE3(T)
        xi = xi_traj[:, k]
        X[:, k] = np.concatenate([zeta, xi])
    return X

def sim2control_state(x):
    """Convert simulation state to control state"""
    r = x[:3]
    q = x[3:7]  # quaternion
    v = x[7:10]
    ω = x[10:13]
    
    T = np.block([
        [qtoQ(q), r.reshape(-1, 1)],
        [np.zeros(3), 1]
    ])
    zeta = Log_SE3(T)
    xi = np.concatenate([v, ω])
    return np.concatenate([zeta, xi])

def getX_errorMPC(x, xd):  # for error-state MPC, x - quat, xd - lie algebra
    """Get error state for MPC"""
    r = x[:3]
    q = x[3:7]  # quaternion
    v = x[7:10]
    ω = x[10:13]
    
    T = np.block([
        [qtoQ(q), r.reshape(-1, 1)],
        [np.zeros(3), 1]
    ])
    Td = Exp_SE3(xd[:6])
    zetaE = Log_SE3(la.inv(Td) @ T)
    xi = np.concatenate([v, ω])
    return np.concatenate([zetaE, xi])
    
def error_state_lie(x, x_d, xic, costN, error_state_MPC=False):
    """Compute error state in Lie algebra"""
    if not error_state_MPC:
        zeta = x[:6]
        zeta_d = x_d[:6]
        xi = x[6:12]
        xi_d = x_d[6:12]
        
        if costN == 1:  # 1st cost
            zeta_delta = -zeta_d + Jac_r_inv_SE3(-zeta_d) @ zeta
            xi_delta = xi - xi_d
        elif costN == 1.5:
            zeta_delta = -zeta_d + Jac_r_inv_SE3(-zeta_d) @ zeta
            xi_delta = xi - xi_d - AdSE3_v(xi_d) @ zeta_delta
        elif costN == 2:  # 2nd cost
            # position
            X_d = Exp_SE3(x_d)
            X_c = GetSE3_M(xic)
            zeta_c = Log_SE3(X_c)
            zeta_cd = Log_SE3(la.inv(X_d) @ X_c)
            
            zeta_delta = zeta_cd + Jac_r_inv_SE3(zeta_cd) @ (-zeta_c + Jac_r_inv_SE3(-zeta_c) @ zeta)
            # velocity
            Ad = (np.eye(6) + AdSE3_v(zeta_c - Jac_r_inv_SE3(-zeta_c) @ zeta)) @ AdSE3(la.inv(X_c) @ X_d)
            xi_delta = xi - Ad @ xi_d
        elif costN == 2.5:
            X_d = Exp_SE3(x_d)
            X_c = GetSE3_M(xic)
            zeta_c = Log_SE3(X_c)
            zeta_cd = Log_SE3(la.inv(X_d) @ X_c)
            
            zeta_delta = zeta_cd + Jac_r_inv_SE3(zeta_cd) @ (-zeta_c + Jac_r_inv_SE3(-zeta_c) @ zeta)
            # velocity
            xi_delta = xi - xi_d - AdSE3_v(xi_d) @ zeta_delta
        elif costN == 3:  # 3rd cost
            # get position
            p_c = xic[:3]
            p_d = x_d[:3]
            
            # Desired and Current Matrices on the manifold (Lie group elements)
            X_c = np.block([
                [qtoQ(xic[3:7]), Jac_l(q2phi(xic[3:7])) @ p_c.reshape(-1, 1)],
                [np.zeros(3), 1]
            ])
            
            X_d = np.block([
                [Exp_SO3(zeta_d[3:6]), Jac_l(zeta_d[3:6]) @ p_d.reshape(-1, 1)],
                [np.zeros(3), 1]
            ])
            
            # The inverse of the error on the manifold
            Psi_inv = la.inv(la.inv(X_d) @ X_c)
            
            # error using Adjoint matrix
            zeta_delta = zeta - AdSE3(Psi_inv) @ zeta_d
            xi_delta = xi - AdSE3(Psi_inv) @ xi_d
        elif costN == 3.5:
            # get position
            p_c = xic[:3]
            p_d = x_d[:3]
            
            # Desired and Current Matrices on the manifold (Lie group elements)
            X_c = np.block([
                [qtoQ(xic[3:7]), Jac_l(q2phi(xic[3:7])) @ p_c.reshape(-1, 1)],
                [np.zeros(3), 1]
            ])
            
            X_d = np.block([
                [Exp_SO3(zeta_d[3:6]), Jac_l(zeta_d[3:6]) @ p_d.reshape(-1, 1)],
                [np.zeros(3), 1]
            ])
            
            # The inverse of the error on the manifold
            Psi_inv = la.inv(la.inv(X_d) @ X_c)
            
            zeta_delta = -zeta_d + Jac_r_inv_SE3(-zeta_d) @ zeta
            xi_delta = xi - AdSE3(Psi_inv) @ xi_d
        elif costN == 4:
            zeta_delta = zeta - zeta_d
            xi_delta = xi - xi_d
        else:
            zeta_delta = np.zeros(6)
            xi_delta = np.zeros(6)
        
        return zeta_delta, xi_delta
    else:
        zetaE = x[:6]
        xi = x[6:12]
        xi_d = x_d[6:12]
        zetaE_dot = -AdSE3_v(xi_d) @ zetaE + xi - xi_d
        return zetaE, zetaE_dot

def rpy_rate_to_angular_velocity(rpy, rpy_rate):
    """
    Convert RPY rates to angular velocity in body frame using vector inputs
    
    Parameters:
    -----------
    rpy : numpy.ndarray or list
        [roll, pitch, yaw] Euler angles in radians
    rpy_rate : numpy.ndarray or list  
        [roll_rate, pitch_rate, yaw_rate] in rad/s
    
    Returns:
    --------
    numpy.ndarray
        Angular velocity vector [p, q, r] in body frame (rad/s)
    """
    # Extract components
    roll, pitch, yaw = rpy
    roll_rate, pitch_rate, yaw_rate = rpy_rate
    
    # Transformation matrix from RPY rates to body angular velocities
    T = np.array([
        [1, 0, -math.sin(pitch)],
        [0, math.cos(roll), math.sin(roll) * math.cos(pitch)],
        [0, -math.sin(roll), math.cos(roll) * math.cos(pitch)]
    ])
    
    # Calculate angular velocity in body frame
    angular_velocity = T @ np.array(rpy_rate)
    
    return angular_velocity

def angular_velocity_to_rpy_rate(rpy, angular_velocity):
    """
    Convert angular velocity in body frame to RPY rates using vector inputs
    
    Parameters:
    -----------
    rpy : numpy.ndarray or list
        [roll, pitch, yaw] Euler angles in radians
    angular_velocity : numpy.ndarray or list
        [p, q, r] angular velocity components in body frame (rad/s)
    
    Returns:
    --------
    numpy.ndarray
        RPY rates vector [roll_rate, pitch_rate, yaw_rate] in rad/s
    """
    # Extract components
    roll, pitch, yaw = rpy
    p, q, r = angular_velocity
    
    # Avoid singularity at pitch = ±90° (gimbal lock)
    if abs(math.cos(pitch)) < 1e-10:
        raise ValueError("Near gimbal lock condition (pitch ≈ ±90°)")
    
    # Inverse transformation matrix
    T_inv = np.array([
        [1, math.sin(roll) * math.tan(pitch), math.cos(roll) * math.tan(pitch)],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll) / math.cos(pitch), math.cos(roll) / math.cos(pitch)]
    ])
    
    # Calculate RPY rates
    rpy_rates = T_inv @ np.array(angular_velocity)
    
    return rpy_rates

def desired_trajectory(T0, xi_di,Nt, dt):
    """
    Output: T_d, xi_d. T_d is SE3, xi_d is local velocity
    for xi=[v,w]
    """   
    # constant xi_d traj
    
    xi_d = np.zeros((6, Nt))
    T_d = np.zeros((4, 4, Nt))
    t = 0
    
    for k in range(Nt):
        # not constant xi_d traj, set Tfinal = 13.0
        # xi_di = np.array([1, 0.5*np.cos(2*2*np.pi*(Nt-k)/Nt), 0, 0, 0, 0])
        
        T = T0 @ Exp_SE3(xi_di * t)
        T_d[:, :, k] = T
        xi_d[:, k] = xi_di
        t = k * dt
    
    return T_d, xi_d

class ConvexMpc:
    def __init__(
        self,
        body_mass,
        body_inertia_list,
        num_legs,
        planning_horizon_steps,
        planning_timestep,
        weights_list,
        regularization_weight=1e-5,
        qp_solver=None
    ):
        """
        Convex MPC controller for legged locomotion.
        
        Args:
            body_mass: Mass of the robot body (kg)
            body_inertia_list: List of inertia tensor components [Ixx, Iyy, Izz] or full 3x3 matrix
            num_legs: Number of legs on the robot
            planning_horizon_steps: Number of steps in the planning horizon
            planning_timestep: Time duration of each planning step (seconds)
            weights_list: List of weights for cost function terms
            qp_solver: QP solver instance (if None, will need to be set later)
        """
        self._body_mass = body_mass
        self._body_inertia = np.array(body_inertia_list).reshape(3, 3)
        self._num_legs = num_legs
        self._PLANNING_HORIZON_STEPS = planning_horizon_steps
        self._PLANNING_TIMESTEP = planning_timestep
        self._weights = weights_list
        self._qp_solver = qp_solver

        self._foot_positions_base_frame = np.array(12)
        self._foot_contact_state = np.array(4)
        self._R = np.identity(3)
        self._foot_friction_coeffs = [0.7,0.7,0.7,0.7]
        # Initialize other necessary variables
        self._setup_mpc_problem()
     
    def _setup_mpc_problem(self):
        """Setup the MPC problem matrices and constraints."""
        # This would typically involve:
        # 1. Creating system dynamics matrices (A, B)
        # 2. Setting up cost matrices (Q, R)
        # 3. Initializing constraint matrices
        # 4. Preparing QP problem formulation
        
        # Placeholder implementation
        self._Nx = 12  
        self._Nu = 12  # Typically 3 forces per foot × 4 feet = 3 * self._num_legs  # 3D force per leg
    
    def compute_contact_forces(
        self,
        com_position,  # [x,y,z] in world frame
        com_velocity,  # [x_dot,y_dot,z_dot] in body frame
        com_roll_pitch_yaw,  # [roll, pitch, yaw] in world frame
        com_roll_pitch_yaw_rate,  # in body frame
        foot_contact_state,  # [c1, c2, c3, c4] contact states for 4 legs
        foot_positions_base_frame,  # [p1_x,p1_y,p1_z,...,p4_x,p4_y,p4_z] in body frame
        foot_friction_coeffs,  # [k1,k2,k3,k4]
        desired_com_position,  # desired_com_position in world frame
        desired_com_velocity,  # desired_linear_velocity in body frame
        desired_com_roll_pitch_yaw,  # desired_com_roll_pitch_yaw
        desired_com_angular_velocity  # desired_com_angular_velocity
    ):
        """
        Compute optimal contact forces using convex MPC.
        
        Returns:
            contact_forces: Array of contact forces for each leg in world frame
                            Shape: (num_legs * 3,) - [f1_x, f1_y, f1_z, f2_x, ...]
        """
        
        # # 1. Compute rotation matrix from body to world frame
        # R_body_to_world = self._rpy_to_rotation_matrix(com_rpy)
        
        # # 2. Formulate the current state vector
        # current_state = self._formulate_state_vector(
        #     com_pos, com_vel, com_rpy, com_ang_vel
        # )
        
        # # 3. Formulate the desired state vector
        # desired_state = self._formulate_state_vector(
        #     desired_pos, desired_vel, desired_rpy, desired_ang_vel
        # )
        
        # # 4. Build the MPC problem
        # qp_problem = self._build_mpc_problem(
        #     current_state,
        #     desired_state,
        #     foot_contact_state,
        #     foot_positions,
        #     friction_coeffs,
        #     R_body_to_world
        # )
        
        # # 5. Solve the QP problem
        # contact_forces = self._solve_mpc(
        #     current_state, desired_state, foot_contact_state, R_body_to_world
        # )       
        # return contact_forces

        self._foot_positions_base_frame = foot_positions_base_frame
        self._foot_contact_state = foot_contact_state
        self._R = self._rpy_to_rotation_matrix(com_roll_pitch_yaw)
        self._foot_friction_coeffs = foot_friction_coeffs

        error_state_MPC = False
        N_mpc = self._PLANNING_HORIZON_STEPS
        # com_position = np.zeros(3)
        # temp, it is not correct, and it is not necessary for full-state lie mpc
        x_d = np.concatenate([
            desired_com_position,
            desired_com_roll_pitch_yaw, 
            desired_com_velocity,
            desired_com_angular_velocity
        ])
        
        # Td0?? make relative trajectory
        # x_c = self.get_zeta(com_position,self._R) + self.get_xi(com_velocity,com_roll_pitch_yaw,com_roll_pitch_yaw_rate)
        
        # ??# discard of xic with quaternions
        q = Qtoq(self._R)
        xi = self.get_xi(com_velocity,com_roll_pitch_yaw,com_roll_pitch_yaw_rate)
        xic = np.concatenate([
            com_position,
            q, 
            xi
        ])

        T_d_traj,xi_d_traj = self.getDesTraj(com_position,self._R,x_d[6:12],N_mpc,self._PLANNING_TIMESTEP)
        X_ref_window = self.getDesLieTraj(com_position,self._R,x_d[6:12],N_mpc,self._PLANNING_TIMESTEP)
        Td0 = np.identity(4)
        if not error_state_MPC:                    
            Td0 = T_d_traj[:,:,1] #  Exp_SE3(X_ref_tilde[1:6,1]) #GetSE3_M(x_c)            
            X_ref_window=GetLocalTrajLie(T_d_traj,xi_d_traj,Td0) 
            xic=GetLocalXstate(xic,Td0) 
        
        Ad,Bd,Termd = self.get_AB(x_d, sim2control_state(xic), error_state_MPC)
        # self._weights = (roll_pitch_yaw, position, angular_velocity, velocity, gravity_place_holder)
        Q_diag = np.concatenate([self._weights[3:6],self._weights[:3],self._weights[9:12],self._weights[6:9]])
        Q = np.diag(Q_diag) 
        R = 0.00001*np.identity(12)
        # temp

       
        costN = 1.5         
        return self.convex_mpc(Ad,Bd,Termd, Q, R,
               X_ref_window, xic,
               N_mpc, costN, error_state_MPC)
        # return np.zeros(12)

    def getDesTraj(self,p,R,xi_d,Nt,dt):
        T = np.identity(4)
        T[:3, :3] = R
        T[:3, 3] = p
        T_d_traj,xi_d_traj=desired_trajectory(T, xi_d,Nt, dt)    
        return T_d_traj,xi_d_traj

    def getDesLieTraj(self,p,R,xi_d,Nt,dt):
        T = np.identity(4)
        T[:3, :3] = R
        T[:3, 3] = p
        T_d_traj,xi_d_traj=desired_trajectory(T, xi_d,Nt, dt)    
        return GetTrajLie(T_d_traj, xi_d_traj)
    def get_zeta(self,p,R):
        T = np.identity(4)
        T[:3, :3] = R
        T[:3, 3] = p
        return Log_SE3(T)

    def get_xi(self,v,rpy,rpy_rate):
        w = rpy_rate_to_angular_velocity(rpy,rpy_rate)
        return np.concatenate([v,w])

    # Linearized dynamics
    def get_AB(self,x_d, x_c, error_state_MPC=False):
        """Get linearized dynamics matrices A, B and constant term"""
        # Gravity term
        g = 9.81  # gravity
        m = self._body_mass
        J = self._body_inertia
        J_g = np.block([
            [np.diag([m, m, m]),np.zeros((3, 3))],
            [np.zeros((3, 3)),J] 
        ])  
        
        # Q = Exp_SO3(x_c[3:6])  # Q = Td_c[:3, :3] 
        
        # Fg_c = Q.T @ np.array([0, 0, -m * g])
        # Gravity_term = 1 / m * np.concatenate([np.zeros(6), [Fg_c[0], Fg_c[1], Fg_c[2], 0, 0, 0]])
        
        # # Gravity linearized term in matrix A
        # mg = hat(Fg_c)
        # Mg = 1 / m * np.block([
        #     [np.zeros((3, 3)), mg],
        #     [np.zeros((3, 6))]
        # ])
        
        # Alternative gravity calculation
        # Fg = np.array([0, 0, -m * g])
        # Gravity_term = np.concatenate([np.zeros(6), la.inv(J_g) @ np.concatenate([Fg, [0, 0, 0]])])
        # Mg = la.inv(J_g) @ np.block([
        #     [np.zeros((3, 3)), hat(Fg)],
        #     [np.zeros((3, 6))]
        # ])
        
        # if dyn_model.model_type == DynModels.DynSE3:
        #     Gravity_term = Gravity_term * 0
        #     Mg = Mg * 0
        #     Bt = np.vstack([np.zeros((6, 6)), la.inv(J_g)])
        # elif dyn_model.model_type == DynModels.DynQuad:
        #     Bt = np.vstack([np.zeros((6, 4)), la.inv(J_g) @ C])
        
        Gravity_term = np.concatenate([
                np.zeros(9),
                self._R.T @ np.array([0, 0, -g])
        ]) 

        rb1 = self._foot_positions_base_frame[0:3]*self._foot_contact_state[0]
        rb2 = self._foot_positions_base_frame[3:6]*self._foot_contact_state[1]
        rb3 = self._foot_positions_base_frame[6:9]*self._foot_contact_state[2]
        rb4 = self._foot_positions_base_frame[9:12]*self._foot_contact_state[3]
        J_inv = la.inv(J)  
        m_inv = (1/m)*np.identity(3)      
        Bt = np.block([
            [np.zeros((6, 12))],
            [J_inv@hat(rb1), J_inv@hat(rb2),J_inv@hat(rb3), J_inv@hat(rb4)],
            [m_inv,m_inv,m_inv,m_inv]
        ])
        
        # From paper [1], Body velocity = xi dynamics linearization 
        xi_c = x_c[6:12]
        v_c = xi_c[:3]
        w_c = xi_c[3:6]
        M = np.block([
            [np.zeros((3, 3)), hat(m * v_c)],
            [hat(m * v_c), hat(J @ w_c)]
        ])
        bt = -la.inv(J_g) @ M @ xi_c
        H = la.inv(J_g) @ AdSE3_v_dual(xi_c) @ J_g + la.inv(J_g) @ M
        
        h = self._PLANNING_TIMESTEP
        
        if not error_state_MPC:
            # x=[zeta;xi] zeta=[v;w]
            zetac = x_c[:6]
            At = np.block([
                [np.zeros((6, 6)), np.eye(6)],
                [np.zeros((6, 6)), H] # [Mg, H]
            ])
            
            # Term_t
            Term_t = Gravity_term + np.concatenate([np.zeros(6), bt])
            
            # Forward Euler integration with zero-order hold on u with time step h
            A_mat = np.eye(12) + At * h
            B_mat = Bt * h
            Term = Term_t * h
            
            # Add zeta dynamics
            A_zeta = np.block([
                [np.eye(6), Jac_r_inv_SE3(zetac) * h],
                [A_mat[6:12, :12]]
            ])
            A_mat = A_zeta
        else:
            # x=[zetaE;xi] zeta=[v;w]
            xi_d = x_d[6:12]
            At = np.block([
                [-AdSE3_v(xi_d), np.eye(6)],
                [np.zeros((6, 6)), H] # [Mg, H]
            ])
            At = np.array(At)
            
            # Term_t
            Term_t = Gravity_term + np.concatenate([-xi_d, bt])
            
            # Forward Euler integration with zero-order hold on u with time step h
            A_mat = np.eye(12) + At * h
            B_mat = Bt * h
            Term = Term_t * h
        
        return A_mat, B_mat, Term


    def convex_mpc(self,A,B,Term, Q, R,
               X_ref_window, xic,
               N_mpc, costN, error_state_MPC):
        """Convex Model Predictive Control solver"""
        # Get sizes for state and control
        nx, nu = B.shape
        
        # Check sizes
        assert A.shape == (nx, nx)
        assert len(xic) == 13
        assert X_ref_window.shape[1] == N_mpc
        
        if not error_state_MPC:
            xic_ = sim2control_state(xic)
        else:
            xic_ = getX_errorMPC(xic, X_ref_window[:, 0])
        
        # Variables we are solving for
        X = cvx.Variable((nx, N_mpc))
        U = cvx.Variable((nu, N_mpc - 1))
        
        # Cost function
        cost = 0
        xi_sum = np.zeros(6)
        
        for k in range(N_mpc - 1):
            xi = X[6:12, k]
            xi_sum = xi_sum + xi
            zeta_delta, xi_delta = error_state_lie(X[:, k], X_ref_window[:, k], xic, costN, error_state_MPC)
            x_k = cvx.hstack([zeta_delta, xi_delta])
            u_k = U[:, k]
            
            # Add stagewise cost
            cost += 0.5 * cvx.quad_form(x_k, Q) + 0.5 * cvx.quad_form(u_k, R)
        
        # Add terminal cost
        zeta_delta, xi_delta  = error_state_lie(X[:, N_mpc - 1], X_ref_window[:, N_mpc - 1], xic, costN, error_state_MPC)
        x_n = cvx.hstack([zeta_delta, xi_delta])
        cost += 0.5 * cvx.quad_form(x_n, 10*Q)
        
        # Constraints
        constraints = []
        
        # Initial condition
        constraints.append(X[:, 0] == xic_)
        
        # Dynamics constraints
        for k in range(N_mpc - 1):
            constraints.append(X[:, k + 1] == A @ X[:, k] + B @ U[:, k] + Term)
        
        mu = self._foot_friction_coeffs
        # Input constraints
        for k in range(N_mpc - 1):
            for j in range(4): # j'th leg
                fj_w = self._R @ U[3*j:3*j+3, k]
                # x axis
                constraints.append(fj_w[0] <= mu[j]*fj_w[2])
                constraints.append(fj_w[0] >= -mu[j]*fj_w[2])
                # y axis
                constraints.append(fj_w[1] <= mu[j]*fj_w[2])
                constraints.append(fj_w[1] >= -mu[j]*fj_w[2])
                # z axis
                constraints.append(fj_w[2] >= 0)
        
        # Solve the problem
        prob = cvx.Problem(cvx.Minimize(cost), constraints)
        prob.solve()
        
        if prob.status != cvx.OPTIMAL:
            raise RuntimeError(f"Solver failed with status: {prob.status}")
        
        # Return the first control input
        return -U[:, 0].value
            
    def _rpy_to_rotation_matrix(self, rpy):
        """Convert roll, pitch, yaw to rotation matrix."""
        roll, pitch, yaw = rpy
        
        # ZYX rotation (common in robotics)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        return Rz @ Ry @ Rx
    