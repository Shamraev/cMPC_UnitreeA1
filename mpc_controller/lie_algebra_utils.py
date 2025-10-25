import numpy as np
import scipy.linalg as la
from enum import Enum
from typing import Union, List, Tuple, Optional

# Formulas are from these papers:
# paper [1]: S. Teng, D. Chen, W. Clark, and M. Ghaffari, "An Error-State Model Predictive Control on Connected Matrix Lie Groups for Legged Robot Control," Jan. 22, 2023. arXiv: 2203.08728.
# paper [2]: J. Sola, J. Deray, and D. Atchuthan, "A micro Lie theory for state estimation in robotics," 2018. arXiv: 1812.01537.


# Hat operations
def hat(v):  # v in R^3
    """Convert 3D vector to skew-symmetric matrix"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def unhat(V):  # V is 3x3 skew-symmetric matrix
    """Convert skew-symmetric matrix to 3D vector"""
    return np.array([V[2, 1], V[0, 2], V[1, 0]])

def hat_6(v):  # v=[v,w], v in R^6
    """Convert 6D vector to 4x4 matrix representation"""
    v_3d = v[:3]
    w_3d = v[3:]
    top = np.hstack([hat(w_3d), v_3d.reshape(-1, 1)])
    bottom = np.zeros((1, 4))
    return np.vstack([top, bottom])

def unhat_6(v_m):
    """Convert 4x4 matrix to 6D vector"""
    w = unhat(v_m[:3, :3])
    v = v_m[:3, 3]
    return np.concatenate([v, w])

# Quaternion stuff
def L(q):
    """Left quaternion multiplication matrix"""
    s = q[0]
    v = q[1:4]
    top = np.hstack([[s], -v])
    bottom = np.hstack([v.reshape(-1, 1), s * np.eye(3) + hat(v)])
    return np.vstack([top, bottom])

# Global matrices
T = np.diag([1, -1, -1, -1])
H = np.vstack([np.zeros((1, 3)), np.eye(3)])

def qtoQ(q):
    """Convert quaternion to rotation matrix"""
    return H.T @ T @ L(q) @ T @ L(q) @ H

def G(q):
    """Quaternion to rotation matrix generator"""
    return L(q) @ H

def rptoq(ϕ):
    """Rodrigues parameters to quaternion"""
    return (1 / np.sqrt(1 + ϕ.T @ ϕ)) * np.array([1, ϕ[0], ϕ[1], ϕ[2]])

def qtorp(q):
    """Quaternion to Rodrigues parameters"""
    return q[1:4] / q[0]

def E(q):
    """Extended matrix for quaternion operations"""
    return la.block_diag(np.eye(3), G(q), np.eye(6))

# Lie Groups' stuff from paper [2]
def Exp_SO3(phi):
    """Exponential map for SO(3)"""
    θ = la.norm(phi)  # scalar
    if abs(θ) < 1.0e-4:
        Q = np.eye(3) + hat(phi)
    else:
        u = phi / θ
        Q = np.eye(3) + np.sin(θ) * hat(u) + (1 - np.cos(θ)) * hat(u) @ hat(u)
    return Q

def Log_SO3(Q):
    """Logarithm map for SO(3)"""
    cos_θ = (np.trace(Q) - 1) / 2
    if abs(cos_θ - 1) < 1.0e-3:  # around identity
        phi = unhat(Q - np.eye(3))
    else:
        θ = np.arccos(cos_θ)  # scalar
        phi = θ * unhat(Q - Q.T) / (2 * np.sin(θ))
    return phi

def phi2q(phi):  # axis-angle to quaternion
    """Convert axis-angle to quaternion"""
    θ = la.norm(phi)  # scalar
    if θ != 0:
        u = phi / θ
        q = np.array([np.cos(θ/2), u[0]*np.sin(θ/2), u[1]*np.sin(θ/2), u[2]*np.sin(θ/2)])
    else:
        q = np.array([1.0, 0.0, 0.0, 0.0])
    return q

def q2phi(q):  # quaternion to axis-angle
    """Convert quaternion to axis-angle representation"""
    Q = qtoQ(q)
    return Log_SO3(Q)

def Qtoq(Q):  # rotation matrix to quaternion
    """Convert rotation matrix to quaternion"""
    phi = Log_SO3(Q)
    return phi2q(phi)

def Ad_SO3(Q):
    """Adjoint representation for SO(3)"""
    return Q

def Jac_r(ϕ):  # SO3 right Jacobian
    """Right Jacobian for SO(3)"""
    θ = la.norm(ϕ)  # scalar
    if θ != 0:
        J = np.eye(3) - ((1 - np.cos(θ)) / (θ*θ)) * hat(ϕ) + ((θ - np.sin(θ)) / (θ*θ*θ)) * hat(ϕ) @ hat(ϕ)
    else:
        J = np.eye(3)
    return J

def Jac_r_inv(ϕ):  # SO3 inverse right Jacobian
    """Inverse right Jacobian for SO(3)"""
    θ = la.norm(ϕ)  # scalar
    if θ != 0:
        J = np.eye(3) + 0.5 * hat(ϕ) + (1/(θ*θ) - (1 + np.cos(θ)) / (2 * θ * np.sin(θ))) * hat(ϕ) @ hat(ϕ)
    else:
        J = np.eye(3)
    return J

def Jac_l_inv(ϕ):  # SO3 inverse left Jacobian
    """Inverse left Jacobian for SO(3)"""
    return Jac_r_inv(ϕ).T

def Jac_l(ϕ):
    """Left Jacobian for SO(3)"""
    return Jac_r(ϕ).T

def Jac_l_inv_SE3(zeta):
    """Inverse left Jacobian for SE(3)"""
    if la.norm(zeta[3:6]) < 1.0e-3:
        return np.eye(6)
        
    v = zeta[:3]
    w = zeta[3:6]
    θ = la.norm(w)
    Jw = Jac_l_inv(w)
    wh = hat(w)
    vh = hat(v)
    
    # Complex SE(3) Jacobian calculation
    Q = 0.5 * hat(v) + ((θ - np.sin(θ)) / (θ**3)) * (wh @ vh + vh @ wh + wh @ vh @ wh)
    Q -= ((1 - θ*θ/2 - np.cos(θ)) / (θ**4)) * (wh @ wh @ vh + vh @ wh @ wh - 3 * wh @ vh @ wh)
    
    b = ((1 - (θ**2)/2 - np.cos(θ)) / (θ**4)) - 3 * (θ - np.sin(θ) - (θ**3)/6) / (θ**5)
    Q -= 0.5 * b * (wh @ vh @ wh @ wh + wh @ wh @ vh @ wh)
    
    J = np.block([
        [Jw, -Jw @ Q @ Jw],
        [np.zeros((3, 3)), Jw]
    ])
    return J

def Jac_l_SE3(zeta):
    """Left Jacobian for SE(3)"""
    if la.norm(zeta[3:6]) < 1.0e-3:
        return np.eye(6)
        
    v = zeta[:3]
    w = zeta[3:6]
    θ = la.norm(w)
    Jw = Jac_l(w)
    wh = hat(w)
    vh = hat(v)
    
    # Complex SE(3) Jacobian calculation
    Q = 0.5 * hat(v) + ((θ - np.sin(θ)) / (θ**3)) * (wh @ vh + vh @ wh + wh @ vh @ wh)
    Q -= ((1 - θ*θ/2 - np.cos(θ)) / (θ**4)) * (wh @ wh @ vh + vh @ wh @ wh - 3 * wh @ vh @ wh)
    
    b = ((1 - (θ**2)/2 - np.cos(θ)) / (θ**4)) - 3 * (θ - np.sin(θ) - (θ**3)/6) / (θ**5)
    Q -= 0.5 * b * (wh @ vh @ wh @ wh + wh @ wh @ vh @ wh)
    
    J = np.block([
        [Jw, Q],
        [np.zeros((3, 3)), Jw]
    ])
    return J

def Jac_r_inv_SE3(zeta):
    """Inverse right Jacobian for SE(3)"""
    return Jac_l_inv_SE3(-zeta)

def Jac_r_SE3(zeta):
    """Right Jacobian for SE(3)"""
    return Jac_l_SE3(-zeta)

def Exp_SE3(zeta):
    """Exponential map for SE(3)"""
    v = zeta[:3]
    w = zeta[3:6]
    R = Exp_SO3(w)
    V_ = Jac_l(w)
    t = V_ @ v  # eq. 172 in paper [2], zeta=[v,w]
    return np.block([
        [R, t.reshape(-1, 1)],
        [np.zeros(3), 1]
    ])

def Log_SE3(T):
    """Logarithm map for SE(3)"""
    R = T[:3, :3]
    t = T[:3, 3]
    w = Log_SO3(R)
    v = Jac_l_inv(w) @ t  # eq 173 in paper [2], zeta=[v,w]
    zeta = np.concatenate([v, w])
    return zeta

def AdSE3(T):
    """Adjoint representation for SE(3)"""
    R = T[:3, :3]
    t = T[:3, 3]
    Ad = np.block([
        [R, hat(t) @ R],
        [np.zeros((3, 3)), R]
    ])
    return Ad

def AdSE3_dual(zeta):
    """Dual adjoint representation for SE(3)"""
    return AdSE3(zeta).T

def AdSE3_v(V):  # lie algebra adjoint
    """Lie algebra adjoint for SE(3)"""
    v = hat(V[:3])
    w = hat(V[3:6])
    return np.block([
        [w, v],
        [np.zeros((3, 3)), w]
    ])

def AdSE3_v_dual(V):
    """Dual lie algebra adjoint for SE(3)"""
    return AdSE3_v(V).T

# Error-state dynamics # xE=[zetaE;V_b]; zetaE=[w;v]
def adjoint_(x):  # [w,v] # from the paper [1]
    """Adjoint operator for error-state dynamics"""
    w = x[:3]
    v = x[3:6]
    adx = np.block([
        [hat(w), np.zeros((3, 3))],
        [hat(v), hat(w)]
    ])
    return adx

# Rotation matrix stuff
def RotZ(theta):
    """Rotation matrix around Z-axis"""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

def RotX(theta):
    """Rotation matrix around X-axis"""
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])

def RotY(theta):
    """Rotation matrix around Y-axis"""
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

def Reorthogonalise_SO3(R):
    """Re-orthogonalize SO(3) matrix using SVD"""
    U, S, Vt = la.svd(R)
    R_new = U @ np.diag([1, 1, la.det(U @ Vt.T)]) @ Vt
    return R_new

def Reorthogonalise_SE3(T):
    """Re-orthogonalize SE(3) matrix"""
    R = T[:3, :3]
    r = T[:3, 3]
    R = Reorthogonalise_SO3(R)
    return np.block([
        [R, r.reshape(-1, 1)],
        [np.zeros(3), 1]
    ])