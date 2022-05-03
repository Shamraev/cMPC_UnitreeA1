# Lint as: python3
"""A torque based stance controller framework."""

from __future__ import absolute_import
from __future__ import division
#from __future__ import google_type_annotations
from __future__ import print_function

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from typing import Any, Sequence, Tuple

import numpy as np
from numpy import cos,cosh,sin,sinh
import pybullet as p  # pytype: disable=import-error

try:
  from mpc_controller import gait_generator as gait_generator_lib
  from mpc_controller import leg_controller
except:  #pylint: disable=W0702
  print("You need to install motion_imitation")
  print("Either run python3 setup.py install --user in this repo")
  print("or use pip3 install motion_imitation --user")
  sys.exit()

try:
  import mpc_osqp as convex_mpc  # pytype: disable=import-error
except:  #pylint: disable=W0702
  print("You need to install motion_imitation")
  print("Either run python3 setup.py install --user in this repo")
  print("or use pip3 install motion_imitation --user")
  sys.exit()

_FORCE_DIMENSION = 3
# The QP weights in the convex MPC formulation. See the MIT paper for details:
#   https://ieeexplore.ieee.org/document/8594448/
# Intuitively, this is the weights of each state dimension when tracking a
# desired CoM trajectory. The full CoM state is represented by
# (roll_pitch_yaw, position, angular_velocity, velocity, gravity_place_holder).
# _MPC_WEIGHTS = (5, 5, 0.2, 0, 0, 10, 0.5, 0.5, 0.2, 0.2, 0.2, 0.1, 0)
# This worked well for in-place stepping in the real robot.
# _MPC_WEIGHTS = (5, 5, 0.2, 0, 0, 10, 0., 0., 0.2, 1., 1., 0., 0)
_MPC_WEIGHTS = (5, 5, 0.2, 0, 0, 10, 0., 0., 1., 1., 1., 0., 0)
_PLANNING_HORIZON_STEPS = 10
_PLANNING_TIMESTEP = 0.025

class SLIP_generator():
  def __init__(self, k, m, duration, time_step):
    self._g = 9.81
    self._k = k
    self._m = m
    self._des_z0 = 0.26
    self._wx = np.sqrt(self._g/(self._des_z0*1.2))
    self._wz = np.sqrt(self._k/self._m)
    self._Ts = time_step
    self._duration = duration

    self._z0 = 0
    self._z_dot0 = 0
    self._x0 = 0
    self._x_dot0 = 0

    self.state_z = np.array([[self._z0], [self._z_dot0]])
    self.state_x = np.array([[self._x0], [self._x_dot0]])
    self.started_cycle = False

    w = self._wz
    Ts = self._Ts
    self.A_z = np.array([ [cos(w*Ts), sin(w*Ts)/w], 
                        [-w*sin(w*Ts), cos(w*Ts)]])
    self.B_z = np.array([[1-cos(w*Ts)], [w*sin(w*Ts)]])

  def Reset(self, z0, z_dot0, x0, x_dot0):
    self._z0 = z0
    self._z_dot0 = z_dot0
    self._x0 = x0
    self._x_dot0 = x_dot0

    self.state_z = np.array([[self._z0], [self._z_dot0]])
    self.state_x = np.array([[self._x0], [self._x_dot0]])
    wz = self._wz
    wx = self._wx
    Ts = self._Ts
    self.A_z = np.array([ [cos(wz*Ts), sin(wz*Ts)/wz], 
                        [-wz*sin(wz*Ts), cos(wz*Ts)]])
    self.B_z = np.array([[1-cos(wz*Ts)], [wz*sin(wz*Ts)]])

    self.A_x = np.array([ [cosh(wx*Ts), sinh(wx*Ts)/wx], 
                        [wx*sinh(wx*Ts), cosh(wx*Ts)]])

  def get_traj_z(self):
    u = self._des_z0 -self._g/(self._wz**2) # 0.23 0.25
    self.state_z = self.A_z@self.state_z + self.B_z*u

    return self.state_z

  def get_traj_x(self):
    self.state_x = self.A_x@self.state_x

    return self.state_x


  def get_traj(self):
    z = self.get_traj_z()
    x = self.get_traj_x()
    return z, x

  

class TorqueStanceLegController(leg_controller.LegController):
  """A torque based stance leg controller framework.

  Takes in high level parameters like walking speed and turning speed, and
  generates necessary the torques for stance legs.
  """
  def __init__(
      self,
      robot: Any,
      gait_generator: Any,
      state_estimator: Any,
      desired_speed: Tuple[float, float] = (0, 0),
      desired_twisting_speed: float = 0,
      desired_body_height: float = 0.45,
      body_mass: float = 220 / 9.8,
      body_inertia: Tuple[float, float, float, float, float, float, float,
                          float, float] = (0.07335, 0, 0, 0, 0.25068, 0, 0, 0,
                                           0.25447),
      num_legs: int = 4,
      friction_coeffs: Sequence[float] = (0.45, 0.45, 0.45, 0.45),
      qp_solver = convex_mpc.QPOASES,
      PLANNING_HORIZON_STEPS = 10,
      PLANNING_TIMESTEP = 0.025
  ):
    """Initializes the class.

    Tracks the desired position/velocity of the robot by computing proper joint
    torques using MPC module.

    Args:
      robot: A robot instance.
      gait_generator: Used to query the locomotion phase and leg states.
      state_estimator: Estimate the robot states (e.g. CoM velocity).
      desired_speed: desired CoM speed in x-y plane.
      desired_twisting_speed: desired CoM rotating speed in z direction.
      desired_body_height: The standing height of the robot.
      body_mass: The total mass of the robot.
      body_inertia: The inertia matrix in the body principle frame. We assume
        the body principle coordinate frame has x-forward and z-up.
      num_legs: The number of legs used for force planning.
      friction_coeffs: The friction coeffs on the contact surfaces.
    """
    self._robot = robot
    self._gait_generator = gait_generator
    self._state_estimator = state_estimator
    self.desired_speed = desired_speed
    self.desired_twisting_speed = desired_twisting_speed

    self._desired_body_height = desired_body_height
    self._body_mass = body_mass
    self._num_legs = num_legs
    self._friction_coeffs = np.array(friction_coeffs)
    body_inertia_list = list(body_inertia)
    weights_list = list(_MPC_WEIGHTS)
    self._cpp_mpc = convex_mpc.ConvexMpc(
        body_mass,
        body_inertia_list,
        self._num_legs,
        _PLANNING_HORIZON_STEPS,
        _PLANNING_TIMESTEP,
        weights_list,
        1e-5,
        qp_solver
        
    )
    self._SLIP = SLIP_generator(4000, body_mass, self._gait_generator.stance_duration[0], 0.002) # ?? # 
    self.last_foot_contact_states = np.array(
        [True, True, True, True],
        dtype=np.int32)
    self.start_cotntact_t = self._robot.GetTimeSinceReset()
    self.duration_of_contact = 0.2
    self.last_t = self._robot.GetTimeSinceReset()

  def reset(self, current_time):
    del current_time

  def update(self, current_time):
    del current_time

  def _estimate_robot_height(self, contacts):
    if np.sum(contacts) == 0:
      # All foot in air, no way to estimate
      return self._desired_body_height
    else:
      base_orientation = self._robot.GetBaseOrientation()
      rot_mat = self._robot.pybullet_client.getMatrixFromQuaternion(
          base_orientation)
      rot_mat = np.array(rot_mat).reshape((3, 3))

      foot_positions = self._robot.GetFootPositionsInBaseFrame()
      foot_positions_world_frame = (rot_mat.dot(foot_positions.T)).T
      # pylint: disable=unsubscriptable-object
      useful_heights = contacts * (-foot_positions_world_frame[:, 2])
      return np.sum(useful_heights) / np.sum(contacts)

  def get_action(self):
    """Computes the torque for stance legs."""
    dt = self._robot.GetTimeSinceReset()-self.last_t
    #print('dt: ', dt)
    desired_com_position = np.array((0., 0., self._desired_body_height),
                                    dtype=np.float64)
    desired_com_velocity = np.array(
        (self.desired_speed[0], self.desired_speed[1], 0.), dtype=np.float64)
    desired_com_roll_pitch_yaw = np.array((0., 0., 0.), dtype=np.float64)
    desired_com_angular_velocity = np.array(
        (0., 0., self.desired_twisting_speed), dtype=np.float64)
    foot_contact_state = np.array(
        [(leg_state in (gait_generator_lib.LegState.STANCE,
                        gait_generator_lib.LegState.EARLY_CONTACT))
         for leg_state in self._gait_generator.desired_leg_state],
        dtype=np.int32)

    # We use the body yaw aligned world frame for MPC computation.
    com_roll_pitch_yaw = np.array(self._robot.GetBaseRollPitchYaw(),
                                  dtype=np.float64)
    com_roll_pitch_yaw[2] = 0

    ## SLIP traj for COM
    # One spring only
    start_contact = self.last_foot_contact_states[0]!=foot_contact_state[0] and foot_contact_state[0]
    end_contact = self.last_foot_contact_states[0]!=foot_contact_state[0] and not foot_contact_state[0]

    contacts = np.array(
    [(leg_state in (gait_generator_lib.LegState.STANCE,
                    gait_generator_lib.LegState.EARLY_CONTACT))
      for leg_state in self._gait_generator.desired_leg_state],
    dtype=np.int32)

    x = 0
    z = self._estimate_robot_height(contacts)
    #z = self._robot.GetBasePosition()[2] # !! #
    z_dot, x_dot = self._state_estimator.com_velocity_body_frame[2], self.desired_speed[0]
    if start_contact:
      self._SLIP.Reset(z, z_dot, x, x_dot)
      self._SLIP.started_cycle = True
    if end_contact:
      self._SLIP.started_cycle = False

    #
    if self._SLIP.started_cycle:
      z, x = self._SLIP.get_traj()
      desired_com_position[2] = z[0]
      desired_com_velocity[2] = z[1]
      # desired_com_velocity[0] = x[1]
      
    ## SLIP traj for COM END

    #predicted_contact_forces=[0]*self._num_legs*_FORCE_DIMENSION
    # print("Com Vel: {}".format(self._state_estimator.com_velocity_body_frame))
    # print("Com RPY: {}".format(self._robot.GetBaseRollPitchYawRate()))
    # print("Com RPY Rate: {}".format(self._robot.GetBaseRollPitchYawRate()))
    p.submitProfileTiming("predicted_contact_forces")
    predicted_contact_forces = self._cpp_mpc.compute_contact_forces(
        [0],  #com_position //??//
        np.asarray(self._state_estimator.com_velocity_body_frame,
                   dtype=np.float64),  #com_velocity
        np.array(com_roll_pitch_yaw, dtype=np.float64),  #com_roll_pitch_yaw
        # Angular velocity in the yaw aligned world frame is actually different
        # from rpy rate. We use it here as a simple approximation.
        np.asarray(self._robot.GetBaseRollPitchYawRate(),
                   dtype=np.float64),  #com_angular_velocity
        foot_contact_state,  #foot_contact_states
        np.array(self._robot.GetFootPositionsInBaseFrame().flatten(),
                 dtype=np.float64),  #foot_positions_base_frame
        self._friction_coeffs,  #foot_friction_coeffs
        desired_com_position,  #desired_com_position
        desired_com_velocity,  #desired_com_velocity
        desired_com_roll_pitch_yaw,  #desired_com_roll_pitch_yaw
        desired_com_angular_velocity  #desired_com_angular_velocity
    )
    p.submitProfileTiming()
    # sol = np.array(predicted_contact_forces).reshape((-1, 12))
    # x_dim = np.array([0, 3, 6, 9])
    # y_dim = x_dim + 1
    # z_dim = y_dim + 1
    # print("Y_forces: {}".format(sol[:, y_dim]))

    contact_forces = {}
    for i in range(self._num_legs):
      contact_forces[i] = np.array(
          predicted_contact_forces[i * _FORCE_DIMENSION:(i + 1) *
                                   _FORCE_DIMENSION])
    action = {}
    for leg_id, force in contact_forces.items():
      # While "Lose Contact" is useful in simulation, in real environment it's
      # susceptible to sensor noise. Disabling for now.
      # if self._gait_generator.leg_state[
      #     leg_id] == gait_generator_lib.LegState.LOSE_CONTACT:
      #   force = (0, 0, 0)
      motor_torques = self._robot.MapContactForceToJointTorques(leg_id, force)
      for joint_id, torque in motor_torques.items():
        action[joint_id] = (0, 0, 0, 0, torque)

    self.last_foot_contact_states = foot_contact_state
    self.last_t = self._robot.GetTimeSinceReset()
    return action, contact_forces, desired_com_position, desired_com_velocity
