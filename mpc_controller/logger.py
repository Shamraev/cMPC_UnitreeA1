"""Logger module."""

import numpy as np
import pandas as pd 
import os
import inspect
from datetime import datetime
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

class Logger:
  def __init__(self, robot, print_COT, logdir):
    self._com_posns,self._com_vels,self._imu_rates,self._actions,self._contact_forces = [],[],[],[],[]
    self._legs_states,self._des_com_posns,self._des_com_vels,self._COT,self._t = [],[],[],[],[]
    self._E_i,self._motor_temp,self._gait_name = [],[],[]
    self._E = 0
    self._COT_started = False
    self._print_COT = print_COT
    self._robot = robot
    self._start_time = self._robot.GetTimeSinceReset()
    self._start_x = 0

    logdir = os.path.join(logdir,
                          datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))
    os.makedirs(logdir)
    self._logdir = logdir

  def __del__(self):
    self.save_to(self._logdir)
    print("logged to: {}".format(self._logdir))

  def get_COT(self): 
      COT = 0
      current_time = self._robot.GetTimeSinceReset()
      if current_time - self._start_time>3:
        if not self._COT_started: self._start_x = self._robot.GetBasePosition()[0]; self._COT_started = True
        self._E = self._E + self._robot.GetEnergyConsumptionPerControlStep()
        x = self._robot.GetBasePosition()[0] - self._start_x
        if x==0:
          x=0.001
        COT = self._E/(self._robot.MPC_BODY_MASS*9.8*x)
        if self._print_COT:
          print("COT=",np.round(COT,2))
          print("v=",self._robot.GetBaseVelocity()[0])

      return COT

  def log(self, hybrid_action, contact_force, des_com_pos, des_com_vel, gait_name):
    self._motor_temp.append(np.array(self._robot.GetMotorTemperature()))
    self._com_posns.append(np.array(self._robot.GetBasePosition()))
    self._com_vels.append(np.array(self._robot.GetBaseVelocity()).copy())
    self._imu_rates.append(np.array(self._robot.GetBaseRollPitchYawRate()).copy())
    self._actions.append(hybrid_action)
    self._contact_forces.append(contact_force)
    self._legs_states.append(self._robot.GetFootContacts()) 
    self._des_com_posns.append(des_com_pos)
    self._des_com_vels.append(des_com_vel)
    self._E_i.append(self._robot.GetEnergyConsumptionPerControlStep())
    self._COT.append(self.get_COT())
    #real_torques.append(elf._robot.GetMotorTorques())
    self._gait_name.append(gait_name)
    self._t.append(self._robot.GetTimeSinceReset())

  def save_to(self, logdir):
    np.savez(os.path.join(logdir, 'states.npz'),
            t=self._t,
            com_posns=self._com_posns,
            com_vels=self._com_vels,
            des_com_posns=self._des_com_posns,
            des_com_vels=self._des_com_vels,
            legs_states=self._legs_states, 
            contact_forces=self._contact_forces, # computed forces from MPC
            E_i=self._E_i,
            COT=self._COT,

            actions=self._actions,                # computed torques (desired torques)
            #real_torques.append(elf._robot.GetMotorTorques()),
            motor_temp=self._motor_temp,
            imu_rates=self._imu_rates,
            gait_name=self._gait_name           
            )



    ## from Convex_mpc_untreeA1 repo. logging with pickle.
    # TODO: rename gait name to gait type. remove csv, npz saving 
    # (csv for plotjuggler, but do I need it? from pkl file I can make csv for Plotjuggler)
    


  # def __init__(self,
  #              use_real_robot: bool = False,
  #              show_gui: bool = False,
  #              logdir: str = 'logs/'):

  # self._logs = []
  # self._logdir = logdir

  # def _start_logging(self):
  #   self._logs = []

  #     def _update_logging(self, action, qp_sol):
  #   frame = dict(
  #       desired_speed=(self._swing_controller.desired_speed,
  #                      self._swing_controller.desired_twisting_speed),
  #       timestamp=self._time_since_reset,
  #       base_rpy=self._robot.base_orientation_rpy,
  #       motor_angles=self._robot.motor_angles,
  #       base_vel=self._robot.motor_velocities,
  #       base_vels_body_frame=self._state_estimator.com_velocity_body_frame,
  #       base_rpy_rate=self._robot.base_rpy_rate,
  #       motor_vels=self._robot.motor_velocities,
  #       motor_torques=self._robot.motor_torques,
  #       contacts=self._robot.foot_contacts,
  #       desired_grf=qp_sol,
  #       robot_action=action,
  #       gait_generator_phase=self._gait_generator.current_phase.copy(),
  #       gait_generator_state=self._gait_generator.leg_state,
  #       ground_orientation=self._state_estimator.
  #       ground_orientation_world_frame,
  #   )
  #   self._logs.append(frame)

  # def _flush_logging(self):
  #   if not os.path.exists(self._logdir):
  #     os.makedirs(self._logdir)
  #   filename = 'log_{}.pkl'.format(
  #       datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))
  #   pickle.dump(self._logs, open(os.path.join(self._logdir, filename), 'wb'))
  #   logging.info("Data logged to: {}".format(
  #       os.path.join(self._logdir, filename)))