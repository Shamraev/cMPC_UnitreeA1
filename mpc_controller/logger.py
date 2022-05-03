"""Logger module."""

import numpy as np
import pandas as pd 
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

class Logger:
  def __init__(self, robot, print_COT):
    self._com_posns,self._com_vels,self._imu_rates,self._actions,self._contact_forces = [],[],[],[],[]
    self._legs_states,self._des_com_posns,self._des_com_vels,self._COT,self._t = [],[],[],[],[]
    self._E = 0
    self._COT_started = False
    self._print_COT = print_COT
    self._robot = robot
    self._start_time = self._robot.GetTimeSinceReset()
    self._start_x = 0

  def get_COT(self): 
      COT = 0
      current_time = self._robot.GetTimeSinceReset()
      if current_time - self._start_time>3:
        if not self._COT_started: self._start_x = self._robot.GetBasePosition()[0]; self._COT_started = True
        self._E = self._E + self._robot.GetEnergyConsumptionPerControlStep()
        x = self._robot.GetBasePosition()[0] - self._start_x
        COT = self._E/(self._robot.MPC_BODY_MASS*9.8*x)
        if self._print_COT:
          print("COT=",np.round(COT,2))
          print("v=",self._robot.GetBaseVelocity()[0])

      return COT

  def log(self, hybrid_action, contact_force, des_com_pos, des_com_vel):
    self._com_posns.append(np.array(self._robot.GetBasePosition()))
    self._com_vels.append(np.array(self._robot.GetBaseVelocity()).copy())
    self._imu_rates.append(np.array(self._robot.GetBaseRollPitchYawRate()).copy())
    self._actions.append(hybrid_action)
    self._contact_forces.append(contact_force)
    self._legs_states.append(self._robot.GetFootContacts()) 
    self._des_com_posns.append(des_com_pos)
    self._des_com_vels.append(des_com_vel)
    self._COT.append(self.get_COT())
    #real_torques.append(elf._robot.GetMotorTorques())
    self._t.append(self._robot.GetTimeSinceReset())

  def save_to(self, path):
    # np.savez(os.path.join(logdir, 'action.npz'),
    #          action=actions,
    #          com_vels=com_vels,
    #          imu_rates=imu_rates
    #          #real_torques=real_torques
    #          )

    d = {
        #'action': actions, 
        #'com_vels': com_vels,
        #'imu_rates': imu_rates,
        'contact_force_z': [row[0][0] for row in self._contact_forces], #contact_force[0][2],
        'legs_state': [int(row[0]) for row in self._legs_states], #robot.GetFootContacts()[0],
        'com_pos_z': [row[2] for row in self._com_posns], #robot.GetBasePosition()[2],
        'com_vel_z': [row[2] for row in self._com_vels], #robot.GetBaseVelocity()[2],
        'des_com_pos_z': [row[2] for row in self._des_com_posns], #float(desired_com_position[2]),
        'des_com_vel_z': [row[2] for row in self._des_com_vels],#float(desired_com_velocity[2]),
        'COT': self._COT,
        't': self._t}
    pd.DataFrame(data=d).to_csv(path)