import matplotlib.pyplot as plt
#from absl import flags
import pandas
import addcopyfighandler
import numpy as np
import pickle
import os
import shutil
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

Tstart = 5.05
Tend = 6

## clear and make directory for plots saving
fig_dir = 'fig'
try:
    shutil.rmtree(fig_dir) 
except:
    ok = 'ok'
os.makedirs(fig_dir, exist_ok=True)

## read from npz

data = np.load('states.npz', allow_pickle=True)

i1=np.where(data['t']==Tstart)[0][0]
i2=np.where(data['t']==Tend)[0][0]

t = data['t'][i1:i2]
actions = data['actions'][i1:i2]
E_i = data['E_i'][i1:i2]
gait_name = data['gait_name'][i1:i2]
imu_rates = data['imu_rates'][i1:i2]
motor_temp = data['motor_temp'][i1:i2]
com_vels = data['com_vels'][i1:i2]
com_posns = data['com_posns'][i1:i2]
legs_states = data['legs_states'][i1:i2]
contactFR = np.array([row[0] for row in legs_states])
des_com_posns = data['des_com_posns'][i1:i2]
des_com_vels = data['des_com_vels'][i1:i2]
contact_forces = data['contact_forces'][i1:i2]

## plot force for front right leg
f=[row[0] for row in contact_forces]
fx = [row[0] for row in f]; fy = [row[1] for row in f]; fz = [row[2] for row in f] 
plt.plot(t, fx, linestyle='--', label='Fx') 
plt.plot(t, fy, linestyle='-.', label='Fy') 
plt.plot(t, fz, linestyle='-', label='Fz') 


plt.xlabel('Time (sec)'); plt.ylabel('Force (N)')
plt.title('Computed force for FR leg vs Time')
plt.fill_between(x=t, y1=plt.ylim()[0],y2=plt.ylim()[1], where=contactFR>0,color='green', alpha=0.2)
plt.legend()
plt.grid()
plt.savefig(fig_dir+'/forceZ.eps', dpi=400)
plt.show()

# f=[row[0] for row in contact_forces]
# fz=[row[1] for row in f]
# plt.plot(t, fz, label = "contact_force_z"); plt.xlabel('Time (sec)'); plt.ylabel('Force (N)')
# plt.title('Computed force for FR leg vs Time')
# #plt.legend()
# plt.grid()
# plt.savefig(fig_dir+'/forceZ.eps', dpi=400)
# plt.show()

## plot CoM z, z_dot 
des_com_vel_z = [row[2] for row in des_com_vels]
com_vel_z=[row[2] for row in com_vels]
plt.plot(t, com_vel_z, label = "com_vel_z")
plt.plot(t, des_com_vel_z, label = "des_com_vel_z"); plt.xlabel('Time (sec)'); plt.ylabel('CoM z velocity (m/s)')
plt.title('CoM z velocity vs Time')
plt.fill_between(x=t, y1=plt.ylim()[0],y2=plt.ylim()[1], where=contactFR>0,color='green', alpha=0.2)
plt.legend()
plt.grid()
plt.show()

## plot CoM x_dot,  des_x_dot
des_com_vel_x = [row[0] for row in des_com_vels]
com_vel_x=[row[0] for row in com_vels]
plt.plot(t, com_vel_x, label = "com_vel_x")
plt.plot(t, des_com_vel_x, label = "des_com_vel_x"); plt.xlabel('Time (sec)'); plt.ylabel('CoM x velocity (m/s)')
plt.title('CoM x velocity vs Time')
plt.fill_between(x=t, y1=plt.ylim()[0],y2=plt.ylim()[1], where=contactFR>0,color='green', alpha=0.2)
plt.legend()
plt.grid()
plt.show()

plt.plot(t, E_i); plt.xlabel('time (sec)'); plt.ylabel('Energy in each step (J)')
plt.title('Energy vs time')
plt.fill_between(x=t, y1=plt.ylim()[0],y2=plt.ylim()[1], where=contactFR>0,color='green', alpha=0.2)
#plt.legend()
plt.grid()
plt.show()

# plt.plot(t, actions[:,0]); plt.xlabel('time (sec)'); plt.ylabel('commanded torques (Nm)')
# plt.title('commanded torques vs time')
# #plt.legend()
# plt.grid()
# plt.show()

# lineObjects=plt.plot(t, imu_rates); plt.xlabel('time (sec)'); plt.ylabel('Imu rates (rad/s)')
# plt.title('Imu rates vs time')
# plt.legend(lineObjects, ('roll rate', 'pitch rate','yaw rate'))
# plt.grid()
# plt.show() 

# lineObjects=plt.plot(t, motor_temp[:,0:3]); plt.xlabel('time (sec)'); plt.ylabel('Motor temperature of FR leg (rad/s)')
# plt.title('Motor temperature vs time')
# plt.legend(lineObjects, ('tem1', 'tem2','tem3'))
# plt.grid()
# plt.show()

###########################################

# ## from pickle
# file = open('crawl_suscess_walk.pkl', 'rb')
# # dump information to that file
# data = pickle.load(file)
# file.close()

# t=[d['timestamp'] for d in data if 'timestamp' in d]
# base_rpy=[d['base_rpy'] for d in data if 'base_rpy' in d]
# motor_angles=[d['motor_angles'] for d in data if 'motor_angles' in d]
# base_vel=[d['base_vel'] for d in data if 'base_vel' in d]
# base_vels_body_frame=[d['base_vels_body_frame'] for d in data if 'base_vels_body_frame' in d]
# base_rpy_rate=[d['base_rpy_rate'] for d in data if 'base_rpy_rate' in d]
# motor_vels=[d['motor_vels'] for d in data if 'motor_vels' in d]
# motor_torques=[d['motor_torques'] for d in data if 'motor_torques' in d]
# contacts=[d['contacts'] for d in data if 'contacts' in d]
# desired_grf_d=[d['desired_grf'] for d in data if 'desired_grf' in d]
# desired_grf=[d['qp_sol'] for d in desired_grf_d if 'qp_sol' in d]
# f0 =[row[0] for row in desired_grf]
# fz=[row[0] for row in f0]
# robot_action=[d['robot_action'] for d in data if 'robot_action' in d]
# desired_speed=[d['desired_speed'] for d in data if 'desired_speed' in d]


# ## plot force
# lineObjects=plt.plot(t, fz); plt.xlabel('Time (sec)'); plt.ylabel('Force (N)')
# plt.title('Force vs Time')
# #plt.legend(lineObjects, ('Fx','Fy','Fz'))
# plt.axis([0, 5, -20, 20])
# plt.grid()
# plt.show()

# ## plot CoM z, z_dot 
# des_com_vel = [row[0] for row in desired_speed]
# des_com_vel_z = [row[2] for row in des_com_vel]
# com_vel_z=[row[2] for row in base_vels_body_frame]
# plt.plot(t, com_vel_z, label = "com_vel_z")
# plt.plot(t, des_com_vel_z, label = "des_com_vel_z"); plt.xlabel('Time (sec)'); plt.ylabel('CoM z velocity (m/s)')
# plt.title('CoM z velocity vs Time')
# plt.axis([0, 5, -0.1, 0.1])
# plt.legend()
# plt.grid()
# plt.show()

# ## plot CoM x_dot,  des_x_dot
# des_com_vel = [row[0] for row in desired_speed]
# des_com_vel_x = [row[0] for row in des_com_vel]
# com_vel_x=[row[0] for row in base_vels_body_frame]
# plt.plot(t, com_vel_x, label = "com_vel_x")
# plt.plot(t, des_com_vel_x, label = "des_com_vel_x"); plt.xlabel('Time (sec)'); plt.ylabel('CoM x velocity (m/s)')
# plt.title('CoM x velocity vs Time')
# #plt.axis([5, 10, -1, 1])
# plt.legend()
# plt.grid()
# plt.show()


# lineObjects=plt.plot(t, base_rpy_rate); plt.xlabel('time (sec)'); plt.ylabel('Imu rates (rad/s)')
# plt.title('Imu rates vs time')
# plt.legend(lineObjects, ('roll rate', 'pitch rate','yaw rate'))
# plt.axis([5, 10, -4, 3])
# plt.grid()
# plt.show()