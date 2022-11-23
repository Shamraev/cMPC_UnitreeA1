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

## clear and make directory for plots saving
fig_dir = 'fig'
try:
    shutil.rmtree(fig_dir) 
except:
    ok = 'ok'
os.makedirs(fig_dir, exist_ok=True)


def plot_main_res(data, Tstart, Tend):
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

    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, figsize=(8,7), sharex=True)

    ## plot force for front right leg
    f=[row[0] for row in contact_forces]
    fx = [row[0] for row in f]; fy = [row[1] for row in f]; fz = [row[2] for row in f] 
    print(min(fz)) #--
    ax1.plot(t, fx, linestyle='--', label='Fx') 
    ax1.plot(t, fy, linestyle='-.', label='Fy') 
    ax1.plot(t, fz, linestyle='-', label='Fz') 


    ax1.set_ylabel('Force (N)')
    ax1.set_title('Computed force for FR leg')
    ax1.fill_between(x=t, y1=ax1.get_ylim()[0],y2=ax1.get_ylim()[1], where=contactFR>0,color='gray', alpha=0.2, label='Ground contact')
    ax1.legend(bbox_to_anchor=(1,1), loc="upper left")
    ax1.grid()
    # ax1.savefig(fig_dir+'/forceZ.eps', dpi=400)
    # ax1.show()

    # f=[row[0] for row in contact_forces]
    # fz=[row[1] for row in f]
    # plt.plot(t, fz, label = "contact_force_z"); plt.xlabel('Time (sec)'); plt.ylabel('Force (N)')
    # plt.title('Computed force for FR leg vs Time')
    # #plt.legend()
    # plt.grid()
    # plt.savefig(fig_dir+'/forceZ.eps', dpi=400)
    # plt.show()

    ## plot CoM z
    des_com_pos_z = [row[2] for row in des_com_posns]
    com_pos_z=[row[2] for row in com_posns]
    ax2.plot(t, com_pos_z, label = "CoM z")
    ax2.plot(t, des_com_pos_z, label = "Des CoM z"); ax2.set_ylabel('CoM z (m)')
    ax2.set_title('CoM z')
    ax2.fill_between(x=t, y1=ax2.get_ylim()[0],y2=ax2.get_ylim()[1], where=contactFR>0,color='gray', alpha=0.2, label='Ground contact')
    ax2.legend(bbox_to_anchor=(1,1), loc="upper left")
    ax2.grid()

    ## plot CoM z_dot 
    des_com_vel_z = [row[2] for row in des_com_vels]
    com_vel_z=[row[2] for row in com_vels]
    ax3.plot(t, com_vel_z, label = "CoM z vel")
    ax3.plot(t, des_com_vel_z, label = "Des CoM z vel");  ax3.set_ylabel('CoM z velocity (m/s)')
    ax3.set_title('CoM z velocity')
    ax3.fill_between(x=t, y1=ax3.get_ylim()[0],y2=ax3.get_ylim()[1], where=contactFR>0,color='gray', alpha=0.2, label='Ground contact')
    ax3.legend(bbox_to_anchor=(1,1), loc="upper left")
    ax3.grid()

    ## plot CoM des_x_dot
    des_com_vel_x = [row[0] for row in des_com_vels]
    com_vel_x=[row[0] for row in com_vels]
    ax4.plot(t, com_vel_x, label = "CoM x vel")
    ax4.plot(t, des_com_vel_x, label = "Des CoM x vel"); ax4.set_xlabel('Time (sec)'); ax4.set_ylabel('CoM x velocity (m/s)')
    ax4.set_title('CoM x velocity')
    ax4.fill_between(x=t, y1=ax4.get_ylim()[0],y2=ax4.get_ylim()[1], where=contactFR>0,color='gray', alpha=0.2, label='Ground contact')
    ax4.legend(bbox_to_anchor=(1,1), loc="upper left")
    ax4.grid()

    ax4.set_xlabel('Time (sec)')

    fig.tight_layout()
    return fig

def plot_energy_comparision(data, dataSLIP, Tstart, Tend):
    i1=np.where(data['t']==Tstart)[0][0]
    i2=np.where(data['t']==Tend)[0][0]

    t = data['t'][i1:i2]
    E_i = data['E_i'][i1:i2]
    E_i_SLIP = dataSLIP['E_i'][i1:i2]
    legs_states = data['legs_states'][i1:i2]
    contactFR = np.array([row[0] for row in legs_states])

    fig, ax1 = plt.subplots(1)

    ax1.plot(t, E_i, label='dE Without SLIP')
    ax1.plot(t, E_i_SLIP, label='dE With SLIP', linestyle='--');  
    ax1.set_xlabel('Time (sec)'); ax1.set_ylabel('Energy in each step dt (J)')
    ax1.set_title('Energy')
    ax1.fill_between(x=t, y1=ax1.get_ylim()[0],y2=ax1.get_ylim()[1], where=contactFR>0,color='gray', alpha=0.2, label='Ground contact')
    ax1.legend(bbox_to_anchor=(1,1), loc="upper left")
    ax1.grid()
    ax1.set_xlabel('Time (sec)')

    fig.tight_layout()
    return fig

def plot_COT_comparision(data, dataSLIP, Tstart, Tend):
    i1=np.where(data['t']==Tstart)[0][0]
    i2=np.where(data['t']==Tend)[0][0]

    t = data['t'][i1:i2]
    COT = data['COT'][i1:i2]
    COT_SLIP = dataSLIP['COT'][i1:i2]
    legs_states = data['legs_states'][i1:i2]
    contactFR = np.array([row[0] for row in legs_states])
    des_com_vels = data['des_com_vels'][i1:i2] # mb use this 
    des_com_vel_x=[row[0] for row in des_com_vels]
    com_vels = data['com_vels'][i1:i2]
    com_vel_x=[row[0] for row in com_vels]

    fig, (ax1, ax2) = plt.subplots(2, sharex=True)

    ax1.plot(t, COT, label='COT Without SLIP')
    ax1.plot(t, COT_SLIP, label='COT With SLIP')  
    ax1.set_ylabel('COT')
    ax1.fill_between(x=t, y1=ax1.get_ylim()[0],y2=ax1.get_ylim()[1], where=contactFR>0,color='gray', alpha=0.2, label='Ground contact')
    ax1.legend(bbox_to_anchor=(1,1), loc="upper left")
    ax1.set_title('COT with and without SLIP with changing desired x velosity')
    ax1.grid()

    ax2.plot(t, com_vel_x, label='CoM x velocity')
    ax2.plot(t, des_com_vel_x, label='CoM des x velocity')
    ax2.set_xlabel('Time (sec)')  
    ax2.set_ylabel('X velocity (m/s)')

    ax2.fill_between(x=t, y1=ax2.get_ylim()[0],y2=ax2.get_ylim()[1], where=contactFR>0,color='gray', alpha=0.2, label='Ground contact')
    ax2.legend(bbox_to_anchor=(1,1), loc="upper left")
    ax2.grid()

    fig.tight_layout()
    return fig

def foot_pos_comparision(data, dataSLIP, Tstart, Tend):
    i1=np.where(data['t']==Tstart)[0][0]
    i2=np.where(data['t']==Tend)[0][0]

    t = data['t'][i1:i2]
    legs_states = data['legs_states'][i1:i2]
    contactFR = np.array([row[0] for row in legs_states])
    footPositionsInBaseFrame = data['footPositionsInBaseFrame'][i1:i2]
    footPosFR = np.array([row[0] for row in footPositionsInBaseFrame])
    footPositionsInBaseFrame_SLIP = dataSLIP['footPositionsInBaseFrame'][i1:i2]
    footPosFR_SLIP = np.array([row[0] for row in footPositionsInBaseFrame_SLIP])


    fig, ax1 = plt.subplots(1)

    ax1.plot(t, footPosFR)
    ax1.plot(t, footPosFR_SLIP, linestyle='--')
    ax1.set_xlabel('Time (sec)'); ax1.set_ylabel('Foot position (m)')
    ax1.set_title('Foot position of FR leg in body frame')
    ax1.legend(['Foot pos x', 'Foot pos y', 'Foot pos z','Foot pos x with SLIP', 'Foot pos y with SLIP', 'Foot pos z with SLIP'], bbox_to_anchor=(1,1), loc="upper left")
    ax1.fill_between(x=t, y1=ax1.get_ylim()[0],y2=ax1.get_ylim()[1], where=contactFR>0,color='gray', alpha=0.2, label='Ground contact')

    ax1.grid()
    ax1.set_xlabel('Time (sec)')

    fig.tight_layout()
    return fig

plotting_main_res = True
plotting_COT_comparison_res = True
plotting_foot_pos_comparision = True

if plotting_main_res:
    Tstart = 5.11
    Tend = 5.91
    data = np.load('pronk_vel_1ms/states.npz', allow_pickle=True)
    fig = plot_main_res(data, Tstart, Tend)
    fig.savefig(fig_dir+'/results_without_SLIP.eps', dpi=500)
    dataSLIP = np.load('pronk_vel_1ms/statesSLIP.npz', allow_pickle=True)
    fig = plot_main_res(dataSLIP, Tstart, Tend)
    fig.savefig(fig_dir+'/results_with_SLIP.eps', dpi=500)

    fig = plot_energy_comparision(data, dataSLIP, Tstart, Tend)
    fig.savefig(fig_dir+'/energy_comparision.eps', dpi=500)
if plotting_COT_comparison_res:
    # plotting COT with speed changing
    Tstart = 4
    Tend = 12
    data = np.load('pronk_changing_vel/states_COT_SPEED_CHANGING.npz', allow_pickle=True)
    dataSLIP = np.load('pronk_changing_vel/states_COT_SPEED_CHANGING_SLIP.npz', allow_pickle=True)
    fig = plot_COT_comparision(data, dataSLIP, Tstart, Tend)
    fig.savefig(fig_dir+'/COT_comparision.eps', dpi=500)
if plotting_foot_pos_comparision:
    Tstart = 5.11
    Tend = 5.91
    data = np.load('pronk_vel_1ms/states.npz', allow_pickle=True)
    dataSLIP = np.load('pronk_vel_1ms/statesSLIP.npz', allow_pickle=True)
    fig = foot_pos_comparision(data, dataSLIP, Tstart, Tend)
    fig.savefig(fig_dir+'/foot_pos_comparision.eps', dpi=500)


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