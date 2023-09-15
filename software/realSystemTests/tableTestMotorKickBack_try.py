import asyncio
import numpy as np
import moteus
import moteus_pi3hat
import time
from datetime import datetime
import os
import matplotlib.pyplot as plt
# create empty csv file using pandas #https://89devs.com/python/howto/create-empty-csv-file/#:~:text=To%20create%20an%20empty%20csv%20file%20using%20with%20open%2C%20pass,the%20content%20to%20the%20file.
import pandas as pd
# Local imports
from utils import (send_rad_command,
                   save_data,
                   read_data,
                   zero_offset,
                   namedtuple,
                   generate_path,
                   plot_data,
                   send_rad_command_one_motor,
                   prepare_store_data,
                   make_results_directory,
                   plot_custom_data_with_dir_meas,
                   read_imu_data
                   )

TAU_LIMIT = 4
DATA = prepare_store_data(n=8000)
TEST = 'tableKickBack'

BUS_NUMBER = 2
MOTOR_ID = [8]
INDEX = [0]
IMU_CONFIG_ODD = (0, -90, 0)

def data_append(
    phase,
    index,

    TIME=np.nan,
    ARM1_MEAS_POS=np.nan,
    ARM1_MEAS_VEL=np.nan,
    TAIL_MEAS_POS=np.nan,
    TAIL_MEAS_VEL=np.nan,
    ARM2_MEAS_POS=np.nan,
    ARM2_MEAS_VEL=np.nan,
    PHI_MEAS_POS=np.nan,
    PHI_MEAS_VEL=np.nan,
    TAIL_DES_TAU=np.nan,
    TAIL_CMD_TAU=np.nan,
    TAIL_MEAS_TAU=np.nan,
    TAIL_CLIPPED_TAU=np.nan,
    PHI_DES_TAU=np.nan,
    PHI_CMD_TAU=np.nan,
    PHI_MEAS_TAU=np.nan,
    PHI_CLIPPED_TAU=np.nan,
    RAW_IMU_POS=np.nan,
    RAW_IMU_VEL=np.nan,
    ARM2_DES_POS=np.nan,
    ARM2_DES_VEL=np.nan,
    PHI_DES_POS=np.nan,
    PHI_DES_VEL=np.nan,
    TAIL_DES_POS=np.nan,
    TAIL_DES_VEL=np.nan,
    ARM1_DES_POS=np.nan,
    ARM1_DES_VEL=np.nan,
    TVLQR_K1=np.nan,
    TVLQR_K2=np.nan,
    TVLQR_K3=np.nan,
    TVLQR_K4=np.nan,
    TVLQR_K5=np.nan,
    TVLQR_K6=np.nan,
    TVLQR_K7=np.nan,
    TVLQR_K8=np.nan,
    TVLQR_K9=np.nan,
    TVLQR_K10=np.nan,
    TVLQR_K11=np.nan,
    TVLQR_K12=np.nan,
    STATE_MACHINE_FLAG=np.nan,
):
    if phase == "swing":
        DATA.arm1_meas_Ang_pos[index] = ARM1_MEAS_POS
        DATA.arm1_meas_Ang_vel[index] = ARM1_MEAS_VEL
        DATA.tail_meas_Ang_pos[index] = TAIL_MEAS_POS
        DATA.tail_meas_Ang_vel[index] = TAIL_MEAS_VEL
        DATA.tail_meas_tau[index] = TAIL_MEAS_TAU
        DATA.tail_cmd_tau[index] = TAIL_CMD_TAU
        DATA.tail_clipped_tau[index] = TAIL_CLIPPED_TAU
        DATA.phi_meas_Ang_pos[index] = PHI_MEAS_POS
        DATA.phi_meas_Ang_vel[index] = PHI_MEAS_VEL
        DATA.phi_meas_tau[index] = PHI_MEAS_TAU
        DATA.phi_cmd_tau[index] = PHI_CMD_TAU
        DATA.phi_clipped_tau[index] = PHI_CLIPPED_TAU
        DATA.arm2_meas_Ang_pos[index] = ARM2_MEAS_POS
        DATA.arm2_meas_Ang_vel[index] = ARM2_MEAS_VEL
        DATA.meas_time[index] = TIME
        DATA.arm1_des_Ang_pos_store[index] = ARM1_DES_POS
        DATA.arm1_des_Ang_vel_store[index] = ARM1_DES_VEL
        DATA.tail_des_Ang_pos_store[index] = TAIL_DES_POS
        DATA.tail_des_Ang_vel_store[index] = TAIL_DES_VEL
        DATA.tail_des_tau_store[index] = TAIL_DES_TAU
        DATA.phi_des_Ang_pos_store[index] = PHI_DES_POS
        DATA.phi_des_Ang_vel_store[index] = PHI_DES_VEL
        DATA.arm2_des_Ang_pos_store[index] = ARM2_DES_POS
        DATA.arm2_des_Ang_vel_store[index] = ARM2_DES_VEL
        DATA.phi_des_tau_store[index] = PHI_DES_TAU
        DATA.raw_imu_pos[index] = RAW_IMU_POS
        DATA.raw_imu_vel[index] = RAW_IMU_VEL
        DATA.k1_store[index] = TVLQR_K1
        DATA.k2_store[index] = TVLQR_K2
        DATA.k3_store[index] = TVLQR_K3
        DATA.k4_store[index] = TVLQR_K4
        DATA.k5_store[index] = TVLQR_K5
        DATA.k6_store[index] = TVLQR_K6
        DATA.k7_store[index] = TVLQR_K7
        DATA.k8_store[index] = TVLQR_K8
        DATA.k9_store[index] = TVLQR_K9
        DATA.k10_store[index] = TVLQR_K10
        DATA.k11_store[index] = TVLQR_K11
        DATA.k12_store[index] = TVLQR_K12
        DATA.state_machine_flag[index] = 1

    elif phase == "kickback":
        DATA.arm1_meas_Ang_pos[index] = ARM1_MEAS_POS
        DATA.arm1_meas_Ang_vel[index] = ARM1_MEAS_VEL
        DATA.tail_meas_Ang_pos[index] = TAIL_MEAS_POS
        DATA.tail_meas_Ang_vel[index] = TAIL_MEAS_VEL
        DATA.tail_meas_tau[index] = TAIL_MEAS_TAU
        DATA.tail_cmd_tau[index] = TAIL_CMD_TAU
        DATA.tail_clipped_tau[index] = TAIL_CLIPPED_TAU
        DATA.phi_meas_Ang_pos[index] = PHI_MEAS_POS
        DATA.phi_meas_Ang_vel[index] = PHI_MEAS_VEL
        DATA.phi_meas_tau[index] = PHI_MEAS_TAU
        DATA.phi_cmd_tau[index] = PHI_CMD_TAU
        DATA.phi_clipped_tau[index] = PHI_CLIPPED_TAU
        DATA.arm2_meas_Ang_pos[index] = ARM2_MEAS_POS
        DATA.arm2_meas_Ang_vel[index] = ARM2_MEAS_VEL
        DATA.meas_time[index] = TIME
        DATA.arm1_des_Ang_pos_store[index] = np.nan
        DATA.arm1_des_Ang_vel_store[index] = np.nan
        DATA.tail_des_Ang_pos_store[index] = np.nan
        DATA.tail_des_Ang_vel_store[index] = np.nan
        DATA.tail_des_tau_store[index] = np.nan
        DATA.phi_des_Ang_pos_store[index] = np.nan
        DATA.phi_des_Ang_vel_store[index] = np.nan
        DATA.phi_des_tau_store[index] = np.nan
        DATA.arm2_des_Ang_pos_store[index] = np.nan
        DATA.arm2_des_Ang_vel_store[index] = np.nan
        DATA.raw_imu_pos[index] = RAW_IMU_POS
        DATA.raw_imu_vel[index] = RAW_IMU_VEL
        DATA.k1_store[index] = np.nan
        DATA.k2_store[index] = np.nan
        DATA.k3_store[index] = np.nan
        DATA.k4_store[index] = np.nan
        DATA.k5_store[index] = np.nan
        DATA.k6_store[index] = np.nan
        DATA.k7_store[index] = np.nan
        DATA.k8_store[index] = np.nan
        DATA.k9_store[index] = np.nan
        DATA.k10_store[index] = np.nan
        DATA.k11_store[index] = np.nan
        DATA.k12_store[index] = np.nan
        DATA.state_machine_flag[index] = 0
     
    elif phase == "catch":
        DATA.arm1_meas_Ang_pos[index] = ARM1_MEAS_POS
        DATA.arm1_meas_Ang_vel[index] = ARM1_MEAS_VEL
        DATA.tail_meas_Ang_pos[index] = TAIL_MEAS_POS
        DATA.tail_meas_Ang_vel[index] = TAIL_MEAS_VEL
        DATA.tail_meas_tau[index] = TAIL_MEAS_TAU
        DATA.tail_cmd_tau[index] = TAIL_CMD_TAU
        DATA.tail_clipped_tau[index] = TAIL_CLIPPED_TAU
        DATA.phi_meas_Ang_pos[index] = PHI_MEAS_POS
        DATA.phi_meas_Ang_vel[index] = PHI_MEAS_VEL
        DATA.phi_meas_tau[index] = PHI_MEAS_TAU
        DATA.phi_cmd_tau[index] = PHI_CMD_TAU
        DATA.phi_clipped_tau[index] = PHI_CLIPPED_TAU
        DATA.arm2_meas_Ang_pos[index] = ARM2_MEAS_POS
        DATA.arm2_meas_Ang_vel[index] = ARM2_MEAS_VEL
        DATA.meas_time[index] = TIME
        DATA.arm1_des_Ang_pos_store[index] = np.nan
        DATA.arm1_des_Ang_vel_store[index] = np.nan
        DATA.tail_des_Ang_pos_store[index] = np.nan
        DATA.tail_des_Ang_vel_store[index] = np.nan
        DATA.tail_des_tau_store[index] = np.nan
        DATA.phi_des_Ang_pos_store[index] = np.nan
        DATA.phi_des_Ang_vel_store[index] = np.nan
        DATA.phi_des_tau_store[index] = np.nan
        DATA.arm2_des_Ang_pos_store[index] = np.nan
        DATA.arm2_des_Ang_vel_store[index] = np.nan
        DATA.raw_imu_pos[index] = RAW_IMU_POS
        DATA.raw_imu_vel[index] = RAW_IMU_VEL
        DATA.k1_store[index] = np.nan
        DATA.k2_store[index] = np.nan
        DATA.k3_store[index] = np.nan
        DATA.k4_store[index] = np.nan
        DATA.k5_store[index] = np.nan
        DATA.k6_store[index] = np.nan
        DATA.k7_store[index] = np.nan
        DATA.k8_store[index] = np.nan
        DATA.k9_store[index] = np.nan
        DATA.k10_store[index] = np.nan
        DATA.k11_store[index] = np.nan
        DATA.k12_store[index] = np.nan
        DATA.state_machine_flag[index] = 2

def kb_cond(brach, t, phi_vel):
    return (
        (t < 0.05 or -1.3 < phi_vel)
        if brach == "even"
        else (t < 0.05 or phi_vel < 1.3)
    )

async def kb_state_estimate(brach, pr, tail_pos, tail_vel):
    pos_arm1, vel_arm1, alpha_tail, omega_tail = await state_estimation(pr=pr, pos_tail=tail_pos, vel_tail=tail_vel)
    return pos_arm1, vel_arm1, alpha_tail, omega_tail

async def Kickback(
    n, torque_phi, tau_limit_KB, servo_phi, init_euler_xyz, pr, brach, t0_experiment
):
    success = False
    meas_vel_phi = 94305

    index_KB = 0
    meas_dt = 0
    t = 0
    """
    await servo.set_stop()  # FIXME HACK

    motorLoops = n

    # obtain starting velocity (should be zero)

    (el_pos, el_vel, el_tau) = await send_rad_command(
        controller_obj=servo,
        pos=0,
        vel=0,
        tau=0,  # do not want to send a torque
        tau_limit=0,
        kp_scale=0,
        kd_scale=0,
        watchdog_timeout=float("nan"),
    )  # 'nan' to prevent needing to stop the motor before next motion command
    """
    while kb_cond(brach, t, meas_vel_phi):
        print(f't: {t}')
        print(f't < 0.05: {t < 0.05}')
        print(f'meas_vel_phi: {meas_vel_phi}')
        if brach == 'even':
            res = -1.3 < meas_vel_phi
        else:
            res = meas_vel_phi < 1.3
        print(f'meas_vel_phi mod within 1.3?: {res}')
        print(f'kb_cond: {kb_cond(brach, t, meas_vel_phi)}')
        start_loop = time.time()
        t += meas_dt
        """
        if index_KB > motorLoops:
            # safety feature, motor loops should never be exceeded
            # kill motor
            await servo.set_stop()
            torque = 0
            print("servo killed at loop number:", index_KB)
            print("success set to", success)
            # exit kickback loop and do not enter LR loop
            success = False
            el_vel = 94305  # impossible velocity value to indicate failure
            break
        """
        tau_phi_cmd = np.clip(torque_phi, -tau_limit_KB, tau_limit_KB)

        meas_pos_phi, meas_vel_phi, meas_tau_phi = await send_rad_command_one_motor(
            controller_obj=servo_phi,
            pos=0.0,  # although 0, kp = 0 gives pos no affect
            vel=0.0,  # although 0, kd = 0 gives vel no affect
            tau=tau_phi_cmd,
            tau_limit=tau_limit_KB,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=float("nan"),
        )

        
        # store measured data
        data_append(
            phase="kickback",
            index=INDEX[0],
            TIME=time.time() - t0_experiment,
            ARM1_MEAS_POS=np.nan,
            ARM1_MEAS_VEL=np.nan,
            TAIL_MEAS_POS=np.nan,
            TAIL_MEAS_VEL=np.nan,
            PHI_MEAS_POS=meas_pos_phi,
            PHI_MEAS_VEL=meas_vel_phi,
            ARM2_MEAS_POS=np.nan,
            ARM2_MEAS_VEL=np.nan,
            TAIL_CMD_TAU=np.nan,
            TAIL_MEAS_TAU=np.nan,
            TAIL_CLIPPED_TAU=np.nan,
            PHI_CMD_TAU=torque_phi,
            PHI_MEAS_TAU=meas_tau_phi,
            PHI_CLIPPED_TAU=tau_phi_cmd,
            RAW_IMU_POS=np.nan,
            RAW_IMU_VEL=np.nan,
            ARM2_DES_POS=np.nan,
            ARM2_DES_VEL=np.nan,
            PHI_DES_POS=np.nan,
            PHI_DES_VEL=np.nan,
            TAIL_DES_POS=np.nan,
            TAIL_DES_VEL=np.nan,
            ARM1_DES_POS=np.nan,
            ARM1_DES_VEL=np.nan,
        )

        index_KB += 1
        INDEX[0] += 1
        meas_dt = time.time() - start_loop
        if t>3:
            break
    print(
        f"kickback loop control frequency = {1/meas_dt}, index_KB={index_KB}"
    )
    if abs(meas_vel_phi) != 94305:
        success = True

    # Disabling the motor
    await servo_phi.set_stop()
    return success


def kickback(
    brach, n, torque_phi, tau_limit, servo_phi, init_euler_xyz, imu, t0_experiment
):
    torque_phi = torque_phi if brach == "odd" else -torque_phi   #ARE YOU SURE WE NEED TO DO THIS??
    success = run(
        Kickback(
            n, torque_phi, tau_limit, servo_phi, init_euler_xyz, imu, brach, t0_experiment
        )
    )

def run(f):
    return loop.run_until_complete(f)
    
async def init():
    transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map={BUS_NUMBER:MOTOR_ID})
    servo_phi = moteus.Controller(id=MOTOR_ID[0], transport=transport)
    
    imu = imu_reset(IMU_CONFIG_ODD)  # imu_reset(IMU_CONFIG_EVEN)#
    _, _, _, init_euler_xyz = await read_imu_data(imu)
    await servo_phi.set_stop()
    controller = None
    return servo_phi, init_euler_xyz, controller, imu
    
def imu_reset(con, sleep_time=3):
    moteus_pi3hat.Pi3HatRouter(mounting_deg={"roll": 0, "pitch": 0, "yaw": 0})
    imu = moteus_pi3hat.Pi3HatRouter(
        mounting_deg={"roll": con[0], "pitch": con[1], "yaw": con[2]}
    )
    time.sleep(sleep_time)
    return imu

def main(loop):
    #init()
    
    brach = "odd"
    servo_phi, init_euler_xyz, controller, imu = loop.run_until_complete(init())
    try:
        t0 = time.time()
        kickback(brach, 49, 1.75, 6.0, servo_phi, init_euler_xyz, imu, t0)

    finally:
        index = INDEX[0]
        valid_data = {
            field: DATA[i][:index]
            for i, field in enumerate(DATA._fields)
            if isinstance(DATA[i], np.ndarray)
        }
        directory = make_results_directory(TEST)
        # Save Trajectory to a csv file to be sent to the motor.
        save_data(valid_data, directory + "/measured.csv")
        plot_custom_data_with_dir_meas(directory, valid_data, show=True)
        # Disabling the motor
        run(servo_phi.set_stop())
        print("Motor disabled.")




zero_offset(BUS_NUMBER, MOTOR_ID[0])
#if input("Press enter"):
#    exit()
#else:
#    exit()
loop = asyncio.get_event_loop()
main(loop)


