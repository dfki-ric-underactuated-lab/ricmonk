import asyncio
import numpy as np
import moteus
import moteus_pi3hat
import time
from datetime import datetime
import os
import matplotlib.pyplot as plt
import pandas as pd
import openpyxl 
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
                   read_imu_data,
                   zero_offset_two_motors,
                   read_two_motors_data,
                   state_estimation,
                   send_rad_command_w_check,
                   state_estimation_even
                   )

TAU_LIMIT = 4
DATA = prepare_store_data(n=8000)
TEST = 'ladder_OddBrachFrontRelease'

BUS_NUMBER = 2
MOTOR_ID = [8,9]
INDEX = [0]
IMU_CONFIG_ODD = (0, 90, 180)

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

def kb_cond(brach, t, phi_vel, tail_vel):
    return (
        (t < 0.05 or -1.3 < tail_vel)
        if brach == "even"
        else (t < 0.05 or abs(phi_vel) < 1.3)
    )

async def kb_state_estimate(brach, pr, tail_pos, tail_vel):
    if brach == "odd":
            arm1_pos, arm1_vel, raw_imu_pos, raw_imu_vel = await state_estimation(
                pr=pr, pos_tail=tail_pos, vel_tail=tail_vel
            )

    else:
        #state = run(state_estimation_v2(pr=imu))  #In RicMonk, there's no need for two different ways of state estimation.
        arm1_pos, arm1_vel, raw_imu_pos, raw_imu_vel = await state_estimation_even(
            pr=pr, pos_tail=tail_pos, vel_tail=tail_vel
        )
    return arm1_pos, arm1_vel, raw_imu_pos, raw_imu_vel # raw_imu_pos = alpha_tail, raw_imu_vel = omega_tail

async def Kickback(
    n, torque_tail, torque_phi, tau_limit_KB, servo_tail, servo_phi, init_euler_xyz, imu, brach, t0_experiment, i, releaseDirection
):
    success = False
    meas_vel_phi = 94305
    meas_vel_tail = 94305
    """
    index_KB_extra = 0
    meas_dt_extra = 0
    t_extra = 0
            
    torque_tail_extra = 2
    torque_phi_extra = 2
    while t_extra<0.03:

        start_loop = time.time()
        t_extra += meas_dt_extra

        tau_tail_cmd_extra = np.clip(torque_tail_extra, -tau_limit_KB, tau_limit_KB)
        tau_phi_cmd_extra = np.clip(torque_phi_extra, -tau_limit_KB, tau_limit_KB)

        (meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi) = await send_rad_command_w_check(
            controller_obj_tail=servo_tail,
            controller_obj_phi=servo_phi, 
            pos_tail=0.0, 
            vel_tail=0.0,
            tau_tail=tau_tail_cmd_extra, 
            pos_phi=0.0,  
            vel_phi=0.0, 
            tau_phi=tau_phi_cmd_extra,
            tau_limit=tau_limit_KB,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=float("nan"),
            prev_meas_pos_tail=meas_pos_tail,
            prev_meas_vel_tail=meas_vel_tail,
            prev_meas_tau_tail=meas_tau_tail,
            prev_meas_pos_phi=meas_pos_phi,
            prev_meas_vel_phi=meas_vel_phi,
            prev_meas_tau_phi=meas_tau_phi,
            index=index_KB
        )

        index_KB += 1
        INDEX[0] += 1
        meas_dt_extra = time.time() - start_loop
        if t>0.04:
            (meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi) = await send_rad_command_w_check(
                controller_obj_tail=servo_tail,
                controller_obj_phi=servo_phi, 
                pos_tail=0.0, 
                vel_tail=0.0,
                tau_tail=0, 
                pos_phi=0.0,  
                vel_phi=0.0, 
                tau_phi=0,
                tau_limit=tau_limit_KB,
                kp_scale=0.0,
                kd_scale=0.0,
                watchdog_timeout=float("nan"),
                prev_meas_pos_tail=meas_pos_tail,
                prev_meas_vel_tail=meas_vel_tail,
                prev_meas_tau_tail=meas_tau_tail,
                prev_meas_pos_phi=meas_pos_phi,
                prev_meas_vel_phi=meas_vel_phi,
                prev_meas_tau_phi=meas_tau_phi,
                index=index_KB
            )
            
            print("")
            print("SHUTTING DOWN 'CAUSE NOTHIN WORKED BRUH!")
            print("")
            await servo_tail.set_stop()
            await servo_phi.set_stop()
            break
            
    print('Sleeping for a couple of seconds' )
    print("")
    await asyncio.sleep(2)
    print('Starting in three')
    # Disabling the motor
    await servo_tail.set_stop()
    await servo_phi.set_stop()
    await asyncio.sleep(3)
    """
            
    index_KB = 0
    meas_dt = 0
    t = 0
    
    meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi = await send_rad_command(
        controller_obj_tail=servo_tail,
        controller_obj_phi=servo_phi,
        pos_tail=0.0,  # although 0, kp = 0 gives pos no affect
        vel_tail=0.0,
        tau_tail=0,  # although 0, kd = 0 gives vel no affect
        pos_phi=0.0,  # although 0, kp = 0 gives pos no affect
        vel_phi=0.0,  # although 0, kd = 0 gives vel no affect
        tau_phi=0,
        tau_limit=1,
        kp_scale=0.0,
        kd_scale=0.0,
        watchdog_timeout=float("nan"),
        )
            
    while kb_cond(brach, t, meas_vel_phi, meas_vel_tail):
        #print(f't: {t}')
        #print(f't < 0.05: {t < 0.05}')
        #print(f'meas_vel_phi: {meas_vel_phi}')
        #print(f'meas_vel_tail: {meas_vel_tail}')
        #if brach == 'even':
        #    res = -1.3 < meas_vel_tail
        #else:
        #    res = -1.3 < meas_vel_tail
        #print(f'meas_vel_phi mod within 1.3?: {res}')
        #print(f'kb_cond: {kb_cond(brach, t, meas_vel_phi, meas_vel_tail)}')
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
        tau_tail_cmd = np.clip(torque_tail, -tau_limit_KB, tau_limit_KB)
        tau_phi_cmd = np.clip(torque_phi, -tau_limit_KB, tau_limit_KB)

        #meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi = await send_rad_command(
        #    controller_obj_tail=servo_tail,
        #    controller_obj_phi=servo_phi,
        #    pos_tail=0.0,  # although 0, kp = 0 gives pos no affect
        #    vel_tail=0.0,
        #    tau_tail=tau_tail_cmd,  # although 0, kd = 0 gives vel no affect
        #    pos_phi=0.0,  # although 0, kp = 0 gives pos no affect
        #    vel_phi=0.0,  # although 0, kd = 0 gives vel no affect
        #    tau_phi=tau_phi_cmd,
        #    tau_limit=tau_limit_KB,
        #    kp_scale=0.0,
        #    kd_scale=0.0,
        #    watchdog_timeout=float("nan"),
        #)
        (meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi) = await send_rad_command_w_check(
            controller_obj_tail=servo_tail,
            controller_obj_phi=servo_phi, 
            pos_tail=0.0, 
            vel_tail=0.0,
            tau_tail=tau_tail_cmd, 
            pos_phi=0.0,  
            vel_phi=0.0, 
            tau_phi=tau_phi_cmd,
            tau_limit=tau_limit_KB,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=float("nan"),
            prev_meas_pos_tail=meas_pos_tail,
            prev_meas_vel_tail=meas_vel_tail,
            prev_meas_tau_tail=meas_tau_tail,
            prev_meas_pos_phi=meas_pos_phi,
            prev_meas_vel_phi=meas_vel_phi,
            prev_meas_tau_phi=meas_tau_phi,
            index=index_KB
        )
        

        
        # store measured data
        data_append(
            phase="kickback",
            index=INDEX[0],
            TIME=time.time() - t0_experiment,
            ARM1_MEAS_POS=np.nan,
            ARM1_MEAS_VEL=np.nan,
            TAIL_MEAS_POS=meas_pos_tail,
            TAIL_MEAS_VEL=meas_vel_tail,
            PHI_MEAS_POS=meas_pos_phi,
            PHI_MEAS_VEL=meas_vel_phi,
            ARM2_MEAS_POS=np.nan,
            ARM2_MEAS_VEL=np.nan,
            TAIL_CMD_TAU=torque_tail,
            TAIL_MEAS_TAU=meas_tau_tail,
            TAIL_CLIPPED_TAU=tau_tail_cmd,
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
        if t>0.15:
            (meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi) = await send_rad_command_w_check(
                controller_obj_tail=servo_tail,
                controller_obj_phi=servo_phi, 
                pos_tail=0.0, 
                vel_tail=0.0,
                tau_tail=0, 
                pos_phi=0.0,  
                vel_phi=0.0, 
                tau_phi=0,
                tau_limit=tau_limit_KB,
                kp_scale=0.0,
                kd_scale=0.0,
                watchdog_timeout=float("nan"),
                prev_meas_pos_tail=meas_pos_tail,
                prev_meas_vel_tail=meas_vel_tail,
                prev_meas_tau_tail=meas_tau_tail,
                prev_meas_pos_phi=meas_pos_phi,
                prev_meas_vel_phi=meas_vel_phi,
                prev_meas_tau_phi=meas_tau_phi,
                index=index_KB
            )
            
            print("")
            print("SHUTTING DOWN 'CAUSE NOTHIN WORKED BRUH!")
            print("")
            await servo_tail.set_stop()
            await servo_phi.set_stop()
            break
    #finalTime = t
    #arraySize = INDEX
    #print(f"arraySize: {arraySize}")
    print(f"Completed in {t} / 0.15 seconds")
    print(
        f"kickback loop control frequency = {1/meas_dt}, index_KB={index_KB}"
    )
    if abs(meas_vel_phi) != 94305:
        success = True
        
    meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi = await read_two_motors_data(
            controller_obj_tail=servo_tail,
            controller_obj_phi=servo_phi)
    meas_pos_arm1, meas_vel_arm1, raw_imu_pos, raw_imu_vel = await kb_state_estimate(
            brach, imu, meas_pos_tail, meas_vel_tail
        )        
    print("")
    print("APPROXIMATE FINAL STATES AFTER RELEASE:")
    print(f"meas_pos_arm1: {meas_pos_arm1}, meas_vel_arm1: {meas_vel_arm1}")
    print(f"meas_pos_tail: {meas_pos_tail}, meas_vel_tail: {meas_vel_tail}, meas_tau_tail: {meas_tau_tail}")
    print(f"meas_pos_phi: {meas_pos_phi}, meas_vel_phi: {meas_vel_phi}, meas_tau_phi: {meas_tau_phi}")
    print("")
    (meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi) = await send_rad_command_w_check(
            controller_obj_tail=servo_tail,
            controller_obj_phi=servo_phi, 
            pos_tail=0.0, 
            vel_tail=0.0,
            tau_tail=0, 
            pos_phi=0.0,  
            vel_phi=0.0, 
            tau_phi=0,
            tau_limit=tau_limit_KB,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=float("nan"),
            prev_meas_pos_tail=meas_pos_tail,
            prev_meas_vel_tail=meas_vel_tail,
            prev_meas_tau_tail=meas_tau_tail,
            prev_meas_pos_phi=meas_pos_phi,
            prev_meas_vel_phi=meas_vel_phi,
            prev_meas_tau_phi=meas_tau_phi,
            index=index_KB
        )
    print('Sleeping for a couple of seconds' )
    print("")
    await asyncio.sleep(5)
    # Disabling the motor
    await servo_tail.set_stop()
    await servo_phi.set_stop()
    
    
    add_q = 'n' #input("Add values to excel sheet?")
    if add_q == 'y':
        fileName = f"{brach}Brach{releaseDirection}ReleaseVelocities.xlsx"
        wb = openpyxl.load_workbook(fileName) 
        sheet = wb.active 
        data = ((i , "", meas_pos_arm1, meas_vel_arm1, meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi))
        sheet.append(data)
        wb.save(fileName)
        print(f"DONE ADDING DATA to {fileName}!")
        print('')
    else:
        print('Not adding values to excel file')
    return success


def kickback(
    brach, n, torque_tail, torque_phi, tau_limit, servo_tail, servo_phi, init_euler_xyz, imu, t0_experiment, i, releaseDirection
):
    #torque_phi = torque_phi if brach == "even" else -torque_phi   #ARE YOU SURE WE NEED TO DO THIS??
    success = run(
        Kickback(
            n, torque_tail, torque_phi, tau_limit, servo_tail, servo_phi, init_euler_xyz, imu, brach, t0_experiment, i, releaseDirection
        )
    )

def run(f):
    return loop.run_until_complete(f)
    
async def init():
    transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map={BUS_NUMBER:MOTOR_ID})
    # servo_tail = moteus.Controller(id=MOTOR_ID[1], transport=transport)
    # servo_phi = moteus.Controller(id=MOTOR_ID[0], transport=transport)
    servo_9 = moteus.Controller(id=MOTOR_ID[1], transport=transport)
    servo_8 = moteus.Controller(id=MOTOR_ID[0], transport=transport)
    
    imu = imu_reset(IMU_CONFIG_ODD)  # imu_reset(IMU_CONFIG_EVEN)#
    _, _, _, init_euler_xyz = await read_imu_data(imu)
    await servo_9.set_stop()
    await servo_8.set_stop()
    controller = None
    return servo_9, servo_8, init_euler_xyz, controller, imu
    
def imu_reset(con, sleep_time=3):
    moteus_pi3hat.Pi3HatRouter(mounting_deg={"roll": 0, "pitch": 0, "yaw": 0})
    imu = moteus_pi3hat.Pi3HatRouter(
        mounting_deg={"roll": con[0], "pitch": con[1], "yaw": con[2]}
    )
    time.sleep(sleep_time)
    return imu

def main(loop, i, servo_tail, servo_phi, init_euler_xyz, controller, imu):
    #init()
    
    # brach = "odd"
    

    #servo_tail, servo_phi, init_euler_xyz, controller, imu = loop.run_until_complete(init())
    try:
        t0 = time.time()
        changeValues = input("Change values(y/n): ")
        if changeValues == 'n':
            if brach == "odd" and (releaseDirection == 'Front' or releaseDirection =='Front'):
                print("Odd front release test")  # motor 9     motor 8
                kickback(brach, 49, 2, 6.5, 11.0, servo_tail, servo_phi, init_euler_xyz, imu, t0, i, releaseDirection)

            elif brach == "odd" and (releaseDirection == 'Back' or releaseDirection =='Back'):
                print("Odd back release test")    # motor 9     motor 8
                kickback(brach, 49, 4.3, 2.2, 6.0, servo_tail, servo_phi, init_euler_xyz, imu, t0, i, releaseDirection)

            elif brach == "even" and (releaseDirection == 'Front' or releaseDirection =='Front'):
                print("Even front release test")      # motor 8     motor 9
                kickback(brach, 49, -2.0, -6.5, 8.0, servo_tail, servo_phi, init_euler_xyz, imu, t0, i, releaseDirection)

            elif brach == "even" and (releaseDirection == 'Back' or releaseDirection =='Back'):
                print("Even back release test")      # motor 8     motor 9
                kickback(brach, 49, -4.3, -2.2, 6.0, servo_tail, servo_phi, init_euler_xyz, imu, t0, i, releaseDirection)
                
        else:
            if brach == "odd" and (releaseDirection == 'Front' or releaseDirection =='Front'):
                print("Odd front release test: old values for servo tail and servo phi: 2Nm, 6.5Nm")

            elif brach == "odd" and (releaseDirection == 'Back' or releaseDirection =='Back'):
                print("Odd back release test: old values for servo tail and servo phi: 4.3Nm, 2.2Nm")

            elif brach == "even" and (releaseDirection == 'Front' or releaseDirection =='Front'):
                print("Even front release test: old values for servo tail and servo phi: -2Nm, -6.5Nm")

            elif brach == "even" and (releaseDirection == 'Back' or releaseDirection =='Back'):
                print("Even back release test: old values for servo tail and servo phi: -4.3Nm, -2.2Nm")
            
            servo_tail_torque = float(input("Input servo tail torque: "))
            servo_phi_torque = float(input("Input servo phi torque: "))
            
            if brach == "odd" and (releaseDirection == 'Front' or releaseDirection =='Front'):
                print("Odd front release test")                                # motor 9     motor 8
                kickback(brach, 49, servo_tail_torque, servo_phi_torque, 11.0, servo_tail, servo_phi, init_euler_xyz, imu, t0, i, releaseDirection)

            elif brach == "odd" and (releaseDirection == 'Back' or releaseDirection =='Back'):
                print("Odd back release test")                                # motor 9     motor 8
                kickback(brach, 49, servo_tail_torque, servo_phi_torque, 6.0, servo_tail, servo_phi, init_euler_xyz, imu, t0, i, releaseDirection)

            elif brach == "even" and (releaseDirection == 'Front' or releaseDirection =='Front'):
                print("Even front release test")                                # motor 8     motor 9
                kickback(brach, 49, servo_tail_torque, servo_phi_torque, 8.0, servo_tail, servo_phi, init_euler_xyz, imu, t0, i, releaseDirection)

            elif brach == "even" and (releaseDirection == 'Back' or releaseDirection =='Back'):
                print("Even back release test")                                # motor 8     motor 9
                kickback(brach, 49, servo_tail_torque, servo_phi_torque, 6.0, servo_tail, servo_phi, init_euler_xyz, imu, t0, i, releaseDirection)

        #kickback(brach, 49, 4.5, 4.2, 6.0, servo_tail, servo_phi, init_euler_xyz, imu, t0) #BACK RELEASE #got back release(ID=8) by giving 4.5Nm to tail_motor, and 4Nm to phi_motor. BRACH = ODD
        #kickback(brach, 49, 5, 2, 6.0, servo_tail, servo_phi, init_euler_xyz, imu, t0) #BACK RELEASE #got back release(ID=8) by giving 5Nm to tail_motor, and 2Nm to phi_motor. BRACH = ODD
        #got back release(ID=8) by giving 4Nm to tail_motor, and 3Nm to phi_motor. BRACH = ODD
        #kickback("odd", 49, 0, 0, 8.0, servo_tail, servo_phi, init_euler_xyz, imu, t0)
        
        # ********** ODD RELEASE TRIALS **********
        #QUESTIONABLE
        #kickback(brach, 49, 2, 6.4, 8.0, servo_tail, servo_phi, init_euler_xyz, imu, t0) #FRONT RELEASE #got front release(ID=8) by giving 2Nm to tail_motor, and 7.5m to phi_motor. Hehe. BRACH = ODD
        #CONFIRMED
        #kickback(brach, 49, 3, 9.2, 11.0, servo_tail, servo_phi, init_euler_xyz, imu, t0, i)
        
        #CONFIRMED
        #kickback(brach, 49, 4.3, 2.2, 6.0, servo_tail, servo_phi, init_euler_xyz, imu, t0, i) #BACK RELEASE #got back release(ID=8) by giving 4.5Nm to tail_motor, and 2Nm to phi_motor. BRACH = ODD
        
        # ********** EVEN RELEASE TRIALS **********
        #No, this front release is not the front release we need. We need the arm attached to motor ID 8 to release.
        #kickback(brach, 49, -2, 5.0, 6.0, servo_tail, servo_phi, init_euler_xyz, imu, t0) #FRONT RELEASE #got front release(ID=9) by giving -2Nm to tail_motor, and 5Nm to phi_motor. Hehe. BRACH = ODD
        
        #CONFIRMED
        #kickback(brach, 49, -4.3, -2.2, 6.0, servo_tail, servo_phi, init_euler_xyz, imu, t0, i) #BACK RELEASE #got front release(ID=9) by giving -2Nm to tail_motor, and -3.75Nm to phi_motor. BRACH = EVEN
        
        #CONFIRMED
        #kickback(brach, 49, -2.0, -6.5, 8.0, servo_tail, servo_phi, init_euler_xyz, imu, t0, i) #FRONT RELEASE #got front release(ID=9) by giving -7Nm to tail_motor, and -2Nm to phi_motor. Hehe. BRACH = EVEN
        

    finally:
        index = INDEX[0]
        valid_data = {
            field: DATA[i][:index]
            for i, field in enumerate(DATA._fields)
            if isinstance(DATA[i], np.ndarray)
        }
        answer = 'n'#input("Should I save results?")
        if answer == 'y':
            directory = make_results_directory(TEST)
            # Save Trajectory to a csv file to be sent to the motor.
            save_data(valid_data, directory + "/measured.csv")
            plot_custom_data_with_dir_meas(directory, valid_data, show=True)
        # Disabling the motor
        run(servo_tail.set_stop())
        run(servo_phi.set_stop())
        print("Motor disabled.")

zero_offset_two_motors(BUS_NUMBER, MOTOR_ID)
loop = asyncio.get_event_loop()
servo_9, servo_8, init_euler_xyz, controller, imu = loop.run_until_complete(init())

brach = input("Testing odd brachiations? Or even? (o/e) ")
assert brach in ('o', 'e', 'O', 'E')
if brach == 'o' or brach == 'O':
    brach = "odd"
    servo_tail = servo_9
    servo_phi = servo_8
elif brach == 'e' or brach == 'E':
    brach = "even"
    servo_tail = servo_8
    servo_phi = servo_9

# if brach == "odd":
#     servo_tail = servo_9
#     servo_phi = servo_8
# else:
#     servo_tail = servo_8
#     servo_phi = servo_9

releaseDirection = input("Testing back releases? Or front? (b/f) ")
assert releaseDirection in ('f', 'b', 'F', 'B')

if releaseDirection == 'b' or releaseDirection == 'B':
    releaseDirection = "Back"
elif releaseDirection == 'f' or releaseDirection == 'F':
    releaseDirection = "Front"

if input("""Was the robot set to zero before running the code? 
    Did you give the right information regarding type of type of release and the release direction? 
    Is the robot in the required release position? (y)""") == 'y':

    go = 'y'
    i = 0
    
    while go == 'y':
        i = i + 1
        print("")
        print("**************************************")
        print(f'Iteration: {i}' )
        main(loop, i, servo_tail, servo_phi, init_euler_xyz, controller, imu)
        print("")
        print(f'Iteration: {i} over' )
        print("")
        
        go = input("Reset? Ready to go more? (y) ")
        
    print("OK, bye!")
else:
    print("Do it, and restart")

#for i in range(1,26):
#    print("")
#    print(f'Iteration: {i}' )
#    if input("Reset? Ready to go more?") == 'y':
#        #
#        #if input("Reset to required release position?")=='y':
#        #loop = asyncio.get_event_loop()
#        main(loop, i, servo_tail, servo_phi, init_euler_xyz, controller, imu)
#    else:
#        print("OK bye!")
#
#print("Done with experiment!")

    
#go = 'y'
#i = 1
#while go == 'y':
#    print("")
#    print(f'Iteration: {i}' )
#    
#for i in range(1,26):
#    print("")
#    print(f'Iteration: {i}' )
#    if input("Reset? Ready to go more?") == 'y':
#        #
#        #if input("Reset to required release position?")=='y':
#        #loop = asyncio.get_event_loop()
#        main(loop, i, servo_tail, servo_phi, init_euler_xyz, controller, imu)
#    else:
#        print("OK bye!")
#       break
#print("Done with experiment!")

#if input("Press enter"):
#    exit()
#else:
#    exit()
#loop = asyncio.get_event_loop()
#main(loop)


