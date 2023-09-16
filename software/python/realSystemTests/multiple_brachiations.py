#!/usr/bin/env python3

import sys
import asyncio
import numpy as np
import moteus
import moteus_pi3hat
import time
import os
from utils import (
    send_rad_command,
    read_imu_data,
    state_estimation,
    plot_custom_data_with_dir,
    save_data,
    read_data,
    prepare_store_data,
    prepare_des_data,
    zero_offset_two_motors,
    create_nominal_pcws,
    tau_tvlqr,
    motor_test_parameters,
    make_results_directory,
    plot_custom_data_with_dir_meas,
    calc_pd_tau_ricMonk,
    make_results_directory_test_traj,
    send_rad_command_w_check
)

from reinforcement_learning_controller import get_controller

FOLDER_NAME = "data/trajectories/closed_loop"
TAU_LIMIT = 4

TEST = sys.argv[1]
assert TEST in ["tvlqr", "pd", "rl", "ff_replay"]
FILE_NAME = sys.argv[2]
TRAJ = FILE_NAME    #These three lines are written to get the trajectory type
TRAJ = TRAJ.split('_')
TRAJ = TRAJ[0]
print(f"traj: {TRAJ}")
BUS_NUMBER = int(sys.argv[3])
motorIDtext = sys.argv[4]
#print(motorIDtext)
#print(motorIDtext[1])
MOTOR_ID = [int(motorIDtext[1]), int(motorIDtext[3])]
#print(MOTOR_ID)



# IMU_CONFIG_ODD = (90, 0, 90) #Changing this to (0,0,0)
IMU_CONFIG_ODD = (0, 90, 180)
CSV_DATA = read_data(folder=FOLDER_NAME, file=FILE_NAME, up_directory=1)
DATA = prepare_store_data(n=8000)
DATA_DES = prepare_des_data(csv_data = CSV_DATA, controller = TEST)
NOMINAL_PCWS = create_nominal_pcws(DATA_DES)
INDEX = [0]

# ---
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
      


class BrachiationFailure(Exception):
    pass


def run(f):
    return loop.run_until_complete(f)

#REPLACED 'kb_cond' FUNCTION WITH 'kb_cond' FROM THE LADDER BAR RELEASE TESTS
def kb_cond(brach, t, phi_vel, tail_vel):
    return (
        (t < 0.05 or -1.3 < tail_vel)
        if brach == "even"
        else (t < 0.05 or abs(phi_vel) < 1.3)
    )

async def kb_state_estimate(brach, pr, tail_pos, tail_vel):
    pos_arm1, vel_arm1, alpha_tail, omega_tail = await state_estimation(pr=pr, pos_tail=tail_pos, vel_tail=tail_vel)
    return pos_arm1, vel_arm1, alpha_tail, omega_tail
    
#There is no requirement for even/odd brach, and we anyway call read_imu, so no requirement of the argument init_euler_xyz

# async def kb_state_estimate(brach, pr, phi_pos, phi_vel, init_euler_xyz):
#     return (
#         (
#             await state_estimation(
#                 pr=pr, pos_phi=phi_pos, vel_phi=phi_vel, imu_init=init_euler_xyz[0]
#             )
#         )
#         if brach == "odd"
#         else (await state_estimation_v2(pr))
#     )

#Assuming some torque should be applied to the tail as well. Maybe not required. If not required, we could give torque_tail zero value.
#REPLACED OLD 'Kickback' FUNCTION WITH 'Kickback' FUNCTION FROM LADDER BAR RELEASE TESTS
async def Kickback(
    n, torque_tail, torque_phi, tau_limit_KB, servo_tail, servo_phi, init_euler_xyz, pr, brach, t0_experiment
):
    success = False
    meas_vel_phi = 94305
    meas_vel_tail = 94305

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
        # print(f't: {t}')
        # print(f't < 0.05: {t < 0.05}')
        # print(f'meas_vel_phi: {meas_vel_phi}')
        # print(f'meas_vel_tail: {meas_vel_tail}')
        # if brach == 'even':
        #     res = -1.3 < meas_vel_tail
        # else:
        #     res = -1.3 < meas_vel_tail
        # print(f'meas_vel_phi mod within 1.3?: {res}')
        # print(f'kb_cond: {kb_cond(brach, t, meas_vel_phi, meas_vel_tail)}')
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

        arm1_pos, arm1_vel, raw_imu_pos, raw_imu_vel = await kb_state_estimate(
            brach, pr, meas_pos_tail, meas_vel_tail
        )
        meas_pos_arm2 = meas_pos_phi + meas_pos_tail
        meas_vel_arm2 = meas_vel_phi + meas_vel_tail
        # store measured data
        data_append(
            phase="kickback",
            index=INDEX[0],
            TIME=time.time() - t0_experiment,
            ARM1_MEAS_POS=arm1_pos,
            ARM1_MEAS_VEL=arm1_vel,
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
            exit()
    print(f"Completed in {t} / 0.15 seconds")
    print(
        f"kickback loop control frequency = {1/meas_dt}, index_KB={index_KB}"
    )
    if abs(meas_vel_phi) != 94305:
        success = True

    # Disabling the motor
    await servo_tail.set_stop()
    await servo_phi.set_stop()
    return success


def kickback(
    brach, n, torque_tail, torque_phi, tau_limit, servo_tail, servo_phi, init_euler_xyz, imu, t0_experiment
):
    #torque_phi = torque_phi if brach == "odd" else -torque_phi   #ARE YOU SURE WE NEED TO DO THIS??  #COMMENTING THIS OUT AFTER LADDER BAR RELEASE TESTS
    success = run(
        Kickback(
            n, torque_tail, torque_phi, tau_limit, servo_tail, servo_phi, init_euler_xyz, imu, brach, t0_experiment
        )
    )
    if not success:
        raise BrachiationFailure(
            "kickback",
            brach,
            n,
            torque_tail, 
            torque_phi,
            tau_limit,
            init_euler_xyz,
            imu,
            t0_experiment,
        )


async def ch_state_estimate(brach, pr, el_pos, el_vel, init_euler_xyz):
    return (
        await state_estimation(
            pr=pr, pos_el=el_pos, vel_el=el_vel, imu_init=init_euler_xyz[0]
        )
        if brach == "odd"
        else (await state_estimation_v2(pr))
    )


def ch_cond(brach, el_pos):
    return (el_pos < -1.9) if brach == "even" else (el_pos > 1.9)


"""
async def Catch(
    n_catch,
    torque_CATCH,
    tau_limit_CATCH,
    servo,
    init_euler_xyz,
    pr,
    brach,
    t0_experiment,
):
    # success = False
    success = True  # FIXME the catch condition has problem and we disabled it for now

    index_CATCH = 0
    t_CATCH = 0
    meas_dt_CATCH = 0

    # t < 3.65 or el_vel < 1.5: #do not break for time instead take data for n loops
    while index_CATCH < n_catch:
        start_loop = time.time()
        t_CATCH += meas_dt_CATCH

        tau_cmd_CATCH = np.clip(
            torque_CATCH, -tau_limit_CATCH, tau_limit_CATCH
        )

        (el_pos, el_vel, el_tau) = await send_rad_command(
            controller_obj=servo,
            pos=0.0,  # although 0, kp = 0 gives pos no affect
            vel=0.0,  # although 0, kd = 0 gives vel no affect
            tau=tau_cmd_CATCH,
            tau_limit=tau_limit_CATCH,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=0.05,
        )

        sh_pos, sh_vel, raw_imu_pos, raw_imu_vel = await ch_state_estimate(
            brach, pr, el_pos, el_vel, init_euler_xyz
        )

        # FIXME HACK this slows down the loop which leads to successful catch!!!
        _, _, _, index_euler_xyz = await read_imu_data(pr)

        # store measured data
        # data_append('catch',index=INDEX[0])
        data_append(
            phase="catch",
            index=INDEX[0],
            TIME=time.time() - t0_experiment,
            SH_MEAS_POS=sh_pos,
            SH_MEAS_VEL=sh_vel,
            EL_MEAS_POS=el_pos,
            EL_MEAS_VEL=el_vel,
            EL_CMD_TAU=torque_CATCH,
            EL_MEAS_TAU=el_tau,
            EL_CLIPPED_TAU=tau_cmd_CATCH,
            RAW_IMU_POS=raw_imu_pos,
            RAW_IMU_VEL=raw_imu_vel,
            EL_DES_POS=np.nan,
            EL_DES_VEL=np.nan,
            SH_DES_POS=np.nan,
            SH_DES_VEL=np.nan,
        )

        index_CATCH += 1
        INDEX[0] += 1
        meas_dt_CATCH = time.time() - start_loop
    print(f'Catch loop control frequency = {1/meas_dt_CATCH}')
    if ch_cond(brach, el_pos):
        success = True # FIXME Catch condition does not work properly, so it is disabled for now
        print(
            "stopping motor in order to restart control, index is: ",
            index_CATCH,
        )
        await servo.set_stop()
        print("Catch success, success set to: ", str(success))
    else:
        print("Catch fail, stopping motor")
        await servo.set_stop()
        print("Catch fail, do not attempt, sucess set to : ", success)

    return success

async def Catch(
    n_catch,
    torque_CATCH,
    tau_limit_CATCH,
    servo,
    init_euler_xyz,
    pr,
    brach,
    t0_experiment,
):
"""

async def Catch(
    idling_time,
    catch_duration,
    torque_tail_CATCH,
    torque_phi_CATCH,
    tau_limit_CATCH,
    servo_tail,
    servo_phi,
    init_euler_xyz,
    pr,
    brach,
    t0_experiment,
):
    # success = False
    success = True  # FIXME the catch condition has problem and we disabled it for now

    index_CATCH = 0
    t_CATCH = 0
    meas_dt_CATCH = 0
    # idling_time = 0.6
    idling_tau_tail = 0
    idling_tau_phi = 0
    # t < 3.65 or el_vel < 1.5: #do not break for time instead take data for n loops
    idling_start = time.time()
    tau_tail_cmd = np.clip(idling_tau_tail, -tau_limit_CATCH, tau_limit_CATCH)
    tau_phi_cmd = np.clip(idling_tau_phi, -tau_limit_CATCH, tau_limit_CATCH)
    # while index_CATCH < n_catch:
    while time.time() - idling_start < idling_time:
        start_loop = time.time()
        t_CATCH += meas_dt_CATCH
        # send zero command to motor to make sure nothing would happen
        meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi = await send_rad_command(
            controller_obj_tail=servo_tail,
            controller_obj_phi=servo_phi,
            pos_tail=0.0,  # although 0, kp = 0 gives pos no affect
            vel_tail=0.0,
            tau_tail=tau_tail_cmd,  # although 0, kd = 0 gives vel no affect
            pos_phi=0.0,  # although 0, kp = 0 gives pos no affect
            vel_phi=0.0,  # although 0, kd = 0 gives vel no affect
            tau_phi=tau_phi_cmd,
            tau_limit=tau_limit_CATCH,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=0.05,
        )
        arm1_pos, arm1_vel, raw_imu_pos, raw_imu_vel = await kb_state_estimate(
            brach, pr, meas_pos_tail, meas_vel_tail
        )
        meas_pos_arm2 = meas_pos_phi + meas_pos_tail
        meas_vel_arm2 = meas_vel_phi + meas_vel_tail

        # (el_pos, el_vel, el_tau) = await send_rad_command(
        #     controller_obj=servo,
        #     pos=0.0,  # although 0, kp = 0 gives pos no affect
        #     vel=0.0,  # although 0, kd = 0 gives vel no affect
        #     tau=idling_tau,
        #     tau_limit=tau_limit_CATCH,
        #     kp_scale=0.0,
        #     kd_scale=0.0,
        #     watchdog_timeout=0.05,
        # )

        # sh_pos, sh_vel, raw_imu_pos, raw_imu_vel = await ch_state_estimate(
        #     brach, pr, el_pos, el_vel, init_euler_xyz
        # )
        

        # store measured data
        data_append(
            phase="catch",
            index=INDEX[0],
            TIME=time.time() - t0_experiment,
            ARM1_MEAS_POS=arm1_pos,
            ARM1_MEAS_VEL=arm1_vel,
            TAIL_MEAS_POS=meas_pos_tail,
            TAIL_MEAS_VEL=meas_vel_tail,
            PHI_MEAS_POS=meas_pos_phi,
            PHI_MEAS_VEL=meas_vel_phi,
            ARM2_MEAS_POS=meas_pos_arm2,
            ARM2_MEAS_VEL=meas_vel_arm2,
            TAIL_CMD_TAU=idling_tau_tail,
            TAIL_MEAS_TAU=meas_tau_tail,
            TAIL_CLIPPED_TAU=tau_tail_cmd,
            PHI_CMD_TAU=idling_tau_phi,
            PHI_MEAS_TAU=meas_tau_phi,
            PHI_CLIPPED_TAU=tau_phi_cmd,
            RAW_IMU_POS=raw_imu_pos,
            RAW_IMU_VEL=raw_imu_vel,
            ARM2_DES_POS=np.nan,
            ARM2_DES_VEL=np.nan,
            PHI_DES_POS=np.nan,
            PHI_DES_VEL=np.nan,
            TAIL_DES_POS=np.nan,
            TAIL_DES_VEL=np.nan,
            ARM1_DES_POS=np.nan,
            ARM1_DES_VEL=np.nan,
        )
        # data_append(
        #     phase="catch",
        #     index=INDEX[0],
        #     TIME=time.time() - t0_experiment,
        #     SH_MEAS_POS=sh_pos,
        #     SH_MEAS_VEL=sh_vel,
        #     EL_MEAS_POS=el_pos,
        #     EL_MEAS_VEL=el_vel,
        #     EL_CMD_TAU=torque_CATCH,
        #     EL_MEAS_TAU=el_tau,
        #     EL_CLIPPED_TAU=torque_CATCH,
        #     RAW_IMU_POS=raw_imu_pos,
        #     RAW_IMU_VEL=raw_imu_vel,
        #     EL_DES_POS=np.nan,
        #     EL_DES_VEL=np.nan,
        #     SH_DES_POS=np.nan,
        #     SH_DES_VEL=np.nan,
        # )
        INDEX[0] += 1
        meas_dt_CATCH = time.time() - start_loop
    # print(f'catch idling time was {idling_time} seconds')
    # print(f'catch started for {catch_duration} seconds')
    catch_start = time.time()
    while time.time() - catch_start < catch_duration:
        start_loop = time.time()
        t_CATCH += meas_dt_CATCH

        tau_tail_cmd_CATCH = np.clip(
            torque_tail_CATCH, -tau_limit_CATCH, tau_limit_CATCH
        )
        tau_phi_cmd_CATCH = np.clip(
            torque_phi_CATCH, -tau_limit_CATCH, tau_limit_CATCH
        )
        # send zero command to motor to make sure nothing would happen
        meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi = await send_rad_command(
            controller_obj_tail=servo_tail,
            controller_obj_phi=servo_phi,
            pos_tail=0.0,  # although 0, kp = 0 gives pos no affect
            vel_tail=0.0,
            tau_tail=tau_tail_cmd_CATCH,  # although 0, kd = 0 gives vel no affect
            pos_phi=0.0,  # although 0, kp = 0 gives pos no affect
            vel_phi=0.0,  # although 0, kd = 0 gives vel no affect
            tau_phi=tau_phi_cmd_CATCH,
            tau_limit=tau_limit_CATCH,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=0.05,
        )
        arm1_pos, arm1_vel, raw_imu_pos, raw_imu_vel = await kb_state_estimate(
            brach, pr, meas_pos_tail, meas_vel_tail
        )
        meas_pos_arm2 = meas_pos_phi + meas_pos_tail
        meas_vel_arm2 = meas_vel_phi + meas_vel_tail
        # (el_pos, el_vel, el_tau) = await send_rad_command(
        #     controller_obj=servo,
        #     pos=0.0,
        #     vel=0.0,
        #     tau=tau_cmd_CATCH,
        #     tau_limit=tau_limit_CATCH,
        #     kp_scale=0.0,
        #     kd_scale=0.0,
        #     watchdog_timeout=0.05,
        # )

        # sh_pos, sh_vel, raw_imu_pos, raw_imu_vel = await ch_state_estimate(
        #     brach, pr, el_pos, el_vel, init_euler_xyz
        # )

        # store measured data
        data_append(
            phase="catch",
            index=INDEX[0],
            TIME=time.time() - t0_experiment,
            ARM1_MEAS_POS=arm1_pos,
            ARM1_MEAS_VEL=arm1_vel,
            TAIL_MEAS_POS=meas_pos_tail,
            TAIL_MEAS_VEL=meas_vel_tail,
            PHI_MEAS_POS=meas_pos_phi,
            PHI_MEAS_VEL=meas_vel_phi,
            ARM2_MEAS_POS=meas_pos_arm2,
            ARM2_MEAS_VEL=meas_vel_arm2,
            TAIL_CMD_TAU=torque_tail_CATCH,
            TAIL_MEAS_TAU=meas_tau_tail,
            TAIL_CLIPPED_TAU=tau_tail_cmd_CATCH,
            PHI_CMD_TAU=torque_phi_CATCH,
            PHI_MEAS_TAU=meas_tau_phi,
            PHI_CLIPPED_TAU=tau_phi_cmd_CATCH,
            RAW_IMU_POS=raw_imu_pos,
            RAW_IMU_VEL=raw_imu_vel,
            ARM2_DES_POS=np.nan,
            ARM2_DES_VEL=np.nan,
            PHI_DES_POS=np.nan,
            PHI_DES_VEL=np.nan,
            TAIL_DES_POS=np.nan,
            TAIL_DES_VEL=np.nan,
            ARM1_DES_POS=np.nan,
            ARM1_DES_VEL=np.nan,
        )
        # data_append(
        #     phase="catch",
        #     index=INDEX[0],
        #     TIME=time.time() - t0_experiment,
        #     SH_MEAS_POS=sh_pos,
        #     SH_MEAS_VEL=sh_vel,
        #     EL_MEAS_POS=el_pos,
        #     EL_MEAS_VEL=el_vel,
        #     EL_CMD_TAU=torque_CATCH,
        #     EL_MEAS_TAU=el_tau,
        #     EL_CLIPPED_TAU=tau_cmd_CATCH,
        #     RAW_IMU_POS=raw_imu_pos,
        #     RAW_IMU_VEL=raw_imu_vel,
        #     EL_DES_POS=np.nan,
        #     EL_DES_VEL=np.nan,
        #     SH_DES_POS=np.nan,
        #     SH_DES_VEL=np.nan,
        # )

        index_CATCH += 1
        INDEX[0] += 1
        meas_dt_CATCH = time.time() - start_loop
    print(f"Catch loop control frequency = {1/meas_dt_CATCH}")
    success = True  # FIXME Catch condition does not work properly, so it is disabled for now
    """
    if ch_cond(brach, el_pos):
        success = True # FIXME Catch condition does not work properly, so it is disabled for now
        print(
            "stopping motor in order to restart control, index is: ",
            index_CATCH,
        )
        await servo.set_stop()
        print("Catch success, success set to: ", str(success))
    else:
        print("Catch fail, stopping motor")
        await servo.set_stop()
        print("Catch fail, do not attempt, sucess set to : ", success)
    """
    await servo_tail.set_stop()
    await servo_phi.set_stop()
    return success


# def catch(brach, n, tau, tau_limit, servo, init_euler_xyz, imu, t0_experiment):
def catch(
    brach,
    idling_time,
    catch_duration,
    tau_tail,
    tau_phi,
    tau_limit,
    servo_tail,
    servo_phi,
    init_euler_xyz,
    imu,
    t0_experiment,
):
    tau_phi = tau_phi if brach == "odd" else -tau_phi   #ARE YOU SURE WE NEED TO DO THIS??
    success = run(
        Catch(
            # n, tau, tau_limit, servo, init_euler_xyz, imu, brach, t0_experiment
            idling_time,
            catch_duration,
            tau_tail,
            tau_phi,   
            tau_limit,
            servo_tail,
            servo_phi,
            init_euler_xyz,
            imu,
            brach,
            t0_experiment,
        )
    )
    if not success:
        """raise BrachiationFailure(
            "catch",
            brach,
            n,
            tau,
            tau_limit,
            init_euler_xyz,
            imu,
            t0_experiment,
        )
        """
        raise BrachiationFailure(
            "catch",
            brach,
            idling_time,
            catch_duration,
            tau_tail,
            tau_phi,
            tau_limit,
            init_euler_xyz,
            imu,
            t0_experiment,
        )


def swing(brach, servo_tail, servo_phi, init_euler_xyz, imu, t0, controller):#(brach, servo, init_euler_xyz, imu, t0_experiment, controller):
    meas_dt = 0
    sign = 1 if brach == "odd" else -1
    t = 0
    pr = imu
    swing_tau_limit = 3
    arm1_pos = NOMINAL_PCWS.arm1_des_Ang_pos_pcw.get_value(t)
    arm1_vel = NOMINAL_PCWS.arm1_des_Ang_vel_pcw.get_value(t)
    tail_pos = NOMINAL_PCWS.tail_des_Ang_pos_pcw.get_value(t)
    tail_vel = NOMINAL_PCWS.tail_des_Ang_vel_pcw.get_value(t)    
    phi_pos = NOMINAL_PCWS.phi_des_Ang_pos_pcw.get_value(t)
    phi_vel = NOMINAL_PCWS.phi_des_Ang_vel_pcw.get_value(t)
    arm2_pos = NOMINAL_PCWS.arm2_des_Ang_pos_pcw.get_value(t)
    arm2_vel = NOMINAL_PCWS.arm2_des_Ang_vel_pcw.get_value(t)
    tail_tau = NOMINAL_PCWS.tail_des_torque_tvlqr_pcw.get_value(t)
    phi_tau = NOMINAL_PCWS.phi_des_torque_tvlqr_pcw.get_value(t)
    
    meas_arm1_pos = arm1_pos
    meas_arm1_vel = arm1_vel
    meas_pos_tail = tail_pos
    meas_vel_tail = tail_vel
    meas_tau_tail = tail_tau
    meas_pos_phi = phi_pos
    meas_vel_phi = phi_vel
    meas_tau_phi = phi_tau
    meas_state = np.array([[arm1_pos], [tail_pos], [phi_pos], [arm1_vel], [tail_vel], [phi_vel]])
    meas_dt = 0
    print(f"Swing start index: {INDEX[0]}")
    while t < DATA_DES.des_time[-1]:
        t_iteration = time.time()
        t += meas_dt
        index = INDEX[0]
        #meas_state = np.array([[arm1_pos], [tail_pos], [phi_pos], [arm1_vel], [tail_vel], [phi_vel]])
        if t > DATA_DES.des_time[-1]:
            print("time=", t)
            break
        arm1_des_pos = NOMINAL_PCWS.arm1_des_Ang_pos_pcw.get_value(t)
        arm1_des_vel = NOMINAL_PCWS.arm1_des_Ang_vel_pcw.get_value(t)
        tail_des_pos = NOMINAL_PCWS.tail_des_Ang_pos_pcw.get_value(t)
        tail_des_vel = NOMINAL_PCWS.tail_des_Ang_vel_pcw.get_value(t)    
        phi_des_pos = sign * NOMINAL_PCWS.phi_des_Ang_pos_pcw.get_value(t)   #Not sure if this sign thing is true. But seems true. During multiple brachiations, the roles of the arms switch
        phi_des_vel = sign * NOMINAL_PCWS.phi_des_Ang_vel_pcw.get_value(t)
        arm2_des_pos = NOMINAL_PCWS.arm2_des_Ang_pos_pcw.get_value(t)
        arm2_des_vel = NOMINAL_PCWS.arm2_des_Ang_vel_pcw.get_value(t)
        tail_des_tau = NOMINAL_PCWS.tail_des_torque_tvlqr_pcw.get_value(t)
        phi_des_tau = NOMINAL_PCWS.phi_des_torque_tvlqr_pcw.get_value(t)
        
        
        des_state = np.array([[arm1_des_pos], [tail_des_pos], [phi_des_pos], [arm1_des_vel], [tail_des_vel], [phi_des_vel]])

        motor_params = motor_test_parameters(
            test=TEST,
            nominal_pcws=NOMINAL_PCWS,
            brach_sign=sign,
            meas_state=meas_state,
            des_state=des_state,
            time=t,
            controller=controller,
        )
        clipped_tau_tail = sign * np.clip(
            motor_params.tau_tail_cmd, -swing_tau_limit, swing_tau_limit
        )
        clipped_tau_phi = sign * np.clip(
            motor_params.tau_phi_cmd, -swing_tau_limit, swing_tau_limit
        )
        if TEST == "tvlqr":
            clipped_tau_tail = np.clip(motor_params.tau_tail_cmd, -swing_tau_limit, swing_tau_limit)
            clipped_tau_phi = np.clip(motor_params.tau_phi_cmd, -swing_tau_limit, swing_tau_limit)

        #(meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi) = run(
        #    send_rad_command(
        #        controller_obj_tail=servo_tail,
        #        controller_obj_phi=servo_phi, 
        #        pos_tail=tail_des_pos, 
        #        vel_tail=tail_des_vel,
        #        tau_tail=clipped_tau_tail, 
        #        pos_phi=phi_des_pos,  
        #        vel_phi=phi_des_vel, 
        #        tau_phi=clipped_tau_phi,
        #        tau_limit=TAU_LIMIT,
        #        kp_scale=motor_params.kp_scale,
        #        kd_scale=motor_params.kd_scale,
        #        watchdog_timeout=0.05,
        #    )
        #)
        #DID YOU CHANGE THE 'SEND_RAD_COMMAND' FUNCTION TO 'SEND_RAD_COMMAND_W_CHECK' IN KICKBACK AND CATCH FUNCTIONS
        (meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi) = run(
            send_rad_command_w_check(
                controller_obj_tail=servo_tail,
                controller_obj_phi=servo_phi, 
                pos_tail=tail_des_pos, 
                vel_tail=tail_des_vel,
                tau_tail=clipped_tau_tail, 
                pos_phi=phi_des_pos,  
                vel_phi=phi_des_vel, 
                tau_phi=clipped_tau_phi,
                tau_limit=TAU_LIMIT,
                kp_scale=motor_params.kp_scale,
                kd_scale=motor_params.kd_scale,
                watchdog_timeout=0.05,
                prev_meas_pos_tail=meas_pos_tail,
                prev_meas_vel_tail=meas_vel_tail,
                prev_meas_tau_tail=meas_tau_tail,
                prev_meas_pos_phi=meas_pos_phi,
                prev_meas_vel_phi=meas_vel_phi,
                prev_meas_tau_phi=meas_tau_phi,
                index=index
            )
        )
        arm2_des_pos = phi_des_pos + tail_des_pos
        arm2_des_vel = phi_des_vel + tail_des_vel
        meas_pos_arm2 = meas_pos_phi + meas_pos_tail
        meas_vel_arm2 = meas_vel_phi + meas_vel_tail

        if brach == "odd":
            arm1_pos, arm1_vel, raw_imu_pos, raw_imu_vel = run(state_estimation(
                pr=pr, pos_tail=meas_pos_tail, vel_tail=meas_vel_tail
            ))

        else:
            #state = run(state_estimation_v2(pr=imu))  #In RicMonk, there's no need for two different ways of state estimation.
            arm1_pos, arm1_vel, raw_imu_pos, raw_imu_vel = run(state_estimation(
                pr=pr, pos_tail=meas_pos_tail, vel_tail=meas_vel_tail
            ))
        #(arm1_pos, arm1_vel, raw_imu_pos, raw_imu_vel) = state
        meas_state = np.array([[arm1_pos], [meas_pos_tail], [meas_pos_phi], [arm1_vel], [meas_vel_tail], [meas_vel_phi]])
        K = motor_params.K_tvlqr
        #print(f'Swing index {INDEX[0]} over')

        # store measured data
        data_append(
            phase="swing",
            index=INDEX[0],
            TIME=time.time() - t0,
            ARM1_MEAS_POS=arm1_pos,
            ARM1_MEAS_VEL=arm1_vel,
            TAIL_MEAS_POS=meas_pos_tail,
            TAIL_MEAS_VEL=meas_vel_tail,
            PHI_MEAS_POS=meas_pos_phi,
            PHI_MEAS_VEL=meas_vel_phi,
            TAIL_CMD_TAU=motor_params.tau_tail_cmd,
            TAIL_MEAS_TAU=meas_tau_tail,
            TAIL_CLIPPED_TAU=clipped_tau_tail,
            TAIL_DES_TAU=tail_des_tau,
            PHI_CMD_TAU=motor_params.tau_phi_cmd,
            PHI_MEAS_TAU=meas_tau_phi,
            PHI_CLIPPED_TAU=clipped_tau_phi,
            PHI_DES_TAU=phi_des_tau,
            RAW_IMU_POS=raw_imu_pos,
            RAW_IMU_VEL=raw_imu_vel,
            ARM2_DES_POS=arm2_des_pos,
            ARM2_DES_VEL=arm2_des_vel,
            PHI_DES_POS=phi_des_pos,
            PHI_DES_VEL=phi_des_vel,
            TAIL_DES_POS=tail_des_pos,
            TAIL_DES_VEL=tail_des_vel,
            ARM1_DES_POS=arm1_des_pos,
            ARM1_DES_VEL=arm1_des_vel,
            ARM2_MEAS_POS=meas_pos_arm2,
            ARM2_MEAS_VEL=meas_vel_arm2,
            TVLQR_K1=K[0][0],
            TVLQR_K2=K[0][1],
            TVLQR_K3=K[0][2],
            TVLQR_K4=K[0][3],
            TVLQR_K5=K[0][4],
            TVLQR_K6=K[0][5],
            TVLQR_K7=K[1][0],
            TVLQR_K8=K[1][1],
            TVLQR_K9=K[1][2],
            TVLQR_K10=K[1][3],
            TVLQR_K11=K[1][4],
            TVLQR_K12=K[1][5],
        )
        INDEX[0] += 1
        meas_dt = time.time() - t_iteration
    print(f"Swing end index: {INDEX[0]}")
    print(f"swing loop control frequency = {1/meas_dt}")


def brachiation(
    loop, ibrach, brach, t0, servo_tail, servo_phi, imu, init_euler_xyz, controller
):
	if TRAJ == 'BF':
	    print(
	        f"[brachiation] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	    )

	    print(
	        f"[kickback] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	    )
	    kickback("odd", 49, 4.3, 2.2, 6.0, servo_tail, servo_phi, init_euler_xyz, imu, t0)   #PASSED "odd" DIRECTLY AS AN ARGUMENT INSTEAD OF VARIABLE 'brach'
	    # loop.exit()
	    print(
	        f"[swing] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	    )
	    swing(brach, servo_tail, servo_phi, init_euler_xyz, imu, t0, controller)#(brach, servo, init_euler_xyz, imu, t0, controller)
	    # loop.exit()
	    #print(
	    #    f"[catch] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	    #)
	    #t_catch_start = time.time()
	    # catch(brach, 20, 0.8, 1.2, servo, init_euler_xyz, imu, t0)
	    #catch(
	    #    brach=brach,
	    #    idling_time=0.0,
	    #    catch_duration=0.0,#0.2 automatic detach of battry swing
	    #    tau_tail=0.0,
	    #    tau_phi=-0.0,
	    #    tau_limit=0,
	    #    servo_tail=servo_tail, 
	    #    servo_phi=servo_phi,
	    #    init_euler_xyz=init_euler_xyz,
	    #    imu=imu,
	    #    t0_experiment=t0,
	    #)

	    time.sleep(0.2)

	elif TRAJ == 'FB':
	    print(
	        f"[brachiation] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	    )

	    print(
	        f"[kickback] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	    )
	    kickback("odd", 49, 2.17, 6.5, 10.0, servo_tail, servo_phi, init_euler_xyz, imu, t0)    #PASSED "odd" DIRECTLY AS AN ARGUMENT INSTEAD OF VARIABLE 'brach'
	    # loop.exit()
	    print(
	        f"[swing] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	    )
	    swing(brach, servo_tail, servo_phi, init_euler_xyz, imu, t0, controller)#(brach, servo, init_euler_xyz, imu, t0, controller)
	    # loop.exit()
	    #print(
	    #    f"[catch] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	    #)
	    t_catch_start = time.time()
	    # catch(brach, 20, 0.8, 1.2, servo, init_euler_xyz, imu, t0)
	    #brach, n, torque_tail, torque_phi, tau_limit, servo_tail, servo_phi, init_euler_xyz, imu, t0_experiment
	    #catch(
	    #    brach=brach,
	    #    idling_time=0.0,
	    #    catch_duration=0.0,#0.2 automatic detach of battry swing
	    #    tau_tail=0.0,
	    #   tau_phi=0.0,
	    #    tau_limit=0.0,
	    #    servo_tail=servo_tail, 
	    #    servo_phi=servo_phi,
	    #    init_euler_xyz=init_euler_xyz,
	    #    imu=imu,
	    #    t0_experiment=t0,
	    #)

	    time.sleep(0.2)

	elif TRAJ == 'ZB':
		#NO KICKBACK REQUIREMENT IN ZB TRAJAECTORY
		# print(
	 #        f"[brachiation] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	 #    )

	 #    print(
	 #        f"[kickback] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	 #    )
	 #    kickback(brach, 49, 0, 2.5, 6.0, servo_tail, servo_phi, init_euler_xyz, imu, t0)
	 #    # loop.exit()
	    print(
	        f"[swing] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	    )
	    swing(brach, servo_tail, servo_phi, init_euler_xyz, imu, t0, controller)#(brach, servo, init_euler_xyz, imu, t0, controller)
	    # loop.exit()
	    print(
	        f"[catch] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	    )
	    t_catch_start = time.time()
	    # catch(brach, 20, 0.8, 1.2, servo, init_euler_xyz, imu, t0)
	    #brach, n, torque_tail, torque_phi, tau_limit, servo_tail, servo_phi, init_euler_xyz, imu, t0_experiment
	    #catch(
	    #    brach=brach,
	    #    idling_time=0.3,
	    #    catch_duration=0.3,#0.2 automatic detach of battry swing
	    #    tau_tail=-0.0,
	    #    tau_phi=-1.5,
	    #    tau_limit=6,
	    #    servo_tail=servo_tail, 
	    #    servo_phi=servo_phi,
	    #    init_euler_xyz=init_euler_xyz,
	    #    imu=imu,
	    #    t0_experiment=t0,
	    #)

	    time.sleep(0.2)

	elif TRAJ == 'ZF':
		#NO KICKBACK REQUIREMENT IN ZF TRAJECTORY
		# print(
	 #        f"[brachiation] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	 #    )

	 #    print(
	 #        f"[kickback] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	 #    )
	 #    kickback(brach, 49, 0, 2.5, 6.0, servo_tail, servo_phi, init_euler_xyz, imu, t0)
	 #    # loop.exit()
	    print(
	        f"[swing] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	    )
	    swing(brach, servo_tail, servo_phi, init_euler_xyz, imu, t0, controller)#(brach, servo, init_euler_xyz, imu, t0, controller)
	    # loop.exit()
	    print(
	        f"[catch] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time()-t0}"
	    )
	    t_catch_start = time.time()
	    # catch(brach, 20, 0.8, 1.2, servo, init_euler_xyz, imu, t0)
	    #brach, n, torque_tail, torque_phi, tau_limit, servo_tail, servo_phi, init_euler_xyz, imu, t0_experiment
	    #catch(
	    #    brach=brach,
	    #    idling_time=0.2,
	    #    catch_duration=0.1,#0.2 automatic detach of battry swing
	    #    tau_tail=0.0,
	    #    tau_phi=-1.2,
	    #    tau_limit=1.5,
	    #    servo_tail=servo_tail, 
	    #    servo_phi=servo_phi,
	    #    init_euler_xyz=init_euler_xyz,
	    #    imu=imu,
	    #    t0_experiment=t0,
	    #)

	    time.sleep(0.2)


def imu_reset(con, sleep_time=3):
    moteus_pi3hat.Pi3HatRouter(mounting_deg={"roll": 0, "pitch": 0, "yaw": 0})
    imu = moteus_pi3hat.Pi3HatRouter(
        mounting_deg={"roll": con[0], "pitch": con[1], "yaw": con[2]}
    )
    time.sleep(sleep_time)
    return imu


async def init():
    transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map={BUS_NUMBER:MOTOR_ID})
    servo_tail = moteus.Controller(id=MOTOR_ID[1], transport=transport)
    servo_phi = moteus.Controller(id=MOTOR_ID[0], transport=transport)
    imu = imu_reset(IMU_CONFIG_ODD)  # imu_reset(IMU_CONFIG_EVEN)#
    _, _, _, init_euler_xyz = await read_imu_data(imu)
    await servo_tail.set_stop()
    await servo_phi.set_stop()
    if TEST == "rl":
        print("Loading RL controller")
        controller = get_controller()
    else:
        controller = None
    return servo_tail, servo_phi, imu, init_euler_xyz, controller


def main(loop):
    nbrachiation = int(os.getenv("NBRACH", "1"))
    try:
        for ibrachiation in range(
            1, nbrachiation + 1
        ):  # To test one brach with IMU arm attach, set range 2
            print(f"[main] ibrachiation:{ibrachiation}")

            brach = "even" if ibrachiation % 2 == 0 else "odd"
            print(f"Brach: {brach}")

            if (
                ibrachiation == 1
            ):  # To test one brach with IMU arm attach, set ibrachiation == 2
                (
                    servo_tail,
                    servo_phi,
                    imu,
                    init_euler_xyz,
                    controller,
                ) = loop.run_until_complete(init())
                if input("Did you set the robot to zero(Z) configuration before running the code? If yes, did you also set the robot to the required initial configuration? If yes, Start? (y/n) ").lower() != "y":
                    exit(1)
                t0 = time.time()
            brachiation(
                loop,
                ibrachiation,
                brach,
                t0,
                servo_tail,
                servo_phi,
                imu,
                init_euler_xyz,
                controller,
            )
    # except Exception as ex:
    # print("[main] unhanlded exception:", ex)
    finally:
        if TEST == "pd":
            DATA_full = calc_pd_tau_ricMonk(DATA, tau_limit = 6)
        else:
            DATA_full = DATA
        index = INDEX[0]
        valid_data = {
            field: DATA_full[i][:index]
            for i, field in enumerate(DATA_full._fields)
            if isinstance(DATA_full[i], np.ndarray)
        }
        # Disabling the motor
        run(servo_tail.set_stop())
        run(servo_phi.set_stop())
        print("Motor disabled.")
        
        #answer='n'
        answer = input("Should I save results?")
        if answer == 'y' or answer == 'Y':
            directory = make_results_directory_test_traj(TEST, TRAJ, brach)
            # Save Trajectory to a csv file to be sent to the motor.
            save_data(valid_data, directory + "/measured.csv")
            plot_custom_data_with_dir_meas(directory, valid_data, show=True)
            
print("DID YOU CHANGE THE 'SEND_RAD_COMMAND' FUNCTION TO 'SEND_RAD_COMMAND_W_CHECK' IN KICKBACK AND CATCH FUNCTIONS")
zero_offset_two_motors(BUS_NUMBER, MOTOR_ID)
#if input("Press enter"):
#    exit()
#else:
#    exit()
loop = asyncio.get_event_loop()
main(loop)
