import asyncio
import moteus
import moteus_pi3hat
## documentation https://github.com/mjbots/moteus/tree/main/lib/python
import numpy as np
import time
import math
import matplotlib.pyplot as plt
from datetime import datetime
import os
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
                   state_estimation_even,
                   read_two_motors_data_w_check
                   )

BUS_NUMBER = 2
MOTOR_ID = [8,9]    

IMU_CONFIG_ODD = (0, 90, 180)

def prepare_data():
    # create empty numpy array so that the measured sensor data can later be stored in this array
    n = 5000
    theta_1 = np.zeros(n)
    theta_2 = np.zeros(n)
    phi = np.zeros(n)
    meas_time = np.zeros(n)
    return theta_1, theta_2, phi, n, meas_time


def plot_data(meas_time, theta_1, theta_2, phi, break_times):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p") 
    print("Making data plots.")
    plt.figure()
    plt.plot(meas_time, theta_1, label='theta_1')
    plt.plot(meas_time, theta_2, label='theta_2')
    plt.plot(meas_time, phi, label='phi')
    
    for time in break_times:
        plt.axvline(x=time, color = 'black', linestyle = '--')
    
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.title("Position (rad) vs Time (s)")
    plt.legend()
    plt.draw()
    plt.savefig(f'{date}_pos.pdf')
    plt.show()
    return date

def save_data(date, meas_pos, meas_vel, meas_tau,meas_time,meas_imu_roll):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
    print("Saving data to .csv files.")
    measured_csv_data = np.array([np.array(meas_time),
                                  np.array(meas_pos),
                                  np.array(meas_vel),
                                  np.array(meas_tau),
                                  np.array(meas_imu_roll)]).T
    np.savetxt(f'{date}_measured.csv',
               measured_csv_data,
               delimiter=',',
               header="time,position,velocity,torque,meas_imu_roll",
               comments="")

def imu_reset(con, sleep_time=3):
    moteus_pi3hat.Pi3HatRouter(mounting_deg={"roll": 0, "pitch": 0, "yaw": 0})
    imu = moteus_pi3hat.Pi3HatRouter(
        mounting_deg={"roll": con[0], "pitch": con[1], "yaw": con[2]}
    )
    time.sleep(sleep_time)
    return imu

async def main(brach, imu, t, meas_dt, afterOdd_finTailPos, afterOdd_finPhiPos, i):
    if brach == 'odd':
        servo_tail = servo_9
        servo_phi = servo_8
    elif brach == 'even':
        servo_tail = servo_8
        servo_phi = servo_9
    
    print("Motors Enabled.")
    
    
    await servo_tail.set_stop()
    await servo_phi.set_stop()
    
    #if input("All set!?") != 'y':
    #    print("Alright, do it again")
    #    return 0

    index = 0
    #t = 0
    #meas_dt = 0
    start_time = time.time()
    n = 3000
    
    meas_pos_tail = 0
    meas_pos_phi = 0

    try:
        while index < n:
            start_loop = time.time()
            
            # read the data from the motor
            #print('here')
            #rint(f'here too __ {index}')
            meas_pos_tail, meas_vel_tail, _, meas_pos_phi, _, _ = await read_two_motors_data_w_check(
                controller_obj_tail=servo_tail,
                controller_obj_phi=servo_phi,                
				prev_meas_pos_tail=meas_pos_tail,
				prev_meas_vel_tail=0,
				prev_meas_tau_tail=0,
				prev_meas_pos_phi=meas_pos_phi,
				prev_meas_vel_phi=0,
				prev_meas_tau_phi=0,)
            #print(f'here too {index}')
            
            if brach == "odd":
                meas_pos_tail = meas_pos_tail + ((i/2)*2*math.pi)
                meas_pos_phi = meas_pos_phi - ((i/2)*2*math.pi)
                meas_theta_1, meas_vel_arm1, raw_imu_pos, raw_imu_vel = await state_estimation(
                    pr=imu, pos_tail=meas_pos_tail, vel_tail=meas_vel_tail
                )
                afterOdd_finTailPos = meas_pos_tail
                afterOdd_finPhiPos = meas_pos_phi

            else:
                meas_pos_tail = meas_pos_tail - afterOdd_finPhiPos + afterOdd_finTailPos
                meas_pos_phi = meas_pos_phi - afterOdd_finTailPos + afterOdd_finPhiPos
                meas_theta_1, meas_vel_arm1, raw_imu_pos, raw_imu_vel = await state_estimation_even(
                    pr=imu, pos_tail=meas_pos_tail, vel_tail=meas_vel_tail
                )
            #print(f'here too too {index}')
            # store data
            
            theta_1.append(meas_theta_1)
            theta_2.append(meas_pos_tail)
            phi.append(meas_pos_phi)
            meas_dt = time.time() - start_loop
            t += meas_dt
            meas_time.append(t)
            index += 1
        
        break_times.append(t)
        print("Done with one trajectory")
        print("Starting in 3 seconds")
        await asyncio.sleep(3)

    except BaseException as e:
        print("**Exception Block**")
        print(f"Error= {e}")
        await servo_tail.set_stop()
        await servo_phi.set_stop()
        #date = plot_data(meas_time, theta_1, theta_2, phi)
        #save_data(date, theta_1, theta_2, phi, meas_time)

    finally:
        # Disabling the motor
        await servo_tail.set_stop()
        await servo_phi.set_stop()
        print("Motor disabled.")

        return meas_time, theta_1, theta_2, phi, break_times, t, meas_dt, afterOdd_finTailPos, afterOdd_finPhiPos
        
theta_1 = []
theta_2 = []
phi = []
meas_time = []
break_times = []

if __name__ == '__main__':
    #theta_1, theta_2, phi, n, meas_time = prepare_data()
    zero_offset_two_motors(BUS_NUMBER, MOTOR_ID)
    transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map={BUS_NUMBER:MOTOR_ID})
    servo_9 = moteus.Controller(id=MOTOR_ID[1], transport=transport)
    servo_8 = moteus.Controller(id=MOTOR_ID[0], transport=transport)
    
    imu = imu_reset(IMU_CONFIG_ODD)  # imu_reset(IMU_CONFIG_EVEN)#
    #_, _, _, init_euler_xyz = await read_imu_data(imu)
    
    #await servo_9.set_stop()
    #await servo_8.set_stop()
    
    t=0
    meas_dt = 0
    afterOdd_finTailPos = 0
    afterOdd_finPhiPos = 0
    
    if(input("Continue (y/n)") == 'y'):
        for ibrachiation in range(3):
            brach = "odd" if ibrachiation % 2 == 0 else "even"
            print()
            print("******************")
            print(f"Current brachiation: {brach}")
            (meas_time, theta_1, theta_2, phi, break_times, new_t, meas_dt, finTailPos, finPhiPos) = asyncio.run(main(brach, imu, t, meas_dt, afterOdd_finTailPos, afterOdd_finPhiPos, ibrachiation))
            t = new_t
            afterOdd_finTailPos = finTailPos #theta_2_9
            afterOdd_finPhiPos = finPhiPos #phi_8
            
        date = plot_data(meas_time, theta_1, theta_2, phi, break_times)    
    else:
        print("Tata!")




