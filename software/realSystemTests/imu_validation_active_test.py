import asyncio
import moteus
import moteus_pi3hat
## documentation https://github.com/mjbots/moteus/tree/main/lib/python
import time
import os
from datetime import datetime
import math
import numpy as np
import matplotlib.pyplot as plt
# Local imports
from utils import (plot_data, 
                   generate_path, 
                   read_data, 
                   prepare_des_data,
                   save_data, 
                   zero_offset, 
                   send_rad_command,
                   get_theta1,
                   read_imu_data,
                   state_estimation,   
                   prepare_empty_arrays,
                   send_rad_command_one_motor)
import utils

def plot_custom_data(meas_time,
                     arm1_meas_Ang_pos,
                     tail_meas_Ang_pos,
                     imu_theta1,
                     raw_imu_pos,                     
                     arm1_meas_Ang_vel,
                     tail_meas_Ang_vel,
                     imu_theta1_dot,
                     raw_imu_vel,
                     arm1_meas_Ang_tau,
                     tail_meas_tau):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
    folder_name = 'results/imu_validation/active_test'
    file_name = date
    directory = generate_path(folder_name, file_name, 1)
    os.makedirs(directory)
    # shoulder
    plt1=\
    plot_data(x_data=[meas_time, 
                      meas_time],
                y_data=[arm1_meas_Ang_pos, 
                        imu_theta1],
              labels=["Time (s)", "Theta1 (rad)"],
              title="Theta1 (rad) vs Time (s)",
              legends=['theta_1_measured', 'imu_theta_1'],
              save_name=directory + '/pos.pdf')
    plt2=\
    plot_data(x_data=[meas_time, 
                      meas_time],
                y_data=[arm1_meas_Ang_vel, 
                        imu_theta1_dot],
              labels=["Time (s)", "Theta1_dot (rad/s)"],
              title="Theta1Dot (rad/s) vs Time (s)",
              legends=['theta_1_dot_measured', 'imu_theta1_dot'],
              save_name=directory + '/vel.pdf')
    plt3=\
    plot_data(x_data=[meas_time, 
                      meas_time],
                y_data=[raw_imu_pos, 
                        tail_meas_Ang_pos],
              labels=["Time (s)", "Theta1 (rad)"],
              title="Theta1 (rad) vs Time (s)",
              legends=['raw_imu_pos', 'tail_meas_Ang_pos'],
              save_name=directory + '/pos1.pdf')
    plt4=\
    plot_data(x_data=[meas_time, 
                      meas_time],
                y_data=[raw_imu_vel, 
                        tail_meas_Ang_vel],
              labels=["Time (s)", "Theta1_dot (rad/s)"],
              title="Theta1Dot (rad/s) vs Time (s)",
              legends=['raw_imu_vel', 'tail_meas_Ang_vel'],
              save_name=directory + '/vel1.pdf')
    plt5=\
    plot_data(x_data=[meas_time, 
                      meas_time],
              y_data=[arm1_meas_Ang_tau, 
                      tail_meas_tau],
              labels=["Time (s)", "Torque (Nm)"],
              title="Torque Plot",
              legends=['arm1_meas_Ang_tau', 'tail_meas_tau'],
              save_name=directory + '/tau1.pdf')
    # plt6=\
    # plot_data(x_data=[meas_time, 
    #                   meas_time],
    #           y_data=[tail_des_tau, 
    #                   tail_meas_tau],
    #           labels=["Time (s)", "Torque (Nm)"],
    #           title="Torque Plot",
    #           legends=['tail_des_tau', 'tail_meas_tau'],
    #           save_name=directory + '/tau1.pdf')
    plt.show()        
    return date, directory

    
           
async def main():
    print("Motor Enabled.")   
    if len(bus_number) == 1:
        transport = moteus_pi3hat.Pi3HatRouter(
            servo_bus_map={
                bus_number[0]: motor_id,
            },
        )
    else:
        transport = moteus_pi3hat.Pi3HatRouter(
            servo_bus_map={           
                 bus_number:motor_id,
            },
        )
    moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll': 0,
                                             'pitch': 0,
                                             'yaw': 0})   
    # create a moteus_pi3hat.Pi3HatRouter for IMU reading
    pr = moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll': 180,
                                                  'pitch': 0,
                                                  'yaw': 0})
    # wait until convergence of IMU reading
    time.sleep(5)

    _, _, _, init_euler_xyz = await read_imu_data(pr)    
    # create a moteus.Controller object for interaction with controller
    c_theta2 = moteus.Controller(id=motor_id[0], transport=transport)
    c_theta1 = moteus.Controller(id=motor_id[1], transport=transport)
    await c_theta1.set_stop()
    await c_theta2.set_stop()
    index = 0
    t = 0
    meas_dt = 0
    tau_limit = 6
    start = time.time()
    try:
        while index < n:
            start_loop = time.time()
            t += meas_dt
            # send commands to motor in radians

            (arm1_meas_Ang_pos[index], 
            arm1_meas_Ang_vel[index], 
            arm1_meas_Ang_tau[index]) = await send_rad_command_one_motor(controller_obj=c_theta1,
                                                             pos=arm1_des_Ang_pos[index],#0,#
                                                             vel=arm1_des_Ang_vel[index],#0,#
                                                             tau_limit=tau_limit)
            (tail_meas_Ang_pos[index], 
             tail_meas_Ang_vel[index], 
             tail_meas_tau[index]) = await send_rad_command_one_motor(controller_obj=c_theta2,
                                                             pos=tail_des_Ang_pos[index],#0,#
                                                             vel=tail_des_Ang_vel[index],#0,#
                                                             tau_limit=tau_limit)                                    
            # estimate theta1, theta1_dot
            (imu_theta1[index], 
             imu_theta1_dot[index], 
             raw_imu_pos[index], 
             raw_imu_vel[index]) = await state_estimation(pr, tail_meas_Ang_pos[index], tail_meas_Ang_vel[index])

            meas_time[index] = t
            index += 1
            #while time.time() - start_loop < dt:
            #    pass
            meas_dt = time.time() - start_loop
        duration = time.time() - start
            

    except BaseException as e:
        print("**Exception Block**")      
        print(f"Error= {e}")

    finally:
        #print(f'freq = {1 / (duration / n):.2f} HZ')
        date, directory = plot_custom_data(meas_time[:index],
                                           arm1_meas_Ang_pos[:index],
                                           tail_meas_Ang_pos[:index],
                                           imu_theta1[:index],
                                           raw_imu_pos[:index],
                                           arm1_meas_Ang_vel[:index],
                                           tail_meas_Ang_vel[:index],
                                           imu_theta1_dot[:index],
                                           raw_imu_vel[:index],
                                           arm1_meas_Ang_tau[:index],
                                           tail_meas_tau[:index]
                                           )
      # Save Trajectory to a csv file to be sent to the motor.
        data = {"time": meas_time[:index],
                "arm1_meas_Ang_pos": arm1_meas_Ang_pos[:index],
                "tail_meas_Ang_pos": tail_meas_Ang_pos[:index],
                "imu_theta1": imu_theta1[:index],
                "raw_imu_pos": raw_imu_pos[:index],
                "arm1_meas_Ang_vel": arm1_meas_Ang_vel[:index],
                "tail_meas_Ang_vel": tail_meas_Ang_vel[:index],
                "imu_theta1_dot": imu_theta1_dot[:index],
                "raw_imu_vel": raw_imu_vel[:index],
                "arm1_meas_Ang_tau": arm1_meas_Ang_tau[:index],
                "tail_meas_tau": tail_meas_tau[:index]}
                # "tail_des_tau": tail_des_torque[:index]} 
        save_data(data, directory+'/measured.csv')
        # Disabling the motor
        await c_theta1.set_stop()
        await c_theta2.set_stop()
        print("Motor disabled.")

if __name__ == '__main__':
    folder_name = 'data/trajectories/closed_loop'
    file_name = 'BF_pid.csv'
    # motor id and bus number mapping
    bus_number = [2]
    motor_id = [8, 9]    
    # zero offset the motors
    zero_offset(bus_number[0], motor_id[0])
    zero_offset(bus_number[0], motor_id[1])
    csv_data= read_data(folder=folder_name,
                        file=file_name,
                        up_directory=1)

    data = prepare_des_data(csv_data, "pid")

    arm1_des_Ang_pos = data.arm1_des_Ang_pos
    arm1_des_Ang_vel = data.arm1_des_Ang_vel
    tail_des_Ang_pos = data.tail_des_Ang_pos
    tail_des_Ang_vel = data.tail_des_Ang_vel
    tail_des_torque = data.tail_des_torque_tvlqr

    n = len(arm1_des_Ang_pos) 
    (arm1_meas_Ang_pos,
    arm1_meas_Ang_vel,
    tail_meas_Ang_pos,
    tail_meas_Ang_vel,
    tail_meas_tau,
    tail_cmd_tau,
    tail_clipped_tau,
    arm2_meas_Ang_pos,
    arm2_meas_Ang_vel,
    arm2_meas_tau,
    arm2_cmd_tau,
    arm2_clipped_tau,
    raw_imu_pos,
    raw_imu_vel,
    meas_time) = prepare_empty_arrays(n)

    (imu_theta1,
    imu_theta1_dot,
    arm1_meas_Ang_tau,
    _,
    _,
    _,
    _,
    _,
    _,
    _,
    _,
    _,
    _,
    _,
    _) = prepare_empty_arrays(n) 

    if input('continue with real time experiment?') == 'y':
        asyncio.run(main())
