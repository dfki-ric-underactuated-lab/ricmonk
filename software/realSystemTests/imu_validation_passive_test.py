import os
import asyncio
import moteus
import moteus_pi3hat
from utils import (prepare_empty_arrays, 
                   rad2rev, 
                   rev2rad, 
                   read_data, 
                   wrap_z2pi,
                   zero_offset,
                   save_data,
                   generate_path, 
                   plot_data, 
                   send_rad_command,
                   read_motor_data,
                   read_imu_data,
                   state_estimation,
                   quaternion_to_euler)
import time
import numpy as np
from datetime import datetime
import math
import matplotlib.pyplot as plt

def plot_custom_data(meas_time,
                     arm1_meas_Ang_pos,
                     tail_meas_Ang_pos,
                     imu_theta1,
                     raw_imu_pos,                     
                     arm1_meas_Ang_vel,
                     tail_meas_Ang_vel,
                     imu_theta1_dot,
                     raw_imu_vel):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
    folder_name = 'results/imu_validation/passive_test'
    file_name = date
    directory = generate_path(folder_name, file_name, 1)
    os.makedirs(directory)
    # shoulder
#    plt1=\
#    plot_data(x_data=[meas_time, 
#                      meas_time,
#                      meas_time,
#                      meas_time],
#                y_data=[arm1_meas_Ang_pos, 
#                        imu_theta1,
#                        raw_imu_pos,
#                        tail_meas_Ang_pos],
#              labels=["Time (s)", "Theta1 (rad)"],
#              title="Theta1 (rad) vs Time (s)",
#              legends=['theta_1_measured', 'imu_theta_1', 'raw_imu_pos', 'theta_2_measured'],
#              save_name=directory + '/pos.pdf')
#    plt2=\
#    plot_data(x_data=[meas_time, 
#                      meas_time,
#                      meas_time,
#                      meas_time],
#                y_data=[arm1_meas_Ang_vel, 
#                        imu_theta1_dot,
#                        raw_imu_vel,
#                        tail_meas_Ang_vel],
#              labels=["Time (s)", "Theta1_dot (rad/s)"],
#              title="Theta1 (rad/s) vs Time (s)",
#              legends=['theta_1_dot_measured', 'imu_theta1_dot', 'raw_imu_vel', 'theta_2_dot_measured'],
#              save_name=directory + '/vel.pdf') 
              
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
    # reset IMU
    moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll': 0,
                                             'pitch': 0,
                                             'yaw': 0})   
    # create a moteus_pi3hat.Pi3HatRouter for IMU reading
    pr = moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll': 0,
                                                  'pitch': -90,
                                                  'yaw': 0})   
    # wait until convergence of IMU reading
    time.sleep(5)
    # create a moteus.Controller object for interaction with controller
    c_theta2 = moteus.Controller(id=motor_id[0], transport=transport)  #phi _ODD
    c_theta1 = moteus.Controller(id=motor_id[1], transport=transport)  #tail _ODD
    await c_theta1.set_stop()
    await c_theta2.set_stop()
    index = 0
    t = 0
    meas_dt = 0
    tau_limit = 2.5   
    try:     
        while index < n:
            start_loop = time.time()        
            t += meas_dt       
            # send commands to motor in radians
            (arm1_meas_Ang_pos[index], 
            arm1_meas_Ang_vel[index], 
            _) = await read_motor_data(controller_obj=c_theta1)
             
            (tail_meas_Ang_pos[index], 
             tail_meas_Ang_vel[index], 
             _) = await read_motor_data(controller_obj=c_theta2) 
            # estimate theta1, theta1_dot
            (imu_theta1[index], 
             imu_theta1_dot[index], 
             raw_imu_pos[index], 
             raw_imu_vel[index]) = await state_estimation(pr, tail_meas_Ang_pos[index], tail_meas_Ang_vel[index])
                                                                                                                          
            quat_wxyz, vel_xyz, acc_xyz, euler_xyz = await read_imu_data(pr)
            meas_time[index] = t
            index += 1                          
            meas_dt = time.time() - start_loop

    except BaseException as e:
        print("**Exception Block**")   
        print(f"Error= {e}")
    finally:
      # Save Trajectory to a csv file to be sent to the motor.
        data = {"time": meas_time[:index],
                "arm1_meas_Ang_pos": arm1_meas_Ang_pos[:index],
                "tail_meas_Ang_pos": tail_meas_Ang_pos[:index],
                "imu_theta1": imu_theta1[:index],
                "raw_imu_pos": raw_imu_pos[:index],
                "arm1_meas_Ang_vel": arm1_meas_Ang_vel[:index],
                "tail_meas_Ang_vel": tail_meas_Ang_vel[:index],
                "imu_theta1_dot": imu_theta1_dot[:index],
                "raw_imu_vel": raw_imu_vel[:index]}

        date, directory = plot_custom_data(meas_time[:index],
                                           arm1_meas_Ang_pos[:index],
                                           tail_meas_Ang_pos[:index],
                                           imu_theta1[:index],
                                           raw_imu_pos[:index],
                                           arm1_meas_Ang_vel[:index],
                                           tail_meas_Ang_vel[:index],
                                           imu_theta1_dot[:index],
                                           raw_imu_vel[:index]                                           
                                           )                
        save_data(data, directory+'/measured.csv')        
        # Disabling the motor
        await c_theta1.set_stop()
        await c_theta2.set_stop()


if __name__ == '__main__':
#    motor id and bus number mapping
    bus_number = [2]
    motor_id = [8, 9]    
    # zero offset the motors
    zero_offset(bus_number[0], motor_id[0])
    zero_offset(bus_number[0], motor_id[1])
    n = 5000 * 2 
    (arm1_meas_Ang_pos,
    arm1_meas_Ang_vel,
    tail_meas_Ang_pos,
    tail_meas_Ang_vel,
    tail_meas_tau,
    tail_cmd_tau,
    tail_clipped_tau,
    phi_meas_Ang_pos,
    phi_meas_Ang_vel,
    phi_meas_tau,
    phi_cmd_tau,
    phi_clipped_tau,
    arm2_meas_Ang_pos,
    arm2_meas_Ang_vel,
    raw_imu_pos,
    raw_imu_vel,
    meas_time) = prepare_empty_arrays(n)
    
    (imu_theta1,
    imu_theta1_dot,
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
    _,
    _,
    _,
    _) = prepare_empty_arrays(n)    

    if input('continue with real time experiment?') == 'y':
        asyncio.run(main())
    print('end of script')    
