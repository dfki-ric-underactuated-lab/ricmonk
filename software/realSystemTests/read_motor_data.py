import asyncio
import moteus
import moteus_pi3hat
## documentation https://github.com/mjbots/moteus/tree/main/lib/python
import numpy as np
import time
import math
import matplotlib.pyplot as plt
from datetime import datetime
from utils import (read_motor_data, generate_path, zero_offset, read_imu_data)


def prepare_data():
    # create empty numpy array so that the measured sensor data can later be stored in this array
    n = 5000
    meas_pos = np.zeros(n)
    meas_vel = np.zeros(n)
    meas_tau = np.zeros(n)
    meas_time = np.zeros(n)
    meas_imu_roll = np.zeros(n)
    return meas_pos, meas_vel, meas_tau, meas_time, meas_imu_roll, n


def plot_data(meas_time, meas_pos, meas_vel, meas_tau, meas_imu_roll):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p") 
    print("Making data plots.")
    plt.figure()
    plt.plot(meas_time, meas_pos)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.title("Position (rad) vs Time (s)")
    plt.legend(['theta'])
    plt.draw()
    plt.savefig(f'{date}_pos.pdf')

    plt.figure()
    plt.plot(meas_time, meas_vel)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad)")
    plt.title("Velocity (rad) vs Time (s)")
    plt.legend(['theta_dot'])
    plt.draw()
    plt.savefig(f'{date}_vel.pdf')

    plt.figure()
    plt.plot(meas_time, meas_tau)
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.title("Torque (Nm) vs Time (s)")
    plt.legend(['tau'])
    plt.draw()
    plt.savefig(f'{date}_tau.pdf')
    plt.show()
    
    plt.figure()
    plt.plot(meas_time, meas_imu_roll)
    plt.xlabel("Time (s)")
    plt.ylabel("IMU meas pos (Nm)")
    plt.title("IMU meas pos (Nm) vs Time (s)")
    plt.legend(['IMUpos'])
    plt.draw()
    plt.savefig(f'{date}_imupos.pdf')
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


async def main(meas_pos, meas_vel, meas_tau, meas_time, meas_imu_roll, n):
    print("Motor Enabled.")
    # motor id and bus number mapping
    # bus:[servo_id]
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={
            2: [8],  # Fixme: change the  motor id
        },
    )
    # create a moteus.Controller object for interaction with controller
    c = moteus.Controller(id=8, transport=transport)
    print(c)
    await c.set_stop()
    
    # Reset IMU
    moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll': 0,
                                             'pitch': 0,
                                             'yaw': 0})  
    time.sleep(0.5)                                                   
    con = (180,0,0) # usb down
    #con = (-90,0,-90) # usb up
    # create a moteus_pi3hat.Pi3HatRouter for IMU reading
    pr = moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll': con[0],
                                                  'pitch': con[1],
                                                  'yaw': con[2]})                                                
    time.sleep(4)                                               
    _, _, _, init_euler_xyz = await read_imu_data(pr) 

    index = 0
    t = 0
    meas_dt = 0
    start_time = time.time()

    try:
        while index < n:
            start_loop = time.time()
            t += meas_dt
            # read the data from the motor
            print('here')
            #state = await c.set_position(position=None, 
            #                             velocity=None, 
            #                             kp_scale=0, 
            #                             kd_scale=0,
            #                             stop_position=None,                              
            #                             feedforward_torque=0, 
            #                             maximum_torque=0, 
            #                             watchdog_timeout=None, 
            #                             query=True)
            mp, mv, mt = await read_motor_data(c)
            
            quat_wxyz, vel_xyz, acc_xyz, euler_xyz = await read_imu_data(pr) 
            r, p, y = euler_xyz
            print('here too')
            # store data
            meas_pos[index] = mp
            meas_vel[index] = mv
            meas_tau[index] = mt
            meas_time[index] = t
            meas_imu_roll[index] = r
            index += 1
            meas_dt = time.time() - start_loop

    except BaseException as e:
        print("**Exeptoin Block**")
        print(f"Error= {e}")
        await c.set_stop()
        date = plot_data(meas_time, meas_pos, meas_vel, meas_tau, meas_imu_roll)
        save_data(date, meas_pos, meas_vel, meas_tau,meas_time,meas_imu_roll)

    finally:
        # Disabling the motor
        await c.set_position(position=None, 
                             velocity=None, 
                             kp_scale=0, 
                             kd_scale=0,
                             stop_position=None,                              
                             feedforward_torque=0, 
                             maximum_torque=0, 
                             watchdog_timeout=None, 
                             query=True)
        await c.set_stop()
        print("Motor disabled.")

        return meas_dt, meas_time, meas_pos, meas_vel, meas_tau, meas_imu_roll


if __name__ == '__main__':
    meas_pos, meas_vel, meas_tau, meas_time, meas_imu_roll, n = prepare_data()
    zero_offset(2,8)
    if(input("continue (y/n)") == 'y'):
        (meas_dt, meas_time, meas_pos, meas_vel, meas_tau, meas_imu_roll) = asyncio.run(main(meas_pos, meas_vel, meas_tau, meas_time, meas_imu_roll, n))
        date = plot_data(meas_time, meas_pos, meas_vel, meas_tau, meas_imu_roll)
    else:
        print("Tata!")




