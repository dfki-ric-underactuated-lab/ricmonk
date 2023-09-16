import os
import asyncio
import moteus
import moteus_pi3hat
from utils import (prepare_empty_arrays, 
                   rad2rev, 
                   rev2rad, 
                   read_data, 
                   prepare_data, 
                   wrap_z2pi,
                   zero_offset,
                   save_data,
                   generate_path, 
                   plot_data, 
                   send_rad_command,
                   get_theta1)
import time
import numpy as np
from datetime import datetime
import math

def simple_print(th1,th2,imu):
    print(f"imu_angle: {np.rad2deg(imu)}") 
    print(f"[theta1,theta2]: [{np.rad2deg(th1),np.rad2deg(th2)}]")                                                                                                                                 
    print(f'imu-theta2: {np.rad2deg(imu) - np.rad2deg(th2)}')
def wrapped_z2pi_print(th1,th2,imu):
    print(f"imu_angle: {np.rad2deg((imu))}") 
    print(f"[theta1,theta2]: [{np.rad2deg((th1)),np.rad2deg((th2))}]")                                                                                                                                 
    print(f"wrapped_imu_angle: {np.rad2deg(wrap_z2pi(imu))}")
    print(f"wrapped[theta1,theta2]: [{np.rad2deg(wrap_z2pi(th1)),np.rad2deg(wrap_z2pi(th2))}]")                                                                                                                                 
    print(f'imu-theta2: {np.rad2deg(wrap_z2pi(imu)) - np.rad2deg(wrap_z2pi(th2))}')    
def plot_custom_data(meas_time, 
                     shoulder_meas_pos,
                     shoulder_meas_vel, 
                     elbow_meas_pos,
                     elbow_meas_vel):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
    folder_name = 'results/imu_validation'
    file_name = date
    directory = generate_path(folder_name, file_name, 1)
    print(f'save to {directory}')
    os.makedirs(directory)
    plot_data(x_data=[meas_time, meas_time],
              y_data=[shoulder_meas_pos, elbow_meas_pos],
              labels=["Time (s)", "Position (rad)"],
              title="Position (rad) vs Time (s)",
              legends=['shoulder', 'elbow'],
              save_name=directory + '/pos.pdf')

    plot_data(x_data=[meas_time, meas_time],
              y_data=[shoulder_meas_vel, elbow_meas_vel],
              labels=["Time (s)", "Velocity (rad/s)"],
              title="Velocity (rad/s) vs Time (s)",
              legends=['shoulder', 'elbow'],
              save_name=directory + '/vel.pdf')
    return date, directory


async def main():
    print("Motor Enabled.")   
    if len(bus_number) == 1:
        transport = moteus_pi3hat.Pi3HatRouter(
            servo_bus_map={
                bus_number[0]: motor_id,
            },
        )
        print(f"check Bus Numbers and motor id: {bus_number[0]}:[{motor_id[0]},{motor_id[1]}]")
    else:
        transport = moteus_pi3hat.Pi3HatRouter(
            servo_bus_map={
#                bus_number[0]: [motor_id[0]],
#                bus_number[1]: [motor_id[1]],                
                 bus_number:motor_id,
            },
        )            
        print(f"check Bus Numbers and motor id: {bus_number[0]}:{[motor_id[0]]},{bus_number[1]}:{[motor_id[1]]}")        
    # create a moteus_pi3hat.Pi3HatRouter for IMU reading
    pr = moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll': 90,
                                                'pitch': 0,
                                                'yaw': 0})
    # wait until convergence of IMU reading
    time.sleep(0.5)     

    # create a moteus.Controller object for interaction with controller
    c_shoulder = moteus.Controller(id=motor_id[0], transport=transport)
    c_elbow = moteus.Controller(id=motor_id[1], transport=transport)
    await c_shoulder.set_stop()
    await c_elbow.set_stop()

    index = 0
    t = 0
    meas_dt = 0
    #    start_time = time.time()
    tau_limit = 3
    pos_sh = 0.
    pos_el = 0.
    # (pos_sh, 
    #     _, 
    #     _) = await send_rad_command(controller_obj=c_shoulder,
    #                                             pos=pos_sh,
    #                                             vel=vel,
    #                                             tau_limit=tau_limit)            
    # (pos_el, 
    #     _, 
    #     _) = await send_rad_command(controller_obj=c_elbow,
    #                                             pos=pos_el,
    #                                             vel=vel,
    #                                             tau_limit=tau_limit)
    while index < n:
        start_loop = time.time()
        t += meas_dt
        # send commands to motor in radians
        (_, 
            _, 
            _) = await send_rad_command(controller_obj=c_shoulder,
                                                    pos=pos_sh,
                                                    vel=vel,
                                                    tau_limit=tau_limit)            
        (_, 
            _, 
            _) = await send_rad_command(controller_obj=c_elbow,
                                                    pos=pos_el,
                                                    vel=vel,
                                                    tau_limit=tau_limit)                                                           
        # meas_time[index] = t                                                          
        # read the data from the motor
        # sh_state = await c_shoulder.set_position(position=rad2rev(pos_sh),
        #                              velocity=vel,
        #                              kp_scale=1,
        #                              kd_scale=1,
        #                              stop_position=None,
        #                              feedforward_torque=None,
        #                              maximum_torque=tau_limit,
        #                              watchdog_timeout=None,
                                    #  query=True)
        # el_state = await c_elbow.set_position(position=rad2rev(pos_el),
        #                              velocity=vel,
        #                              kp_scale=1,
        #                              kd_scale=1,
        #                              stop_position=None,
        #                              feedforward_torque=None,
        #                              maximum_torque=tau_limit,
        #                              watchdog_timeout=None,
        #                              query=True)  
        # imu = await pr.cycle([], request_attitude=True)
        # quaternion_obj = imu[0]
        # w, x, y, z = quaternion_obj.attitude.w, \
        #              quaternion_obj.attitude.x, \
        #              quaternion_obj.attitude.y, \
        #              quaternion_obj.attitude.z
        # '''
        # denom = math.sqrt(1 - w * w)
        # print(f"denom: {denom}")
        # '''
        # denom = 1
        # ax = x / denom
        # ay = y / denom
        # az = z / denom
        # angle_axis = 2 * math.acos(w)
        # theta2 = angle_axis if ay < 0 else -angle_axis
        # # store data
        # shoulder_meas_pos[index] = rev2rad(sh_state.values[moteus.Register.POSITION]) 
        # shoulder_meas_vel[index] = rev2rad(sh_state.values[moteus.Register.VELOCITY])
        # elbow_meas_pos[index] = rev2rad(el_state.values[moteus.Register.POSITION]) 
        # elbow_meas_vel[index] = rev2rad(el_state.values[moteus.Register.VELOCITY])            
        # meas_time[index] = t                          
        # print(f"imu_angle: {theta2}") 
        # print(f"[theta1,theta2]: [{shoulder_meas_pos[index],elbow_meas_pos[index]}]")                                                                                        
        index += 1
        pos_el += final_pos_elbow / n
        pos_sh += final_pos_shoudler / n
        '''
        while time.time() - start_loop < dt:
            pass
        '''
        # keep the final position
        # print(f'freez for {freez_time} seconds at pos [{np.rad2deg(final_pos_shoudler)},{np.rad2deg(final_pos_elbow)}].')
        # start_time = time.time()
        # while time.time() - start_time < freez_time:
    print(f'freez for {freez_loop} loops at pos [{np.rad2deg(final_pos_shoudler)},{np.rad2deg(final_pos_elbow)}].')
    index = 0
    t = 0
    meas_dt = 0   
    try:     
        while index < freez_loop:
            start_loop = time.time()        
            t += meas_dt    
            imu = await pr.cycle([], request_attitude=True)
            quaternion_obj = imu[0]
            w, x, y, z = quaternion_obj.attitude.w, \
                        quaternion_obj.attitude.x, \
                        quaternion_obj.attitude.y, \
                        quaternion_obj.attitude.z
            '''
            denom = math.sqrt(1 - w * w)
            print(f"denom: {denom}")
            '''
            denom = math.sqrt(1-w*w)
            ax = x / denom
            ay = y / denom
            az = z / denom
            angle_axis = 2 * math.acos(w)
            angle_axis = angle_axis if ay < 0 else -angle_axis           
            # send commands to motor in radians
            (shoulder_meas_pos[index], 
            shoulder_meas_vel[index], 
            _) = await send_rad_command(controller_obj=c_shoulder,
                                                    pos=pos_sh,
                                                    vel=vel,
                                                    tau_limit=tau_limit)            
            (elbow_meas_pos[index], 
            elbow_meas_vel[index], 
            _) = await send_rad_command(controller_obj=c_elbow,
                                                    pos=pos_el,
                                                    vel=vel,
                                                    tau_limit=tau_limit)                                                           
            theta1 = get_theta1(angle_axis, elbow_meas_pos[index])    
            # print(f"imu_angle: {np.rad2deg(angle_axis)}") 
            print(f"[theta1,theta2]: [{np.rad2deg(shoulder_meas_pos[index]),np.rad2deg(elbow_meas_pos[index])}]")                                                                                                                                 
            print(f'imu-theta1: {np.rad2deg(theta1)}')      
            meas_time[index] = t
            index += 1                          

            meas_dt = time.time() - start_loop

    except BaseException as e:
        print("**Exeptoin Block**")   
        print(f"Error= {e}")
    finally:
      # Save Trajectory to a csv file to be sent to the motor.
        data = {"time": meas_time[:index],
                "shoulder_pos": shoulder_meas_pos[:index],
                "shoulder_vel": shoulder_meas_vel[:index],
                "elbow_pos": elbow_meas_pos[:index],
                "elbow_vel": elbow_meas_vel[:index]}
        # date, directory = plot_custom_data(meas_time[:index],
        #                                    shoulder_meas_pos[:index],
        #                                    shoulder_meas_vel[:index],
        #                                    elbow_meas_pos[:index],
        #                                    elbow_meas_vel[:index])                
        # save_data(data, directory+'/measured.csv')
        # Disabling the motor
        await c_shoulder.set_stop()
        await c_elbow.set_stop()


if __name__ == '__main__':
#    motor id and bus number mapping
    bus_number = [2]
    motor_id = [8, 9]    
    # zero offset the motors
    zero_offset(bus_number[0], motor_id[0])
    zero_offset(bus_number[0], motor_id[1])
    n = 500
    # freez_time = 20.
    freez_loop = 5000
    final_pos_shoudler = np.deg2rad(0.)
    final_pos_elbow = np.deg2rad(0)
    vel =  0.2         
    # (meas_time, 
    #  shoulder_meas_pos, 
    #  shoulder_meas_vel, 
    #  elbow_meas_pos, 
    #  elbow_meas_vel,
    #  _,
    #  _,
    #  _,
    #  _) = prepare_empty_arrays(n)  
    (meas_time, 
     shoulder_meas_pos, 
     shoulder_meas_vel, 
     elbow_meas_pos, 
     elbow_meas_vel,
     _,
     _,
     _,
     _) = prepare_empty_arrays(freez_loop)  
    if input('continue with real time experiment?') == 'y':
        asyncio.run(main())
    print('end of script')    
