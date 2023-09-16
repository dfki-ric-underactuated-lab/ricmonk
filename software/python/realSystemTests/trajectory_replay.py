import asyncio
import numpy as np
import moteus
import moteus_pi3hat
import time
# Local imports
from utils import (send_rad_command,
                   read_imu_data,
                   state_estimation,
                   state_estimation_v2,                   
                   plot_custom_data,
                   save_data,
                   read_data,
                   prepare_des_data,
                   zero_offset,
                   create_nominal_pcws,
                   tau_tvlqr,
                   prepare_store_data,
                   motor_test_parameters
                   )

DATA = prepare_store_data(n=8000)
INDEX = 0
def data_append(
    index,

    TIME=np.nan,
    
    ARM1_MEAS_POS=np.nan,
    ARM1_MEAS_VEL=np.nan,
    TAIL_MEAS_POS=np.nan,
    TAIL_MEAS_VEL=np.nan,
    ARM2_MEAS_POS=np.nan,
    ARM2_MEAS_VEL=np.nan,
    TAIL_CMD_TAU=np.nan,
    TAIL_MEAS_TAU=np.nan,
    TAIL_CLIPPED_TAU=np.nan,
    ARM2_CMD_TAU=np.nan,
    ARM2_MEAS_TAU=np.nan,
    ARM2_CLIPPED_TAU=np.nan,
    RAW_IMU_POS=np.nan,
    RAW_IMU_VEL=np.nan,
    ARM2_DES_POS=np.nan,
    ARM2_DES_VEL=np.nan,
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
):
    DATA.arm1_meas_Ang_pos[index] = ARM1_MEAS_POS
    DATA.arm1_meas_Ang_vel[index] = ARM1_MEAS_VEL
    DATA.tail_meas_Ang_pos[index] = TAIL_MEAS_POS
    DATA.tail_meas_Ang_vel[index] = TAIL_MEAS_VEL
    DATA.tail_meas_tau[index] = TAIL_MEAS_TAU
    DATA.tail_cmd_tau[index] = TAIL_CMD_TAU
    DATA.tail_clipped_tau[index] = TAIL_CLIPPED_TAU
    DATA.arm2_meas_Ang_pos[index] = ARM2_MEAS_POS
    DATA.arm2_meas_Ang_vel[index] = ARM2_MEAS_VEL
    DATA.arm2_meas_tau[index] = ARM2_MEAS_TAU
    DATA.arm2_cmd_tau[index] = ARM2_CMD_TAU
    DATA.arm2_clipped_tau[index] = ARM2_CLIPPED_TAU
    DATA.meas_time[index] = TIME
    DATA.arm1_des_Ang_pos_store[index] = ARM1_DES_POS
    DATA.arm1_des_Ang_vel_store[index] = ARM1_DES_VEL
    DATA.tail_des_Ang_pos_store[index] = TAIL_DES_POS
    DATA.tail_des_Ang_vel_store[index] = TAIL_DES_VEL
    #DATA.tail_des_tau_store[index] = 
    DATA.arm2_des_Ang_pos_store[index] = ARM2_DES_POS
    DATA.arm2_des_Ang_vel_store[index] = ARM2_DES_VEL
    #DATA.arm2_des_tau_store[index] = 
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


async def main(data, test):
    print("Motor Enabled.")
    # motor id and bus number mapping
    transport1 = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={
            bus_number: [motor_id[0]],
        },
    )
    transport2= moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={
            bus_number: [motor_id[1]],
        },
    )
    # create a moteus.Controller object for interaction with controller
    servo_tail = moteus.Controller(id=motor_id[0], transport=transport1)
    servo_arm2 = moteus.Controller(id=motor_id[1], transport=transport2)
    # Reset the IMU
    moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll' : 20,
                                             'pitch': 0,
                                             'yaw'  : 0})  
    time.sleep(0.5)                                                   
    #con = (90,0,90) # usb down  #OLD CODE
    con = (0,0,0)
    #con = (-90,0,-90) # usb up
    # create a moteus_pi3hat.Pi3HatRouter for IMU reading
    pr = moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll' : con[0],
                                                  'pitch': con[1],
                                                  'yaw'  : con[2]})
    time.sleep(2)                                               
    _, _, _, init_euler_xyz = await read_imu_data(pr)                       
    await servo.set_stop()
    # nominal pcws
    nominal_pcws= create_nominal_pcws(data)  
    # runnig loop variables    
    index = 0
    t = 0
    meas_dt = 0
    time_exceed_counter = 0

    # inital state array
    arm1_pos = nominal_pcws.arm1_des_Ang_pos_pcw.get_value(t)
    arm1_vel = nominal_pcws.arm1_des_Ang_vel_pcw.get_value(t)
    tail_pos = nominal_pcws.tail_des_Ang_pos_pcw.get_value(t)
    tail_vel = nominal_pcws.tail_des_Ang_vel_pcw.get_value(t)    
    arm2_pos = nominal_pcws.arm2_des_Ang_pos_pcw.get_value(t)
    arm2_vel = nominal_pcws.arm2_des_Ang_vel_pcw.get_value(t)
    test = ff_replay
    
    try:
        while t < data.des_time[-1]:
            start_loop = time.time()
            t += meas_dt
            if t > data.des_time[-1]:
                t = data.des_time[-1]
                time_exceed_counter += 1

            #desired states
            des_arm1_pos = nominal_pcws.arm1_des_Ang_pos_pcw.get_value(t)
            des_arm1_vel = nominal_pcws.arm1_des_Ang_vel_pcw.get_value(t)
            des_tail_pos = nominal_pcws.tail_des_Ang_pos_pcw.get_value(t)
            des_tail_vel = nominal_pcws.tail_des_Ang_vel_pcw.get_value(t)    
            des_arm2_pos = nominal_pcws.arm2_des_Ang_pos_pcw.get_value(t)
            des_arm2_vel = nominal_pcws.arm2_des_Ang_vel_pcw.get_value(t)
            des_states_array = np.array([
                                [des_arm1_pos],
                                [des_tail_pos],
                                [des_arm2_pos],                                
                                [des_arm1_vel],
                                [des_tail_vel],
                                [des_arm2_vel],
                                ])
            
            # send commands to motor in radians
            meas_state = np.array([
                                [arm1_pos],
                                [tail_pos],
                                [arm2_pos],                                
                                [arm1_vel],
                                [tail_vel],
                                [arm2_vel],
                                ]) 
            #Assumption
            brach_sign=1
            #Making a few changes to this line, to be able to use 'tau_tvlqr'
            if test == 'tvlqr':
                u_tvlqr = tau_tvlqr(nominal_pcws=nominal_pcws, brach_sign=brach_sign, des_states_array=des_states_array, meas_states_array=meas_state, time=t)
                tau_tail_cmd = u_tvlqr[0]
                tau_arm2_cmd = u_tvlqr[1]
            else:
                motor_params = motor_test_parameters(test=test, nominal_pcws=nominal_pcws, brach_sign=brach_sign, meas_state=meas_state, des_state=des_states_array, time=t, controller=None)
                tau_tail_cmd = motor_params.tau_tail_cmd
                tau_arm2_cmd = motor_params.tau_arm2_cmd
                
            u_tail_ff = nominal_pcws.tail_des_torque_tvlqr_pcw.get_value(t)
            u_arm2_ff = nominal_pcws.arm2_des_torque_tvlqr_pcw.get_value(t)
            
            tau_tail_clip = np.clip(tau_tail_cmd,-tau_limit,tau_limit)
            tau_arm2_clip = np.clip(tau_arm2_cmd,-tau_limit,tau_limit)

            (meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_arm2, meas_vel_arm2, meas_tau_arm2) = run(
	            send_rad_command(
	                controller_obj_tail=servo_tail,
	                controller_obj_arm2=servo_arm2, 
	                pos_tail=des_tail_pos, 
	                vel_tail=des_tail_vel,
	                tau_tail=tau_tail_clip, 
	                pos_arm2=des_arm2_pos,  
	                vel_arm2=des_arm2_vel, 
	                tau_arm2=tau_arm2_clip,
	                tau_limit=tau_limit,
	                kp_scale=motor_params.kp_scale,
	                kd_scale=motor_params.kd_scale,
	                watchdog_timeout=0.05,
	            )
        	)
            # (el_pos, 
            #  el_vel, 
            #  el_tau) = await send_rad_command(controller_obj=servo,
            #                                   pos=nominal_pcws.el_des_pos_pcw.get_value(t),
            #                                   vel=nominal_pcws.el_des_vel_pcw.get_value(t),
            #                                   tau=tau_cmd,
            #                                   tau_limit=tau_limit,
            #                                   kp_scale=motor_params.kp_scale,
            #                                   kd_scale=motor_params.kd_scale)
            (pos_arm1, vel_arm1, _, _) = run(state_estimation(
                pr=pr, pos_arm2=tail_pos, vel_arm2=tail_vel
            ))
            # (sh_pos,
            #  sh_vel,
            #  _,
            #  _) = await state_estimation(pr=pr, 
            #                              pos_el=el_pos, 
            #                              vel_el=el_vel, 
            #                              imu_init=init_euler_xyz[0])
            # store measured data
            data_append(
	            index=INDEX,
	            TIME=t,
	            ARM1_MEAS_POS=pos_arm1,
	            ARM1_MEAS_VEL=vel_arm1,
	            TAIL_MEAS_POS=meas_pos_tail,
	            TAIL_MEAS_VEL=meas_vel_tail,
	            ARM2_MEAS_POS=meas_pos_arm2,
	            ARM2_MEAS_VEL=meas_vel_arm2,
	            TAIL_CMD_TAU=u_tail_ff,
	            TAIL_MEAS_TAU=tau_tail_cmd,
	            TAIL_CLIPPED_TAU=tau_tail_clip,
	            ARM2_CMD_TAU=u_tail_ff,
	            ARM2_MEAS_TAU=tau_arm2_cmd,
	            ARM2_CLIPPED_TAU=tau_arm2_clip,
	            RAW_IMU_POS=np.nan,
	            RAW_IMU_VEL=np.nan,
	            ARM2_DES_POS=np.nan,
	            ARM2_DES_VEL=np.nan,
	            TAIL_DES_POS=np.nan,
	            TAIL_DES_VEL=np.nan,
	            ARM1_DES_POS=np.nan,
	            ARM1_DES_VEL=np.nan,
        	)
            INDEX += 1
            # data.elbow_meas_pos[index]=el_pos
            # data.elbow_meas_vel[index]=el_vel
            # data.elbow_meas_tau[index]=el_tau
            # data.shoulder_meas_pos[index]=sh_pos
            # data.shoulder_meas_vel[index]=sh_vel
            # # store desired data
            # data.elbow_des_pos[index]=nominal_pcws.el_des_pos_pcw.get_value(t)
            # data.elbow_des_vel[index]=nominal_pcws.el_des_vel_pcw.get_value(t)
            # data.elbow_des_tau[index]=nominal_pcws.el_des_tau_pcw.get_value(t)
            # data.shoulder_des_pos[index]=nominal_pcws.sh_des_pos_pcw.get_value(t)
            # data.shoulder_des_vel[index]=nominal_pcws.sh_des_vel_pcw.get_value(t)
            # data.elbow_des_tau_tvlqr[index]=nominal_pcws.el_tau_tvlqr_pcw.get_value(t)
            # data.meas_time[index] = t            

            meas_dt = time.time() - start_loop
    finally:
        print(f'time exceded from desired trajectory = {time_exceed_counter * meas_dt}')
        store_data={
            field:DATA[i][:INDEX] for i, field in enumerate(DATA._fields)
            if isinstance(DATA[i], np.ndarray)
        }        
        directory = plot_custom_data(store_data, test)        
      # Save Trajectory to a csv file to be sent to the motor.
        save_data(store_data, directory+'/measured.csv')
        # Disabling the motor
        await servo.set_stop()
        print("Motor disabled.")

if __name__ == '__main__':
    test = 'tvlqr'#'ff_replay'#'pd'#
    folder_name = 'data/trajectories/closed_loop'
    file_name = 'ZF_pid.csv'#'ZL.csv'#'ZR.csv'#
    tau_limit = 6
    bus_number = 2
    motor_id = [8, 9]    
    csv_data= read_data(folder=folder_name,
                        file=file_name,
                        up_directory=1)                    
    # prepare data for moteus controller
    data = prepare_des_data(csv_data, "pid") #changed 'prepare_data' to 'prepare_des_data'
    zero_offset(bus_number, motor_id)
    if input('continue with real time experiment?') == 'y':
        asyncio.run(main(data,test))
