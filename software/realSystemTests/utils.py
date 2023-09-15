import time
import numpy as np
import os
from collections import namedtuple
import pandas as pd
import moteus
import moteus_pi3hat
from scipy.optimize import curve_fit as cf

try:
    import matplotlib.pyplot as plt
    import os
    import math
    from datetime import datetime
    import matplotlib.transforms as mtransforms
    import asyncio
    #from scipy.optimize import curve_fit as cf
    #import moteus
    #import moteus_pi3hat
except BaseException as e:
    print(e)
try:
    from pydrake.all import (
        DirectCollocation,
        PiecewisePolynomial,
        Solve,
        Parser,
        MultibodyPlant,
        SceneGraph,
        DiagramBuilder,
        MultibodyPositionToGeometryPose,
        TrajectorySource,
        ConnectMeshcatVisualizer,
        Simulator,
    )
except BaseException as e:
    print(e)


def parent(path):
    return os.path.dirname(path)


def generate_path(path_to_folder, file_name, up_directory_counter):
    cur_dir = os.path.realpath(os.curdir)
    tmp_dir = cur_dir
    i = 0
    while i < up_directory_counter:
        tmp_dir = parent(tmp_dir)
        i += 1
    main_dir = tmp_dir
    return os.path.join(main_dir, path_to_folder, file_name)


def read_data(folder, file, up_directory):
    path_to_file = generate_path(folder, file, up_directory)
    data = pd.read_csv(path_to_file)
    return data


def plot_data(
    x_data,
    y_data,
    labels,
    title,
    legends=None,
    save_name=None,
    linestyle=None,
    linewidth=None,
    colors=None,
):
    plt.figure(figsize=(15, 10))
    if linestyle is None:
        linestyle = [None] * len(x_data)
    if colors is None:
        colors = [None] * len(x_data)
    for i in range(len(x_data)):
        plt.plot(
            x_data[i],
            y_data[i],
            linestyle=linestyle[i],
            linewidth=linewidth,
            c=colors[i],
        )

    plt.xlabel(labels[0])
    plt.ylabel(labels[1])
    plt.title(title)
    if legends is None:
        pass
    else:
        plt.legend(legends)
    plt.draw()
    if save_name is None:
        pass
    else:
        print(f"Making {save_name} plot.")
        plt.savefig(save_name)
    return plt


def prepare_store_data(n):
    # empty arrays for measured data
    (
        arm1_meas_Ang_pos,
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
        meas_time,
    ) = prepare_empty_arrays(n)
    # empty arrays to store desired data
    (
        arm1_des_Ang_pos_store,
        arm1_des_Ang_vel_store,
        tail_des_Ang_pos_store,
        tail_des_Ang_vel_store,
        tail_des_tau_store,
        phi_des_Ang_pos_store,
        phi_des_Ang_vel_store,
        phi_des_tau_store,
        arm2_des_Ang_pos_store,
        arm2_des_Ang_vel_store,
        k1_store,
        k2_store,
        k3_store,
        k4_store,
        k5_store,
        k6_store,
        k7_store,
        k8_store,
        k9_store,
        k10_store,
        k11_store,
        k12_store,
        state_machine_flag,
    ) = prepare_empty_arrays_with_extra(n)
    # (
    #     shoulder_des_pos_store,
    #     shoulder_des_vel_store,
    #     _,
    #     _,
    #     elbow_des_pos_store,
    #     elbow_des_vel_store,
    #     elbow_des_tau_store,
    #     k1_store,
    #     k2_store,
    #     k3_store,
    #     k4_store,
    #     state_machine_flag,
    # ) = prepare_empty_arrays(n)
    Data = namedtuple(
        "Data",
        [
            "meas_time",
            "arm1_des_Ang_pos_store",
            "arm1_meas_Ang_pos",
            "arm1_des_Ang_vel_store",
            "arm1_meas_Ang_vel",
            "tail_des_Ang_pos_store",
            "tail_meas_Ang_pos",
            "tail_des_Ang_vel_store",
            "tail_meas_Ang_vel",
            "tail_des_tau_store",
            "tail_meas_tau",
            "tail_cmd_tau",
            "tail_clipped_tau",
            "phi_des_Ang_pos_store",
            "phi_meas_Ang_pos",
            "phi_des_Ang_vel_store",
            "phi_meas_Ang_vel",
            "phi_des_tau_store",
            "phi_meas_tau",
            "phi_cmd_tau",
            "phi_clipped_tau",
        	"arm2_des_Ang_vel_store",
            "arm2_meas_Ang_pos",
            "arm2_des_Ang_pos_store",
        	"arm2_meas_Ang_vel",
            "raw_imu_pos",
            "raw_imu_vel",
            "k1_store",
            "k2_store",
            "k3_store",
            "k4_store",
            "k5_store",
            "k6_store", 
            "k7_store",
            "k8_store",
            "k9_store",
            "k10_store",
            "k11_store",
            "k12_store",  
            "state_machine_flag", 
        ],
    )
    data = Data(
        meas_time=meas_time,
        arm1_des_Ang_pos_store=arm1_des_Ang_pos_store,
        arm1_meas_Ang_pos=arm1_meas_Ang_pos,
        arm1_des_Ang_vel_store=arm1_des_Ang_vel_store,
        arm1_meas_Ang_vel=arm1_meas_Ang_vel,
        tail_des_Ang_pos_store=tail_des_Ang_pos_store,
        tail_meas_Ang_pos=tail_meas_Ang_pos,
        tail_des_Ang_vel_store=tail_des_Ang_vel_store,
        tail_meas_Ang_vel=tail_meas_Ang_vel,
        tail_des_tau_store=tail_des_tau_store,
        tail_meas_tau=tail_meas_tau,
        tail_cmd_tau=tail_cmd_tau,
        tail_clipped_tau=tail_clipped_tau,
        phi_des_Ang_pos_store=phi_des_Ang_pos_store,
        phi_meas_Ang_pos=phi_meas_Ang_pos,
        phi_des_Ang_vel_store=phi_des_Ang_vel_store,
        phi_meas_Ang_vel=phi_meas_Ang_vel,
        phi_des_tau_store=phi_des_tau_store,
        phi_meas_tau=phi_meas_tau,
        phi_cmd_tau=phi_cmd_tau,
        phi_clipped_tau=phi_clipped_tau,
        arm2_des_Ang_vel_store=arm2_des_Ang_vel_store,
        arm2_meas_Ang_pos=arm2_meas_Ang_pos,
        arm2_des_Ang_pos_store=arm2_des_Ang_pos_store,
    	arm2_meas_Ang_vel=arm2_meas_Ang_vel,
        raw_imu_pos=raw_imu_pos,
        raw_imu_vel=raw_imu_vel,
        k1_store=k1_store,
        k2_store=k2_store,
        k3_store=k3_store,
        k4_store=k4_store,
        k5_store=k5_store,
        k6_store=k6_store,
        k7_store=k7_store,
        k8_store=k8_store,
        k9_store=k9_store,
        k10_store=k10_store,
        k11_store=k11_store,
        k12_store=k12_store,
        state_machine_flag=state_machine_flag,
    )
    return data


def prepare_des_data(csv_data, controller):
    des_time = csv_data["time"].values
    ## arm1
    arm1_des_Ang_pos = csv_data["arm1_Ang_pos"].values
    arm1_des_Ang_vel = csv_data["arm1_Ang_vel"].values
    arm1_des_Ang_acc = csv_data["arm1_Ang_acc"].values
    arm1_des_Ang_jerk = csv_data["arm1_Ang_jerk"].values
    #arm1_Ang_tau = csv_data["arm1_Ang_vel"].values
    ## tail
    tail_des_Ang_pos = csv_data["tail_Ang_pos"].values
    tail_des_Ang_vel = csv_data["tail_Ang_vel"].values
    tail_des_Ang_acc = csv_data["tail_Ang_acc"].values
    tail_des_Ang_jerk = csv_data["tail_Ang_jerk"].values
    tail_des_torque = csv_data["tail_torque"].values
    tail_des_torque_tvlqr = csv_data["tail_meas_torque"].values
    ## phi
    arm2_des_Ang_pos = csv_data["arm2_Ang_pos"].values
    arm2_des_Ang_vel = csv_data["arm2_Ang_vel"].values
    arm2_des_Ang_acc = csv_data["arm2_Ang_acc"].values
    arm2_des_Ang_jerk = csv_data["arm2_Ang_jerk"].values
    ###
    phi_des_Ang_pos = csv_data["phi_pos"].values
    phi_des_Ang_vel = csv_data["phi_vel"].values
    phi_des_Ang_acc = csv_data["phi_acc"].values
    phi_des_Ang_jerk = csv_data["phi_jerk"].values
    phi_des_torque = csv_data["phi_torque"].values
    phi_des_torque_tvlqr = csv_data["phi_meas_torque"].values

    # ## shoulder
    # shoulder_des_pos = csv_data["shoulder_pos"].values
    # shoulder_des_vel = csv_data["shoulder_vel"].values
    # shoulder_des_acc = csv_data["shoulder_acc"].values
    # shoulder_des_jerk = csv_data["shoulder_jerk"].values
    # shoulder_des_tau = csv_data["shoulder_torque"].values
    # ## elbow.values
    # elbow_des_pos = csv_data["elbow_pos"].values
    # elbow_des_vel = csv_data["elbow_vel"].values
    # elbow_des_acc = csv_data["elbow_acc"].values
    # elbow_des_jerk = csv_data["elbow_jerk"].values
    # elbow_des_tau = csv_data["elbow_torque"].values
    # elbow_des_tau_tvlqr = csv_data["elbow_torque_tvlqr"].values
    # K values
    #print(len(phi_des_torque_tvlqr))
    if controller == "tvlqr":
        #print(type(phi_des_torque_tvlqr))
        #k1 = np.zeros(len(phi_des_torque_tvlqr))
        k1 = csv_data["K1"].values
        k2 = csv_data["K2"].values
        k3 = csv_data["K3"].values
        k4 = csv_data["K4"].values
        k5 = csv_data["K5"].values
        k6 = csv_data["K6"].values
        k7 = csv_data["K7"].values
        k8 = csv_data["K8"].values
        k9 = csv_data["K9"].values
        k10 = csv_data["K10"].values
        k11 = csv_data["K11"].values
        k12 = csv_data["K12"].values
        k0 = csv_data["k0"].values
    elif controller =="pd":
        k1 = np.zeros(len(phi_des_torque_tvlqr))
        k2 = np.zeros(len(phi_des_torque_tvlqr))
        k3 = np.zeros(len(phi_des_torque_tvlqr))
        k4 = np.zeros(len(phi_des_torque_tvlqr))
        k5 = np.zeros(len(phi_des_torque_tvlqr))
        k6 = np.zeros(len(phi_des_torque_tvlqr))
        k7 = np.zeros(len(phi_des_torque_tvlqr))
        k8 = np.zeros(len(phi_des_torque_tvlqr))
        k9 = np.zeros(len(phi_des_torque_tvlqr))
        k10 = np.zeros(len(phi_des_torque_tvlqr))
        k11 = np.zeros(len(phi_des_torque_tvlqr))
        k12 = np.zeros(len(phi_des_torque_tvlqr))
        k0 = np.zeros(len(phi_des_torque_tvlqr))    

    # converting the desired trajectories according to the gear ratio
    dt = csv_data["time"][2] - csv_data["time"][1]  #HERE: previously there was a field called 'shoulder_tau'. Not using it anymore.
    Data = namedtuple(  
        "Data",
        [
            "arm1_des_Ang_pos",
            "arm1_des_Ang_vel",
            "arm1_des_Ang_acc",
            "arm1_des_Ang_jerk",
            "tail_des_Ang_pos",
            "tail_des_Ang_vel",
            "tail_des_Ang_acc",
            "tail_des_Ang_jerk",
            "tail_des_torque",
            "tail_des_torque_tvlqr",
            "arm2_des_Ang_pos",
            "arm2_des_Ang_vel",
            "arm2_des_Ang_acc",
            "arm2_des_Ang_jerk",
            "phi_des_Ang_pos",
            "phi_des_Ang_vel",
            "phi_des_Ang_acc",
            "phi_des_Ang_jerk",
            "phi_des_torque",
            "phi_des_torque_tvlqr",
            "des_time",
            "k1",
            "k2",
            "k3",
            "k4",
            "k5",
            "k6",
            "k7",
            "k8",
            "k9",
            "k10",
            "k11",
            "k12",
            "k0",
        ],
    )
    data = Data(
        arm1_des_Ang_pos=arm1_des_Ang_pos,
        arm1_des_Ang_vel=arm1_des_Ang_vel,
        arm1_des_Ang_acc=arm1_des_Ang_acc,
        arm1_des_Ang_jerk=arm1_des_Ang_jerk,
        tail_des_Ang_pos=tail_des_Ang_pos,
        tail_des_Ang_vel=tail_des_Ang_vel,
        tail_des_Ang_acc=tail_des_Ang_acc,
        tail_des_Ang_jerk=tail_des_Ang_jerk,
        tail_des_torque=tail_des_torque,
        tail_des_torque_tvlqr=tail_des_torque_tvlqr,
        arm2_des_Ang_pos=arm2_des_Ang_pos,
        arm2_des_Ang_vel=arm2_des_Ang_vel,
        arm2_des_Ang_acc=arm2_des_Ang_acc,
        arm2_des_Ang_jerk=arm2_des_Ang_jerk,
        phi_des_Ang_pos=phi_des_Ang_pos,
        phi_des_Ang_vel=phi_des_Ang_vel,
        phi_des_Ang_acc=phi_des_Ang_acc,
        phi_des_Ang_jerk=phi_des_Ang_jerk,
        phi_des_torque=phi_des_torque,
        phi_des_torque_tvlqr=phi_des_torque_tvlqr,
        des_time=des_time,
        k1=k1,
        k2=k2,
        k3=k3,
        k4=k4,
        k5=k5,
        k6=k6,
        k7=k7,
        k8=k8,
        k9=k9,
        k10=k10,
        k11=k11,
        k12=k12,
        k0=k0,
    )
    return data


def prepare_empty_arrays(n):
    arm1_meas_Ang_pos = np.empty(n)
    arm1_meas_Ang_vel = np.empty(n)     #NOTE: ignored 'arm1_meas_tau' and 'arm1_cmd_tau'
    tail_meas_Ang_pos = np.empty(n)
    tail_meas_Ang_vel = np.empty(n)
    tail_meas_tau = np.empty(n)
    tail_cmd_tau = np.empty(n)
    tail_clipped_tau = np.empty(n)
    phi_meas_Ang_pos = np.empty(n)
    phi_meas_Ang_vel = np.empty(n)
    phi_meas_tau = np.empty(n)
    phi_cmd_tau = np.empty(n)
    phi_clipped_tau = np.empty(n)
    arm2_meas_Ang_pos = np.empty(n)
    arm2_meas_Ang_vel = np.empty(n)
    raw_imu_pos = np.empty(n)
    raw_imu_vel = np.empty(n)
    meas_time = np.empty(n)

    arm1_meas_Ang_pos[:] = np.nan
    arm1_meas_Ang_vel[:] = np.nan
    tail_meas_Ang_pos[:] = np.nan
    tail_meas_Ang_vel[:] = np.nan
    tail_meas_tau[:] = np.nan
    tail_cmd_tau[:] = np.nan
    tail_clipped_tau[:] = np.nan
    phi_meas_Ang_pos[:] = np.nan
    phi_meas_Ang_vel[:] = np.nan
    phi_meas_tau[:] = np.nan
    phi_cmd_tau[:] = np.nan
    phi_clipped_tau[:] = np.nan
    arm2_meas_Ang_pos[:] = np.nan
    arm2_meas_Ang_vel[:] = np.nan
    raw_imu_pos[:] = np.nan
    raw_imu_vel[:] = np.nan
    meas_time[:] = np.nan

    return (
        arm1_meas_Ang_pos,
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
        meas_time,
    )
    
def prepare_empty_arrays_with_extra(n):
    arm1_meas_Ang_pos = np.empty(n)
    arm1_meas_Ang_vel = np.empty(n)     #NOTE: ignored 'arm1_meas_tau' and 'arm1_cmd_tau'
    tail_meas_Ang_pos = np.empty(n)
    tail_meas_Ang_vel = np.empty(n)
    tail_meas_tau = np.empty(n)
    tail_cmd_tau = np.empty(n)
    tail_clipped_tau = np.empty(n)
    phi_meas_Ang_pos = np.empty(n)
    phi_meas_Ang_vel = np.empty(n)
    phi_meas_tau = np.empty(n)
    phi_cmd_tau = np.empty(n)
    phi_clipped_tau = np.empty(n)
    arm2_meas_Ang_pos = np.empty(n)
    arm2_meas_Ang_vel = np.empty(n)
    raw_imu_pos = np.empty(n)
    raw_imu_vel = np.empty(n)
    meas_time = np.empty(n)
    K7 = np.empty(n)
    K8 = np.empty(n)
    K9 = np.empty(n)
    K10 = np.empty(n)
    K11 = np.empty(n)
    K12 = np.empty(n)
    

    arm1_meas_Ang_pos[:] = np.nan
    arm1_meas_Ang_vel[:] = np.nan
    tail_meas_Ang_pos[:] = np.nan
    tail_meas_Ang_vel[:] = np.nan
    tail_meas_tau[:] = np.nan
    tail_cmd_tau[:] = np.nan
    tail_clipped_tau[:] = np.nan
    phi_meas_Ang_pos[:] = np.nan
    phi_meas_Ang_vel[:] = np.nan
    phi_meas_tau[:] = np.nan
    phi_cmd_tau[:] = np.nan
    phi_clipped_tau[:] = np.nan
    arm2_meas_Ang_pos[:] = np.nan
    arm2_meas_Ang_vel[:] = np.nan
    raw_imu_pos[:] = np.nan
    raw_imu_vel[:] = np.nan
    meas_time[:] = np.nan
    K7[:] = np.nan
    K8[:] = np.nan
    K9[:] = np.nan
    K10[:] = np.nan
    K11[:] = np.nan
    K12[:] = np.nan

    return (
        arm1_meas_Ang_pos,
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
        meas_time,
        K7,
        K8,
        K9,
        K10,
        K11,
        K12
    )


def save_data(data, path_to_save):
    header = ",".join(data.keys())
    data = np.vstack(data.values()).T
    np.savetxt(path_to_save, data, delimiter=",", header=header, comments="")
    print(f"saved csv to {path_to_save}")


def rad2rev(angle_in_radians):
    return angle_in_radians * (1 / (2 * np.pi))


def rev2rad(angle_in_revolution):
    return angle_in_revolution * (2 * np.pi)

def zero_offset_two_motors(bus_number, motor_id):
    zero_offset(bus_number, motor_id[0])
    zero_offset(bus_number, motor_id[1]) 
    
    
def zero_offset(bus_number, motor_id):
    os.system(
        f"sudo moteus_tool --zero-offset  --pi3hat-cfg '{bus_number}={motor_id}' -t {motor_id}"
    )
    print(f"motor {motor_id} zero offset was successful.")


async def send_rad_command(
    controller_obj_tail=None,
    controller_obj_phi=None,
    pos_tail=None,
    vel_tail=None,
    tau_tail=None,
    pos_phi=None,
    vel_phi=None,
    tau_phi=None,
    tau_limit=None,
    kp_scale=1,
    kd_scale=1,
    watchdog_timeout=None,
):
    state_tail = await controller_obj_tail.set_position(
        position=rad2rev(pos_tail),  # 0,#
        velocity=rad2rev(vel_tail),  # 0,#
        kp_scale=kp_scale * 1,
        kd_scale=kd_scale * 1,
        stop_position=None,
        feedforward_torque=tau_tail,
        maximum_torque=tau_limit,
        watchdog_timeout=watchdog_timeout,
        query=True,
    )
    state_phi = await controller_obj_phi.set_position(
        position=rad2rev(pos_phi),  # 0,#
        velocity=rad2rev(vel_phi),  # 0,#
        kp_scale=kp_scale,
        kd_scale=kd_scale,
        stop_position=None,
        feedforward_torque=tau_phi,
        maximum_torque=tau_limit,
        watchdog_timeout=watchdog_timeout,
        query=True,
    )
    # store data
    meas_pos_tail = rev2rad(state_tail.values[moteus.Register.POSITION])
    meas_vel_tail = rev2rad(state_tail.values[moteus.Register.VELOCITY])
    meas_tau_tail = state_tail.values[moteus.Register.TORQUE]
    meas_pos_phi = rev2rad(state_phi.values[moteus.Register.POSITION])
    meas_vel_phi = rev2rad(state_phi.values[moteus.Register.VELOCITY])
    meas_tau_phi = state_phi.values[moteus.Register.TORQUE]
    return meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi
    
async def send_rad_command_w_check(
    controller_obj_tail=None,
    controller_obj_phi=None,
    pos_tail=None,
    vel_tail=None,
    tau_tail=None,
    pos_phi=None,
    vel_phi=None,
    tau_phi=None,
    tau_limit=None,
    kp_scale=1,
    kd_scale=1,
    watchdog_timeout=None,
    prev_meas_pos_tail=None,
    prev_meas_vel_tail=None,
    prev_meas_tau_tail=None,
    prev_meas_pos_phi=None,
    prev_meas_vel_phi=None,
    prev_meas_tau_phi=None,
    index=None,
):
    state_tail = await controller_obj_tail.set_position(
        position=rad2rev(pos_tail),  # 0,#
        velocity=rad2rev(vel_tail),  # 0,#
        kp_scale=kp_scale * 1,
        kd_scale=kd_scale * 1,
        stop_position=None,
        feedforward_torque=tau_tail,
        maximum_torque=tau_limit,
        watchdog_timeout=watchdog_timeout,
        query=True,
    )
    state_phi = await controller_obj_phi.set_position(
        position=rad2rev(pos_phi),  # 0,#
        velocity=rad2rev(vel_phi),  # 0,#
        kp_scale=kp_scale,
        kd_scale=kd_scale,
        stop_position=None,
        feedforward_torque=tau_phi,
        maximum_torque=tau_limit,
        watchdog_timeout=watchdog_timeout,
        query=True,
    )
    # store data
    if state_tail is None:
        meas_pos_tail = prev_meas_pos_tail
        meas_vel_tail = prev_meas_vel_tail
        meas_tau_tail = prev_meas_tau_tail
        print("*")
        print(f"Tail missed a beat at index {index}")
        print("*")
    else:
        meas_pos_tail = rev2rad(state_tail.values[moteus.Register.POSITION])
        meas_vel_tail = rev2rad(state_tail.values[moteus.Register.VELOCITY])
        meas_tau_tail = state_tail.values[moteus.Register.TORQUE]
    if state_phi is None:
        meas_pos_phi = prev_meas_pos_phi
        meas_vel_phi = prev_meas_vel_phi
        meas_tau_phi = prev_meas_tau_phi
        print("*")
        print(f"Phi missed a beat at index {index}")
        print("*")
    else:
        meas_pos_phi = rev2rad(state_phi.values[moteus.Register.POSITION])
        meas_vel_phi = rev2rad(state_phi.values[moteus.Register.VELOCITY])
        meas_tau_phi = state_phi.values[moteus.Register.TORQUE]
    return meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi

async def send_rad_command_one_motor(
    controller_obj=None,
    pos=None,
    vel=None,
    tau=None,
    tau_limit=None,
    kp_scale=1,
    kd_scale=1,
    watchdog_timeout=None,
):
    state = await controller_obj.set_position(
        position=rad2rev(pos),  # 0,#
        velocity=rad2rev(vel),  # 0,#
        kp_scale=kp_scale,
        kd_scale=kd_scale,
        stop_position=None,
        feedforward_torque=tau,
        maximum_torque=tau_limit,
        watchdog_timeout=watchdog_timeout,
        query=True,
    )
    # store data
    meas_pos = rev2rad(state.values[moteus.Register.POSITION])
    meas_vel = rev2rad(state.values[moteus.Register.VELOCITY])
    meas_tau = state.values[moteus.Register.TORQUE]
    return meas_pos, meas_vel, meas_tau



async def read_motor_data(controller_obj=None):
    state = await controller_obj.set_position(
        kp_scale=0, kd_scale=0, query=True
    )
    meas_pos = rev2rad(state.values[moteus.Register.POSITION])
    meas_vel = rev2rad(state.values[moteus.Register.VELOCITY])
    meas_tau = state.values[moteus.Register.TORQUE]
    return meas_pos, meas_vel, meas_tau
    
async def read_two_motors_data(controller_obj_tail=None, controller_obj_phi=None):
    state_tail = await controller_obj_tail.set_position(
        kp_scale=0, kd_scale=0, query=True
    )
    state_phi = await controller_obj_phi.set_position(
        kp_scale=0, kd_scale=0, query=True
    )
    meas_pos_tail = rev2rad(state_tail.values[moteus.Register.POSITION])
    meas_vel_tail = rev2rad(state_tail.values[moteus.Register.VELOCITY])
    meas_tau_tail = state_tail.values[moteus.Register.TORQUE]
    meas_pos_phi = rev2rad(state_phi.values[moteus.Register.POSITION])
    meas_vel_phi = rev2rad(state_phi.values[moteus.Register.VELOCITY])
    meas_tau_phi = state_phi.values[moteus.Register.TORQUE]
    return meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi
    
async def read_two_motors_data_w_check(
    controller_obj_tail=None, 
    controller_obj_phi=None,    
    prev_meas_pos_tail=None,
    prev_meas_vel_tail=None,
    prev_meas_tau_tail=None,
    prev_meas_pos_phi=None,
    prev_meas_vel_phi=None,
    prev_meas_tau_phi=None,):
        
    state_tail = await controller_obj_tail.set_position(
        kp_scale=0, kd_scale=0, query=True
    )
    state_phi = await controller_obj_phi.set_position(
        kp_scale=0, kd_scale=0, query=True
    )
    #CHANGE THE FOLLOWING IF THERE IS A BETTER WAY
    if state_tail is None:
        meas_pos_tail = prev_meas_pos_tail
        meas_vel_tail = prev_meas_vel_tail
        meas_tau_tail = prev_meas_tau_tail
        print("*")
        print(f"Tail missed a beat at index {index}")
        print("*")
    else:
        meas_pos_tail = rev2rad(state_tail.values[moteus.Register.POSITION])
        meas_vel_tail = rev2rad(state_tail.values[moteus.Register.VELOCITY])
        meas_tau_tail = state_tail.values[moteus.Register.TORQUE]
    #print("read tail")
    if state_phi is None:
        meas_pos_phi = prev_meas_pos_phi
        meas_vel_phi = prev_meas_vel_phi
        meas_tau_phi = prev_meas_tau_phi
        print("*")
        print(f"Phi missed a beat at index {index}")
        print("*")
    else:
        meas_pos_phi = rev2rad(state_phi.values[moteus.Register.POSITION])
        meas_vel_phi = rev2rad(state_phi.values[moteus.Register.VELOCITY])
        meas_tau_phi = state_phi.values[moteus.Register.TORQUE]
    #print("read phi")
    return meas_pos_tail, meas_vel_tail, meas_tau_tail, meas_pos_phi, meas_vel_phi, meas_tau_phi


async def read_imu_data(pr):
    imu = await pr.cycle([], request_attitude=True)
    imu_data = imu[0]
    quat_wxyz = (
        imu_data.attitude.w,
        imu_data.attitude.x,
        imu_data.attitude.y,
        imu_data.attitude.z,
    )
    vel_xyz = imu_data.rate_dps.x, imu_data.rate_dps.y, imu_data.rate_dps.z
    acc_xyz = (
        imu_data.accel_mps2.x,
        imu_data.accel_mps2.y,
        imu_data.accel_mps2.z,
    )
    euler_xyz = (
        imu_data.euler_rad.roll,
        imu_data.euler_rad.pitch,
        imu_data.euler_rad.yaw,
    )
    return quat_wxyz, vel_xyz, acc_xyz, euler_xyz


def imu_zero_offset(
    imu_reading, imu_init
):  # FIXME This creates bugs and wrapper problem
    if np.sign(imu_reading) == np.sign(imu_reading):
        return imu_reading - imu_init
    else:
        return imu_reading + imu_init


def determin_sign(roll, ax):
    sign = int(np.sign(roll))
    if int(np.sign(roll)) * int(np.sign(ax)) < 0:
        sign = int(np.sign(ax))
    return sign


def quat2angle(w, x, y, z):
    denom = math.sqrt(1 - w * w)
    ax = x / denom
    ay = y / denom
    az = z / denom
    angle_axis = 2 * math.acos(w)
    return (angle_axis, ax, ay, az, denom)


async def state_estimation(pr, pos_tail, vel_tail): #Changes made to state_estimation to fit to RicMonk.
    # print('[state_estimation]')  #Commenting the following limit related lines for testing changes between odd and even brachiations
    #if abs(pos_tail) > (np.pi):
    #    raise Exception(f"tail pos={pos_tail} out of range [-pi, pi]")
    quat_wxyz, vel_xyz, acc_xyz, euler_xyz = await read_imu_data(pr)
    #print(f'euler_xyz[0]: {euler_xyz[0]}')
    alpha_tail = euler_xyz[0]
    omega_tail, _, _ = np.deg2rad(vel_xyz)

    pos_arm1 = alpha_tail - pos_tail
    vel_arm1 = omega_tail -vel_tail

    #return pos_arm1, vel_arm1, euler_xyz, np.deg2rad(vel_xyz)  #CHANGED THIS!!!
    return pos_arm1, vel_arm1, alpha_tail, omega_tail
    
async def state_estimation_even(pr, pos_tail, vel_tail):
    # print('[state_estimation_v2]')  #Commenting the following limit related lines for testing changes between odd and even brachiations
    #if abs(pos_tail) > (np.pi):
    #    raise Exception(f"tail pos={pos_tail} out of range [-pi, pi]")
    quat_wxyz, vel_xyz, acc_xyz, euler_xyz = await read_imu_data(pr)
    #print(f'euler_xyz[0]: {euler_xyz[0]}')
    alpha_tail = -euler_xyz[0]
    omega_tail, _, _ = -np.deg2rad(vel_xyz)

    pos_arm1 = alpha_tail - pos_tail
    vel_arm1 = omega_tail -vel_tail

    #return pos_arm1, vel_arm1, euler_xyz, np.deg2rad(vel_xyz)  #CHANGED THIS!!!
    return pos_arm1, vel_arm1, alpha_tail, omega_tail


# async def state_estimation(pr, pos_el, vel_el):
#     # print('[state_estimation]')
#     if abs(pos_el) > (np.pi):
#         raise Exception(f"Elbow pos={pos_el} out of range [-pi, pi]")
#     quat_wxyz, vel_xyz, acc_xyz, euler_xyz = await read_imu_data(pr)
#     th_imu = euler_xyz[0]
#     omega_x, _, _ = np.deg2rad(vel_xyz)

#     th_imu_0_to_2pi = th_imu + np.pi
#     th_2_0_to_2pi = pos_el + np.pi
#     th_1 = np.mod(np.pi - th_2_0_to_2pi + th_imu_0_to_2pi, 2 * np.pi) - np.pi
#     th_1_dot = omega_x - vel_el

#     return th_1, th_1_dot, euler_xyz, np.deg2rad(vel_xyz)


""" # Working Version!!!!
async def state_estimation(pr, pos_el, vel_el, imu_init):
    if abs(pos_el) > (np.pi):
        raise Exception(f"Elbow pos={pos_el} out of range [-2pi,2pi]")
    quat_wxyz, vel_xyz, acc_xyz, euler_xyz = await read_imu_data(pr)
    #w, x, y, z = quat_wxyz
    omega_x, _, _ = np.deg2rad(vel_xyz)
    #_, ax, _, _, _ = quat2angle(w, x, y, z)
    raw_imu = euler_xyz[0]
    '''
    sign = determin_sign(raw_imu, ax)
    if sign > 0:
       es_theta1 = wrap_z2pi(raw_imu) - pos_el
    elif sign == 0:
       es_theta1 = 0
    elif sign < 0:
       es_theta1 = (-2 * np.pi + wrap_z2pi(raw_imu)) - pos_el
    if abs(es_theta1) > np.deg2rad(358):
       es_theta1 = 0
       '''
    es_theta1 = raw_imu - pos_el
    '''
    if abs(imu_init) < np.deg2rad(5):
       es_theta1 = imu_zero_offset(es_theta1, imu_init)
    '''
    es_theta1_dot = omega_x - vel_el
    return es_theta1, es_theta1_dot, raw_imu, omega_x
"""


def wrap_pi2pi(angle):
    # This is a function that wraps any angle in [-pi, pi]
    angle = angle % (2 * np.pi)
    if angle > np.pi:
        angle -= 2 * np.pi
    return angle


def wrap_z2pi(angle):
    # This is a function that wraps any angle in [0, 2 * pi]
    if angle < 0:
        return angle + np.ceil(-angle / (2 * np.pi)) * 2 * np.pi
    elif angle > 2 * np.pi:
        return angle - (np.ceil(angle / (2 * np.pi)) - 1) * 2 * np.pi

    return angle

def print_val(**kwargs):
    for name, value in kwargs.items():
        print("%s = \n%s" % (name, repr(value)))


def get_theta1(imu_angle, theta2):

    return wrap_pi2pi(wrap_z2pi(imu_angle - theta2))


def quaternion_to_euler(w, x, y, z):
    t0 = 2 * (w * x + y * z)
    t1 = 1 - 2 * (x * x + y * y)
    X = math.atan2(t0, t1)

    t2 = 2 * (w * y - z * x)
    t2 = 1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    Y = math.asin(t2)

    t3 = 2 * (w * z + x * y)
    t4 = 1 - 2 * (y * y + z * z)
    Z = math.atan2(t3, t4)
    return X, Y, Z


def quat_magnitude(q):
    w, x, y, z = q
    return (w ** 2 + x ** 2 + y ** 2 + z ** 2) ** 0.5


def quat_dot(q1, q2):
    q1q2 = tuple(l * r for l, r in zip(q1, q2))
    return q1q2


def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


# def qp_mult(q1, p1):
#     q2 = (0.0,) + p1
#     return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]


def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z


def quaternion_to_euler(quat_tuple):
    w, x, y, z = quat_tuple
    t0 = 2 * (w * x + y * z)
    t1 = 1 - 2 * (x * x + y * y)
    X = math.atan2(t0, t1)

    t2 = 2 * (w * y - z * x)
    t2 = 1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    Y = math.asin(t2)

    t3 = 2 * (w * z + x * y)
    t4 = 1 - 2 * (y * y + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z


def quat_magnitude(q):
    w, x, y, z = q
    return (w ** 2 + x ** 2 + y ** 2 + z ** 2) ** 0.5


def quat_dot(q1, q2):
    q1q2 = tuple(l * r for l, r in zip(q1, q2))
    return q1q2


def get_angle(q1, q2):
    q1q2 = quat_dot(q1, q2)
    return 2 * math.acos(sum(q1q2) / (quat_magnitude(q1) * quat_magnitude(q2)))


def get_angle_atan2(q1, q2):
    q1 = np.asarray(q1)
    q2 = np.asarray(q2)
    cos_theta = q1.dot(q2) / (np.linalg.norm(q1) * np.linalg.norm(q2))
    sin_theta = math.sqrt(1 - cos_theta ** 2)
    return 2 * math.atan2(sin_theta, cos_theta)


def plot_analysis_data(data, laptop_timeout, rpi_timeout):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
    flags = data["flag"]
    waitings = np.reshape(np.where(flags == 1), -1)
    fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(15, 10))
    ax[0].plot(data["time"], data["meas_pos"], label="meas")
    ax[0].plot(data["time"], data["des_pos"], label="des")
    ax[0].legend()
    ax[1].plot(data["time"], data["latency"], label="latency")
    ax[1].legend()
    # trans = mtransforms.blended_transform_factory(ax.transData, ax.transAxes)
    # ax[0].fill_between(data["time"],max(abs(data["des_pos"])) + 0.5, where=data["flag"]>0,
    #                 facecolor='red', alpha=0.5, transform=ax[0].get_xaxis_transform())
    ax[0].set_xlabel("time")
    ax[0].set_ylabel("position (rad)")
    ax[1].set_xlabel("time")
    ax[1].set_ylabel("latency")
    ax[0].set_title(
        f"Position Plot: laptop_timeout= {laptop_timeout}, rpi_timeout = {rpi_timeout}"
    )
    for i in waitings:
        for j in range(2):
            ax[j].axvline(data["time"][i], linestyle=":", color="indianred")
    fig.savefig(
        "timeout_results/"
        + date
        + f"_pos_lp_{laptop_timeout}_rpi_{rpi_timeout}.pdf"
    )

    fig, axs = plt.subplots(nrows=2, ncols=2, figsize=(15, 10))
    plt.suptitle(
        f"Laptop timeout = {laptop_timeout}/RPI timeout = {rpi_timeout}"
    )

    axs[0, 0].set_title("main cycle")
    axs[0, 0].scatter(data["cmd_id"], data["rpi_main_cycle_dt"], s=8)  # ,
    #   label=f'rpi: mean={np.mean(data["rpi_main_cycle_dt"]):.6f} s= {round(1/np.mean(data["rpi_main_cycle_dt"]))} Hz')
    axs[0, 0].scatter(data["cmd_id"], data["laptop_main_cycle_dt"], s=8)  # ,
    #   label=f'laptop: mean={np.mean(data["laptop_main_cycle_dt"]):.6f} s= {round(1/np.mean(data["laptop_main_cycle_dt"]))} Hz')
    axs[0, 0].set_ylabel("dt")
    axs[0, 0].set_xlabel("cmd_id")
    # axs[0, 0].legend()

    axs[0, 1].set_title("subscription")
    axs[0, 1].scatter(data["cmd_id"], data["rpi_sub_dt"], s=8)  # ,
    #   label=f'rpi: mean={np.mean(data["rpi_sub_dt"]):.6f} s')
    axs[0, 1].scatter(data["cmd_id"], data["laptop_sub_dt"], s=1)  # ,
    #   label=f'laptop: mean={np.mean(data["laptop_sub_dt"]):.6f} s')
    axs[0, 1].set_ylabel("dt")
    axs[0, 1].set_xlabel("cmd_id")
    # axs[0, 1].legend()

    axs[1, 0].set_title("publish")
    axs[1, 0].scatter(data["cmd_id"], data["rpi_pub_dt"], s=8)  # ,
    #   label=f'rpi: mean={np.mean(data["rpi_pub_dt"]):.6f} s')
    axs[1, 0].scatter(data["cmd_id"], data["laptop_pub_dt"], s=1)  # ,
    # label=f'laptop: mean={np.mean(data["laptop_pub_dt"]):.6f} s')
    axs[1, 0].set_ylabel("dt")
    axs[1, 0].set_xlabel("cmd_id")
    # axs[1, 0].legend()
    axs[1, 1].set_title("rpi execution")
    axs[1, 1].scatter(data["cmd_id"], data["rpi_exec_dt"], s=8)  # ,
    #   label=f'rpi: mean={np.mean(data["rpi_exec_dt"]):.6f} s')
    axs[1, 1].set_ylabel("dt")
    axs[1, 1].set_xlabel("cmd_id")
    # axs[1, 1].legend()
    fig.tight_layout()
    for i in waitings:
        for j in range(2):
            for k in range(2):
                axs[j, k].axvline(
                    data["cmd_id"][i], linestyle=":", color="indianred"
                )
    fig.savefig(
        "timeout_results/"
        + date
        + f"_analysis_p_{laptop_timeout}_rpi_{rpi_timeout}.pdf"
    )

    plt.show()


def fit_polynomial(data):
    """
    This function takes a data as input and fit a polynomial of degree 1 to the torque and
    a cubic one to state, and derivative of order 1 and 2 of states and returns the polynomials
    """
    csv_data = prepare_des_data(data)
    csv_data.shoulder_des_pos
    elbow_des_tau = csv_data.elbow_des_tau.reshape(
        csv_data.elbow_des_tau.shape[0], -1
    ).T
    des_time = csv_data.des_time.reshape(csv_data.des_time.shape[0], -1)
    x0_desc = np.vstack(
        (
            csv_data.shoulder_des_pos,
            csv_data.elbow_des_pos,
            csv_data.shoulder_des_vel,
            csv_data.elbow_des_vel,
        )
    )
    u0_desc = elbow_des_tau
    u0 = PiecewisePolynomial.FirstOrderHold(des_time, u0_desc)
    x0 = PiecewisePolynomial.CubicShapePreserving(
        des_time, x0_desc, zero_end_point_derivatives=True
    )
    x0_d = x0.derivative(derivative_order=1)
    x0_dd = x0.derivative(derivative_order=2)
    return x0, u0, x0_d, x0_dd


def extract_data_from_polynomial(polynomial, frequency):
    n_points = int(polynomial.end_time() / (1 / frequency))
    time_traj = np.linspace(
        polynomial.start_time(), polynomial.end_time(), n_points
    )
    extracted_time = time_traj.reshape(n_points, 1).T
    extracted_data = np.hstack(
        [
            polynomial.value(t)
            for t in np.linspace(
                polynomial.start_time(), polynomial.end_time(), n_points
            )
        ]
    )
    return extracted_data, extracted_time


def ff_acromonk(theta1, theta2, theta1_dot, theta2_dot):
    l1 = 0.31401  # 0.32
    l2 = l1
    ee_y = (l1 * np.sin(theta1)) + (l2 * np.sin(theta1 + theta2))
    ee_y_dot = (l1 * theta1_dot * np.cos(theta1)) + (
        l2 * (theta1_dot + theta2_dot) * np.cos(theta1 + theta2)
    )
    ee_z = -l1 * np.cos(theta1) - (l2 * np.cos(theta1 + theta2))
    ee_z_dot = (l1 * theta1_dot * np.sin(theta1)) - (
        l2 * (theta1_dot + theta2_dot) * np.sin(theta1 + theta2)
    )
    return ee_y, ee_z, ee_y_dot, ee_z_dot

def forward_kinematics(theta1, theta2):
    '''
    Function to compute the forward kinematics of the AcroMonk Robot,
    i.e. compute the end-effector position given the joint angles.
    Returns ons the (y,z) coordinates as the AcroMonk can only move in a plane.
    '''
    l1 = 0.31401  # 0.32
    l2 = l1
    ee_y = (l1 * np.sin(theta1)) + (l2 * np.sin(theta1 + theta2))
    ee_z = -(l1 * np.cos(theta1) + (l2 * np.cos(theta1 + theta2)))

    return ee_y, ee_z

def forward_diff_kinematics(theta1, theta2, theta1_dot, theta2_dot):
    '''
    Function to compute the differential forward kinematics of the AcroMonk Robot,
    i.e. compute the end-effector velocity given the join position/velocities
    Returns ons the (y_dot,z_dot) coordinates as the AcroMonk can only move in a plane.
    '''
    l1 = 0.31401  # 0.32
    l2 = l1
    ee_y_dot = (l1 * theta1_dot * np.cos(theta1)) + (l2 * (theta1_dot + theta2_dot) * np.cos(theta1 + theta2))
    ee_z_dot = (l1 * theta1_dot * np.sin(theta1)) -(l2 * (theta1_dot + theta2_dot) * np.sin(theta1 + theta2))
    return ee_y_dot, ee_z_dot
    

def save_dict(dict, path):
    with open(path, "w") as f:
        for key, value in dict.items():
            f.write(f"{key}={value}\n")
    print(f"Dictonary saved in: {path}.")
    with open(path, "r") as f:
        print(f"Dictonary:\n{f.read()}")


def trajopt(
    plant,
    context,
    n,
    tau_limit,
    initial_state,
    theta_limit,
    speed_limit,
    ladder_distance,
    final_state,
    R,
    time_panalization,
    init_guess,
):
    min_timestep = 0.1
    max_timestep = 0.8
    dircol = DirectCollocation(
        plant,
        context,
        num_time_samples=n,
        minimum_timestep=min_timestep,
        maximum_timestep=max_timestep,
        input_port_index=plant.get_actuation_input_port().get_index(),
    )
    dircol.AddEqualTimeIntervalsConstraints()
  
  
    ## Constraints


    # Initial Torque
    torque_limit = tau_limit  # N*m.
    u_init = dircol.input(0)
    dircol.AddConstraintToAllKnotPoints(u_init[0] == 0)
    # Torque limit
    u = dircol.input()
    dircol.AddConstraintToAllKnotPoints(-torque_limit <= u[0])
    dircol.AddConstraintToAllKnotPoints(u[0] <= torque_limit)
    initial_state = initial_state

    # Speed limit
    prog = dircol.prog()
    prog.AddBoundingBoxConstraint(
        initial_state, initial_state, dircol.initial_state()
    )
    state = dircol.state()
    speed_limit = speed_limit
    dircol.AddConstraintToAllKnotPoints(state[2] <= speed_limit)
    dircol.AddConstraintToAllKnotPoints(-speed_limit <= state[2])
    dircol.AddConstraintToAllKnotPoints(state[3] <= speed_limit)
    dircol.AddConstraintToAllKnotPoints(-speed_limit <= state[3])

    # Collision avoidance for front bar
    ee_y, ee_z = forward_kinematics(state[0], state[1])
    ee_y_dot, ee_z_dot = forward_diff_kinematics(state[0], state[1], state[2], state[3])
    front_bar_coordinate = [ladder_distance, 0.0] #[y, z]
    R_bar_radius = 0.01#0.001   #assumed as the radius of the ladder branch 
    distanceR = np.sqrt((front_bar_coordinate[0]-ee_y)**2 + (front_bar_coordinate[1]-ee_z)**2)
    dircol.AddConstraintToAllKnotPoints(R_bar_radius <= distanceR)

    # Collision avoidance for back bar
    ee_y, ee_z = forward_kinematics(state[0], state[1])
    ee_y_dot, ee_z_dot = forward_diff_kinematics(state[0], state[1], state[2], state[3])
    back_bar_coordinate = [-1 * ladder_distance, 0.0] #[y, z]    
    distanceL = np.sqrt((back_bar_coordinate[0]-ee_y)**2 + (back_bar_coordinate[1]-ee_z)**2)
    dircol.AddConstraintToAllKnotPoints(R_bar_radius <= distanceL)

    # Elbow angle limits for collision avoidance to attatched bar
    theta_limit = theta_limit
    # Shoulder bounds
    dircol.AddConstraintToAllKnotPoints(state[0] <= theta_limit[0])
    dircol.AddConstraintToAllKnotPoints(-theta_limit[0] <= state[0])
    # Elbow bounds
    dircol.AddConstraintToAllKnotPoints(state[1] <= theta_limit[1])
    dircol.AddConstraintToAllKnotPoints(-theta_limit[1] <= state[1])    
    final_state = final_state
    prog.AddBoundingBoxConstraint(
        final_state, final_state, dircol.final_state()
    )


    ## Costs
    
    # Input 
    dircol.AddRunningCost(R * u[0] ** 2)

    # Velocities
    dircol.AddRunningCost(state[2] ** 2)
    dircol.AddRunningCost(state[3] ** 2)

    # Total duration
    dircol.AddFinalCost(dircol.time() * time_panalization)
    # dircol.AddFinalCost(np.sum(dircol.time()) * time_panalization)

    # Initial guess
    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
        init_guess, np.column_stack((initial_state, final_state))
    )
    dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)
    result = Solve(prog)
    assert result.is_success()
    hyper_params_dict = {
        "n": n,
        "tau_limit": tau_limit,
        "initial_state": initial_state,
        "theta_limit": theta_limit,
        "speed_limit": speed_limit,
        "ladder_distance": ladder_distance,
        "final_state": final_state,
        "R": R,
        "time_panalization": time_panalization,
        "init_guess": init_guess,
        "max_timestep": max_timestep,
        "min_timestep": min_timestep,
    }
    return result, dircol, hyper_params_dict


class FitPiecewisePolynomial:
    """
    Gets data and number of break points and
    fit cubic segment polynomials to each section of data
    """

    def __init__(self, data_x, data_y, num_break, poly_degree):
        self.data_x = data_x
        self.data_y = data_y
        self.num_break = num_break
        self.poly_degree = poly_degree
        (
            self.x_sec_data,
            self.y_sec_data,
            self.coeff_sec_data,
            self.t0_polys,
            self.tf_polys,
        ) = self.create_section_poly()

    def determin_poly(self):
        if self.poly_degree == 1:
            return self.poly1
        elif self.poly_degree == 2:
            return self.poly2
        elif self.poly_degree == 3:
            return self.poly3
        else:
            print('Choose between "1,2,3" for the degree of the polynomial')
            return None

    def poly1(self, t, A, B):
        return A * pow(t, 1) + B

    def poly2(self, t, A, B, C):
        return A * pow(t, 2) + B * pow(t, 1) + C

    def poly3(self, t, A, B, C, D):
        return A * pow(t, 3) + B * pow(t, 2) + C * pow(t, 1) + D

    def end_time(self):
        return self.data_x[-1]

    def start_time(self):
        return self.data_x[0]

    def split_data(self, data):
        """
        Takes the original data and return a list of splitted arrays
        """
        l = len(data)
        sec_len = int(np.ceil(l / self.num_break))
        #         if l % self.num_break != 0 :
        #             print(f'Uneven division.len={l},section_len={sec_len}, last_sec_len={l - (sec_len * (self.num_break - 1))}')
        return np.array_split(data, self.num_break)

    def create_section_poly(self):
        """
        This function takes the splitted data(x, y) and return 2 lists
        - list of the x-data to be fitted to the setion data
        - list of the fitted value
        """
        splitted_data_x = self.split_data(self.data_x)
        splitted_data_y = self.split_data(self.data_y)
        x_sec_data = []
        y_sec_data = []
        coeff_sec_data = []
        index = 0
        for sec in splitted_data_x:
            x_sec_data.append(np.linspace(sec[0], sec[-1], len(sec)))
            func = self.determin_poly()
            p_coeff, p_cov = cf(
                func, splitted_data_x[index], splitted_data_y[index]
            )
            fit = func(x_sec_data[index], *p_coeff)
            y_sec_data.append(fit)
            coeff_sec_data.append(p_coeff)
            index += 1
        t0_polys = np.asarray([sec_t[0] for sec_t in x_sec_data])
        tf_polys = np.asarray([sec_t[-1] for sec_t in x_sec_data])
        self.x_sec_data = x_sec_data
        self.y_sec_data = y_sec_data
        self.coeff_sec_data = coeff_sec_data
        self.t0_polys = t0_polys
        self.tf_polys = tf_polys
        return x_sec_data, y_sec_data, coeff_sec_data, t0_polys, tf_polys

    def get_value(self, time_stamp):

        if time_stamp > self.data_x[-1]:
            print(f"time exceeded,time={time_stamp}, value={self.data_y[-1]}")
            return self.data_y[-1]
        else:
            poly_index = len(
                np.where(np.array(time_stamp <= self.tf_polys) == False)[0]
            )
            # poly_index = min(
            #     [
            #         index
            #         for index, element in enumerate(
            #             [any(poly >= time_stamp) for poly in self.x_sec_data]
            #         )
            #         if element == True
            #     ]
            # )

            p_coeff = self.coeff_sec_data[poly_index]
            func = self.determin_poly()
            return func(time_stamp, *p_coeff)


def create_gain_arrays(tvlqr_obj, frequency):
    n_points = int(tvlqr_obj.K.end_time() / (1 / frequency))
    time_stamp = np.linspace(
        tvlqr_obj.K.start_time(), tvlqr_obj.K.end_time(), n_points
    )
    K = [tvlqr_obj.K.value(i) for i in time_stamp]
    index = 0
    K_array = np.zeros((n_points, 4))
    for k_vec in K:
        K_array[index] = k_vec[0]
        index += 1
    k0 = np.array([tvlqr_obj.k0.value(i)[0][0] for i in time_stamp])
    return K_array, k0, time_stamp


def create_nominal_pcws(
    data,
):  # changed to increase the control loop frequency
    breaks = 25  # 70
    poly_degree_states = 2  # 3
    arm1_des_Ang_pos_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.arm1_des_Ang_pos,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    arm1_des_Ang_vel_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.arm1_des_Ang_vel,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    tail_des_Ang_pos_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.tail_des_Ang_pos,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    tail_des_Ang_vel_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.tail_des_Ang_vel,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    tail_des_torque_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.tail_des_torque,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    tail_des_torque_tvlqr_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.tail_des_torque_tvlqr,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )   
    phi_des_Ang_pos_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.phi_des_Ang_pos,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    phi_des_Ang_vel_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.phi_des_Ang_vel,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    phi_des_torque_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.phi_des_torque,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    phi_des_torque_tvlqr_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.phi_des_torque_tvlqr,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    arm2_des_Ang_pos_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.arm2_des_Ang_pos,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    arm2_des_Ang_vel_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.arm2_des_Ang_vel,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    num_break = 25  # 100
    poly_deg = 2#03

    k0 = FitPiecewisePolynomial(
        data_y=data.k0,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k1 = FitPiecewisePolynomial(
        data_y=data.k1,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k2 = FitPiecewisePolynomial(
        data_y=data.k2,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k3 = FitPiecewisePolynomial(
        data_y=data.k3,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k4 = FitPiecewisePolynomial(
        data_y=data.k4,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )
    k5 = FitPiecewisePolynomial(
        data_y=data.k5,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k6 = FitPiecewisePolynomial(
        data_y=data.k6,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )
    k7 = FitPiecewisePolynomial(
        data_y=data.k7,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k8 = FitPiecewisePolynomial(
        data_y=data.k8,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k9 = FitPiecewisePolynomial(
        data_y=data.k9,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k10 = FitPiecewisePolynomial(
        data_y=data.k10,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )
    k11 = FitPiecewisePolynomial(
        data_y=data.k11,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k12 = FitPiecewisePolynomial(
        data_y=data.k12,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )
    PCWS = namedtuple(
        "PCWS",
        [
            "k0",
            "k1",
            "k2",
            "k3",
            "k4",
            "k5",
            "k6",
            "k7",
            "k8",
            "k9",
            "k10",
            "k11",
            "k12",
            "arm1_des_Ang_pos_pcw",
            "tail_des_Ang_pos_pcw",
            "phi_des_Ang_pos_pcw",
            "arm2_des_Ang_pos_pcw",
            "arm1_des_Ang_vel_pcw",
            "tail_des_Ang_vel_pcw",
            "phi_des_Ang_vel_pcw",
            "arm2_des_Ang_vel_pcw",
            "tail_des_torque_pcw",
            "tail_des_torque_tvlqr_pcw",
            "phi_des_torque_pcw",
            "phi_des_torque_tvlqr_pcw",
        ],
    )
    pcws = PCWS(
        k0=k0,
        k1=k1,
        k2=k2,
        k3=k3,
        k4=k4,
        k5=k5,
        k6=k6,
        k7=k7,
        k8=k8,
        k9=k9,
        k10=k10,
        k11=k11,
        k12=k12,
        arm1_des_Ang_pos_pcw=arm1_des_Ang_pos_pcw,
        tail_des_Ang_pos_pcw=tail_des_Ang_pos_pcw,
        phi_des_Ang_pos_pcw=phi_des_Ang_pos_pcw,
        arm2_des_Ang_pos_pcw=arm2_des_Ang_pos_pcw,
        arm1_des_Ang_vel_pcw=arm1_des_Ang_vel_pcw,
        tail_des_Ang_vel_pcw=tail_des_Ang_vel_pcw,
        phi_des_Ang_vel_pcw=phi_des_Ang_vel_pcw,
        arm2_des_Ang_vel_pcw=arm2_des_Ang_vel_pcw,
        tail_des_torque_pcw=tail_des_torque_pcw,
        tail_des_torque_tvlqr_pcw=tail_des_torque_tvlqr_pcw,
        phi_des_torque_pcw=phi_des_torque_pcw,
        phi_des_torque_tvlqr_pcw=phi_des_torque_tvlqr_pcw,
    )

    return pcws


def tau_tvlqr(
    nominal_pcws, brach_sign, des_states_array, meas_states_array, time
):
    # print(f'x0 = {x0}')
    xbar = meas_states_array - des_states_array  # x0
    # print(f'xbar = {xbar}')
    K = np.zeros([2,6])
    K[0,0] = nominal_pcws.k1.get_value(time)
    K[0,1] = nominal_pcws.k2.get_value(time)
    K[0,2] = nominal_pcws.k3.get_value(time)
    K[0,3] = nominal_pcws.k4.get_value(time)
    K[0,4] = nominal_pcws.k5.get_value(time)
    K[0,5] = nominal_pcws.k6.get_value(time)
    K[1,0] = nominal_pcws.k7.get_value(time)
    K[1,1] = nominal_pcws.k8.get_value(time)
    K[1,2] = nominal_pcws.k9.get_value(time)
    K[1,3] = nominal_pcws.k10.get_value(time)
    K[1,4] = nominal_pcws.k11.get_value(time)
    K[1,5] = nominal_pcws.k12.get_value(time)
    # print(f'K = {K}')
    # k0 = np.array([[nominal_pcws.k0.get_value(time)]])
    # print(f'k0 = {k0}')
    #u0 = brach_sign * np.array([[nominal_pcws.el_des_tau_pcw.get_value(time)]])
    u0 = np.vstack((nominal_pcws.tail_des_torque_pcw.get_value(time), nominal_pcws.phi_des_torque_pcw.get_value(time)))
    u0 = brach_sign * u0
    tau = u0 - np.dot(K, xbar)  # - k0
    # print(f'time={time}\nx0=\n{x0}, \nx=\n{meas_states_array}, \nK=\n{K}, \nu0=\n{u0},\ntau={tau}\n==================================================')
    return tau, K


def tau_tvlqr_subsequent(nominal_pcws, meas_states_array, time):
    # print('tau_tvlqr:start')
    x0 = np.array(
        [
            [-1 * nominal_pcws.sh_des_pos_pcw.get_value(time)],
            [-1 * nominal_pcws.el_des_pos_pcw.get_value(time)],
            [-1 * nominal_pcws.sh_des_vel_pcw.get_value(time)],
            [-1 * nominal_pcws.el_des_vel_pcw.get_value(time)],
        ]
    )

    """x0 = np.array([
                   [nominal_pcws.sh_des_pos_pcw.get_value(time)],
                   [-1 * nominal_pcws.el_des_pos_pcw.get_value(time)],
                   [nominal_pcws.sh_des_vel_pcw.get_value(time)],
                   [-1 * nominal_pcws.el_des_vel_pcw.get_value(time)],
                  ])"""
    # print(f'x0 = {x0}')
    xbar = meas_states_array - x0
    # print(f'xbar = {xbar}')
    K = np.array(
        [
            [
                nominal_pcws.k1.get_value(time),
                nominal_pcws.k2.get_value(time),
                nominal_pcws.k3.get_value(time),
                nominal_pcws.k4.get_value(time),
            ]
        ]
    )
    # print(f'K = {K}')
    # k0 = np.array([[nominal_pcws.k0.get_value(time)]])
    # print(f'k0 = {k0}')
    u0 = -1 * np.array([[nominal_pcws.el_des_tau_pcw.get_value(time)]])
    tau = u0 - np.dot(K, xbar)  # - k0
    return tau


def forward_kinematics(theta1, theta2):
    '''
    Function to compute the forward kinematics of the AcroMonk Robot,
    i.e. compute the end-effector position given the joint angles.
    Returns ons the (y,z) coordinates as the AcroMonk can only move in a plane.
    '''
    l1 = 0.31401#0.32
    l2 = l1    
    ee_y = (l1 * np.sin(theta1)) + (l2 * np.sin(theta1 + theta2))
    ee_z = -(l1 * np.cos(theta1) + (l2 * np.cos(theta1 + theta2)))

    return ee_y, ee_z


def forward_diff_kinematics(theta1, theta2, theta1_dot, theta2_dot):
    '''
    Function to compute the differential forward kinematics of the AcroMonk Robot,
    i.e. compute the end-effector velocity given the join position/velocities
    Returns ons the (y_dot,z_dot) coordinates as the AcroMonk can only move in a plane.
    '''
    l1 = 0.31401#0.32
    l2 = l1    
    ee_y_dot = (l1 * theta1_dot * np.cos(theta1)) + (l2 * (theta1_dot + theta2_dot) * np.cos(theta1 + theta2))
    ee_z_dot = (l1 * theta1_dot * np.sin(theta1)) -(l2 * (theta1_dot + theta2_dot) * np.sin(theta1 + theta2))
    return ee_y_dot, ee_z_dot


def plot_custom_data_with_dir(directory, data, show=True):
    plot_data(
        x_data=[
            data["time"],
            data["time"],
            data["time"],
            data["time"],
            data["time"],
            data["time"],
        ],
        y_data=[
            data["arm1_Ang_meas_pos"],
            data["arm1_Ang_pos"],
            data["tail_Ang_meas_pos"],
            data["tail_Ang_pos"],
            data["phi_Ang_meas_pos"],
            data["phi_Ang_pos"],
        ],
        labels=["Time (s)", "Position (rad)"],
        title="Position (rad) vs Time (s)",
        legends=["arm1_Ang_meas_pos", "arm1_Ang_pos", "tail_Ang_meas_pos", "tail_Ang_pos", "phi_Ang_meas_pos", "phi_Ang_pos"],
        linestyle=["-", "--", "-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred", "green", "springgreen"],
        save_name=directory + "/pos.pdf",
    )
    plot_data(
        x_data=[
            data["time"],
            data["time"],
            data["time"],
            data["time"],
            data["time"],
            data["time"],
        ],
        y_data=[
            data["arm1_Ang_meas_vel"],
            data["arm1_Ang_vel"],
            data["tail_Ang_meas_vel"],
            data["tail_Ang_vel"],
            data["phi_Ang_meas_vel"],
            data["phi_Ang_vel"],
        ],
        labels=["Time (s)", "Velocity (rad/s)"],
        title="Velocity (rad/s) vs Time (s)",
        legends=["arm1_Ang_meas_vel", "arm1_Ang_vel", "tail_Ang_meas_vel", "tail_Ang_vel", "phi_Ang_meas_vel", "phi_Ang_vel"],
        linestyle=["-", "--", "-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred", "green", "springgreen"],
        save_name=directory + "/vel.pdf",
    )

    plot_data(
        x_data=[
            data["time"], 
            data["time"],
            data["time"],
            data["time"],
            ],
        y_data=[
            data["tail_meas_torque"],
            data["tail_torque"],
            data["phi_meas_torque"],
            data["phi_torque"],
        ],
        labels=["Time (s)", "Torque (Nm)"],
        title="Torque (Nm) vs Time (s)",
        legends=["torque_meas_tail","torque_des_tail","torque_meas_phi","torque_des_phi"],
        linestyle=["-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred"],
        save_name=directory + "/tau.pdf",
    )
    if show:
        plt.show()

def calc_pd_tau_ricMonk(
    data,
    tau_limit,
    gear_ratio = 6
    ):
    des_tail_pos=data.tail_des_Ang_pos_store
    #des_tail_pos=data["tail_des_Ang_pos_store"]
    des_tail_vel=data.tail_des_Ang_vel_store
    des_phi_pos=data.phi_des_Ang_pos_store
    des_phi_vel=data.phi_des_Ang_vel_store

    tail_cmd_tau = data.tail_cmd_tau
    phi_cmd_tau = data.phi_cmd_tau
    tail_clipped_tau = data.tail_clipped_tau
    phi_clipped_tau = data.phi_clipped_tau

    meas_tail_pos = data.tail_meas_Ang_pos
    meas_tail_vel = data.tail_meas_Ang_vel
    meas_phi_pos = data.phi_meas_Ang_pos
    meas_phi_vel = data.phi_meas_Ang_vel
    flag = data.state_machine_flag

    Kp = 100
    Kd = 2
    swing_phase_index = np.where(flag==1)[0]

    tail_vel_error = des_tail_vel - meas_tail_vel
    tail_pos_error = des_tail_pos - meas_tail_pos
    phi_vel_error = des_phi_vel - meas_phi_vel
    phi_pos_error = des_phi_pos - meas_phi_pos

    tail_cmd_tau_pd = Kp * (tail_pos_error) + Kd * (tail_vel_error)
    tail_cmd_tau_pd /= gear_ratio
    phi_cmd_tau_pd = Kp * (phi_pos_error) + Kd * (phi_vel_error)
    phi_cmd_tau_pd /= gear_ratio

    tail_clipped_tau_pd = np.clip(tail_cmd_tau_pd, -tau_limit, tau_limit)
    phi_clipped_tau_pd = np.clip(phi_cmd_tau_pd, -tau_limit, tau_limit)

    for index in swing_phase_index:
        tail_cmd_tau[index] = tail_cmd_tau_pd[index]
        tail_clipped_tau[index] = tail_clipped_tau_pd[index]
        phi_cmd_tau[index] = phi_cmd_tau_pd[index]
        phi_clipped_tau[index] = phi_clipped_tau_pd[index]
        data.tail_cmd_tau[index] = tail_cmd_tau_pd[index]
        data.tail_clipped_tau[index] = tail_clipped_tau_pd[index]
        data.phi_cmd_tau[index] = phi_cmd_tau_pd[index]
        data.phi_clipped_tau[index] = phi_clipped_tau_pd[index]

    #data["tail_cmd_tau"] = tail_cmd_tau
    #data["tail_clipped_tau"] = tail_clipped_tau
    #data["phi_cmd_tau"] = phi_cmd_tau
    #data["phi_clipped_tau"] = phi_clipped_tau

    return data#, tail_cmd_tau, tail_clipped_tau, phi_cmd_tau, phi_clipped_tau

        
def plot_custom_data_with_dir_meas(directory, data, show=True):
    plot_data(
        x_data=[
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
        ],
        y_data=[
            data["arm1_meas_Ang_pos"],
            data["arm1_des_Ang_pos_store"],
            data["tail_meas_Ang_pos"],
            data["tail_des_Ang_pos_store"],
            data["phi_meas_Ang_pos"],
            data["phi_des_Ang_pos_store"],
        ],
        labels=["Time (s)", "Position (rad)"],
        title="Position (rad) vs Time (s)",
        legends=["arm1_Ang_meas_pos", "arm1_Ang_pos", "tail_Ang_meas_pos", "tail_Ang_pos", "phi_Ang_meas_pos", "phi_Ang_pos"],
        linestyle=["-", "--", "-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred", "green", "springgreen"],
        save_name=directory + "/pos.pdf",
    )
    plot_data(
        x_data=[
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
        ],
        y_data=[
            data["arm1_meas_Ang_vel"],
            data["arm1_des_Ang_vel_store"],
            data["tail_meas_Ang_vel"],
            data["tail_des_Ang_vel_store"],
            data["phi_meas_Ang_vel"],
            data["phi_des_Ang_vel_store"],
        ],
        labels=["Time (s)", "Velocity (rad/s)"],
        title="Velocity (rad/s) vs Time (s)",
        legends=["arm1_Ang_meas_vel", "arm1_Ang_vel", "tail_Ang_meas_vel", "tail_Ang_vel", "phi_Ang_meas_vel", "phi_Ang_vel"],
        linestyle=["-", "--", "-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred", "green", "springgreen"],
        save_name=directory + "/vel.pdf",
    )

    plot_data(
        x_data=[
            data["meas_time"], 
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            ],
        y_data=[
            data["tail_clipped_tau"],
            data["tail_des_tau_store"],
            data["phi_clipped_tau"],
            data["phi_des_tau_store"],
        ],
        labels=["Time (s)", "Torque (Nm)"],
        title="Torque (Nm) vs Time (s)",
        legends=["torque_meas_tail","torque_des_tail","torque_meas_phi","torque_des_phi"],
        linestyle=["-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred"],
        save_name=directory + "/tau.pdf",
    )
    '''
    rIpMinusTMAP = data["raw_imu_pos"] - data["tail_meas_Ang_pos"]
    mOtmapPlusRIP = -data["tail_meas_Ang_pos"] - data["raw_imu_pos"]
    plot_data(
        x_data=[
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
        ],
        y_data=[
            data["arm1_meas_Ang_pos"],
            data["arm1_des_Ang_pos_store"],
            data["tail_meas_Ang_pos"],
            data["tail_des_Ang_pos_store"],
            data["phi_meas_Ang_pos"],
            data["phi_des_Ang_pos_store"],
            data["raw_imu_pos"],
            rIpMinusTMAP,
            mOtmapPlusRIP
        ],
        labels=["Time (s)", "Position (rad)"],
        title="Position (rad) vs Time (s)",
        legends=["arm1_Ang_meas_pos", "arm1_Ang_pos", "tail_Ang_meas_pos", "tail_Ang_pos", "phi_Ang_meas_pos", "phi_Ang_pos", "raw_imu_pos", "rawIMUposMinusTailMeasAPos", "minusOfTailMeasAPosPlusRawIMUpos"],
        linestyle=["-", "--", "-", "--", "-", "--", "-", "--", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred", "green", "springgreen", "purple", "plum", "lightcoral"],
        save_name=directory + "/posComp.pdf",
    )
    if show:
        plt.show()
    '''
    
def plot_custom_data_with_dir_meas_release(directory, data, show=True):
    plot_data(
        x_data=[
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
        ],
        y_data=[
            data["arm1_meas_Ang_pos"],
            data["arm1_des_Ang_pos_store"],
            data["tail_meas_Ang_pos"],
            data["tail_des_Ang_pos_store"],
            data["phi_meas_Ang_pos"],
            data["phi_des_Ang_pos_store"],
        ],
        labels=["Time (s)", "Position (rad)"],
        title="Position (rad) vs Time (s)",
        legends=["arm1_Ang_meas_pos", "arm1_Ang_pos", "tail_Ang_meas_pos", "tail_Ang_pos", "phi_Ang_meas_pos", "phi_Ang_pos"],
        linestyle=["-", "--", "-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred", "green", "springgreen"],
        save_name=directory + "/pos.pdf",
    )
    plot_data(
        x_data=[
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
        ],
        y_data=[
            data["arm1_meas_Ang_vel"],
            data["arm1_des_Ang_vel_store"],
            data["tail_meas_Ang_vel"],
            data["tail_des_Ang_vel_store"],
            data["phi_meas_Ang_vel"],
            data["phi_des_Ang_vel_store"],
        ],
        labels=["Time (s)", "Velocity (rad/s)"],
        title="Velocity (rad/s) vs Time (s)",
        legends=["arm1_Ang_meas_vel", "arm1_Ang_vel", "tail_Ang_meas_vel", "tail_Ang_vel", "phi_Ang_meas_vel", "phi_Ang_vel"],
        linestyle=["-", "--", "-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred", "green", "springgreen"],
        save_name=directory + "/vel.pdf",
    )

    plot_data(
        x_data=[
            data["meas_time"], 
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            ],
        y_data=[
            data["tail_clipped_tau"],
            data["tail_des_tau_store"],
            data["phi_clipped_tau"],
            data["phi_des_tau_store"],
        ],
        labels=["Time (s)", "Torque (Nm)"],
        title="Torque (Nm) vs Time (s)",
        legends=["torque_meas_tail","torque_des_tail","torque_meas_phi","torque_des_phi"],
        linestyle=["-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred"],
        save_name=directory + "/tau.pdf",
    )


def make_results_directory(test):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
    folder_name = f"results/{test}"
    file_name = date
    directory = generate_path(folder_name, file_name, 1)
    os.makedirs(directory)
    return directory
    
def make_results_directory_multiple(test):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
    folder_name = f"results/multipleBrachiation_{test}"
    file_name = date
    directory = generate_path(folder_name, file_name, 1)
    os.makedirs(directory)
    return directory
    
def make_results_directory_test_traj(test, traj, brach):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
    folder_name = f"results/newGripper/{test}_{traj}_{brach}"
    file_name = date
    directory = generate_path(folder_name, file_name, 1)
    os.makedirs(directory)
    return directory


def plot_custom_data(data, test, show=True):
    directory = make_results_directory(test)
    plot_custom_data_with_dir(directory, data, show)
    return directory

def calc_pd_tau(
    des_pos,
    meas_pos,
    des_vel,
    meas_vel,
    gear_ratio = 6
    ):    
    Kp = 100
    Kd = 2
    vel_error = des_vel - meas_vel
    pos_error = des_pos - meas_pos
    tau_pd = Kp * (pos_error) + Kd * (vel_error)
    tau_pd /= gear_ratio
    return tau_pd 


def motor_test_parameters(
    test, nominal_pcws, brach_sign, meas_state, des_state, time, controller
):
    if test == "pd":
        kp_scale = 1.0
        kd_scale = 1.0
        #print(kp_scale)
        des_tail_pos = des_state[1]
        des_tail_vel = des_state[4]
        des_phi_pos = des_state[2]
        des_phi_vel = des_state[5]
        meas_tail_pos = meas_state[1]
        meas_tail_vel = meas_state[4]
        meas_phi_pos = meas_state[2]
        meas_phi_vel = meas_state[5]
        tau_tail_cmd = 0
        tau_phi_cmd = 0
    elif test == "ff_replay":
        u_tail_ff = nominal_pcws.tail_des_torque_tvlqr_pcw.get_value(time)  # feed-forward
        u_phi_ff = nominal_pcws.phi_des_torque_tvlqr_pcw.get_value(time)  # feed-forward
        kp_scale = 0
        kd_scale = 0
        tau_tail_cmd = u_tail_ff
        tau_phi_cmd = u_phi_ff
    elif test == "tvlqr":         #GOTTA CHANGE THIS. WILL DEPEND ON HOW MAHDI ASKS US TO IMPLEMENT tau_tvlqr
        u_tvlqr, K = tau_tvlqr(
            nominal_pcws=nominal_pcws,
            brach_sign=brach_sign,
            des_states_array=des_state,
            meas_states_array=meas_state,
            time=time,
        )
        #NEED u_tail_tvlqr & u_phi_tvlqr
        kp_scale = 0
        kd_scale = 0
        tau_tail_cmd = u_tvlqr[0]
        tau_phi_cmd = u_tvlqr[1]
    elif test == "rl":
        # flip elbow states for el controller in even brachiations
        #YEH TOH APUN KO EK SHABDH BHI NAHI PATHA
        if brach_sign == -1:
            meas_state = np.multiply(meas_state, [[1], [1], [-1], [1], [1], [-1]])  #[[1], [-1], [1], [-1]])
        u_rl = controller.get_torque(meas_state)
        kp_scale = 0
        kd_scale = 0
        tau_tail_cmd = u_rl #FOR NOW, LETTING BOTH "tau_tail_cmd" and "tau_phi_cmd" to be "u_rl"
        tau_phi_cmd = u_rl
    else:
        raise Exception("Wrong test input!")
    Data = namedtuple("Data", ["kp_scale", "kd_scale", "tau_tail_cmd", "tau_phi_cmd", "K_tvlqr"])  #SHOULD THERE BE 'K_tail_tvlqr' and 'K_phi_tvlqr'
    data = Data(
        kp_scale=kp_scale,
        kd_scale=kd_scale,
        tau_tail_cmd=tau_tail_cmd,
        tau_phi_cmd=tau_phi_cmd,
        K_tvlqr=K if test == "tvlqr" else np.zeros((2, 6)),
    )
    return data


async def go_to_pos(servo, final_pos, rad_per_cycle=0.003, tau_limit = 4):

    print('[go_to_pos]')
    await servo.set_stop()    
    (pos, _, _) = await read_motor_data(servo)    
    cmd_pos = pos
    sgn = -1 if pos > final_pos else 1
    try:
        while sgn * (final_pos - pos) > 0:
            cmd_pos += sgn * rad_per_cycle
            (pos,
            vel,
            tau) = await send_rad_command(
                controller_obj=servo,
                pos=cmd_pos,
                vel=0.0,
                tau_limit=tau_limit)
    finally:
        await servo.set_stop()

async def arms_attached(imu, servo, number_of_loops, brach_type):
    print('[arms_attached]')
    #await servo.set_stop() # suspicious! check if this cause detach!
    temp_array_ee_z=np.zeros(number_of_loops)
    temp_array_ee_y=np.zeros(number_of_loops)
    while True:#el_vel > 0.01 and all(np.deg2rad(omegas) > 0.01):
        #_, el_vel, _ = await read_motor_data(servo)
        _, omegas, _, _ = await read_imu_data(imu)
        omegas = abs(np.deg2rad(omegas))
        #if abs(el_vel) < 0.008:# and all((omegas) <= 0.008):
        if all((omegas) <= 0.05):
            #print(f'el={el_vel}, omegas={omegas}')
            #await servo.set_stop()
            break
        time.sleep(0.3)
    #print('The robot stopped the movement! Check started')
    #time.sleep(0.5)    
    index = 0
    while index < number_of_loops:
        #el_pos, el_vel, el_tau = await read_motor_data(servo)
        (el_pos, el_vel, el_tau) = await send_rad_command(
            controller_obj=servo,
            pos=0.0,  # although 0, kp = 0 gives pos no affect
            vel=0.0,  # although 0, kd = 0 gives vel no affect
            tau=0.0,
            tau_limit=1.,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=0.05,
        )        
        if brach_type == 'odd':
            # print('[arms_attached][state_estimation]')
            state = await state_estimation(
                    pr=imu,
                    pos_el=el_pos,
                    vel_el=el_vel
                )
        else:        
            # print('[arms_attached][state_estimation_v2]')
            state = await state_estimation_v2(imu)    
        (sh_pos, sh_vel, raw_imu_pos, raw_imu_vel) = state                            
        roll, pitch, yaw = raw_imu_pos          
        (ee_y, ee_z) = forward_kinematics(abs(sh_pos), abs(el_pos))
        if abs(pitch) >= 0.35 or abs(yaw) >= 0.35 or abs(ee_y) >0.5: 
            return 0 
        temp_array_ee_z[index] = ee_z
        temp_array_ee_y[index] = ee_y
        index += 1  
    conf = None
    if (abs(roll) < np.deg2rad(90.0)) or (roll > np.deg2rad(-170.0) and roll < np.deg2rad(-90.0)):
        conf = 1
    elif (abs(roll) >= np.deg2rad(170.0)) or (roll > np.deg2rad(90.0) and roll < np.deg2rad(170.0)):
        conf = 2
    #print(f'[detect_conf]:v{conf}, roll={np.rad2deg(roll)}')    
    if (np.mean(abs(temp_array_ee_z))) <= 0.02 and (np.mean(abs(temp_array_ee_y))) > 0.32 and (np.mean(abs(temp_array_ee_y))) < 0.4:
        #print(f'2:[ee_y,ee_z] = {ee_y,ee_z}')
        print('[arms_attached]ended')
        return 2, conf
    else:
        #print(f'1:[ee_y,ee_z] = {ee_y,ee_z}')
        print('[arms_attached]ended')
        return 1, conf

try:
    def create_imu_object(config_in_degree, convergence_delay):
        imu = moteus_pi3hat.Pi3HatRouter(
            mounting_deg={
                "roll": config_in_degree[0], 
                "pitch": config_in_degree[1], 
                "yaw": config_in_degree[2]}
        )
        time.sleep(convergence_delay)    
        return imu

    def create_servo_object(bus_number, motor_id):
        transport = moteus_pi3hat.Pi3HatRouter(
            servo_bus_map={
                bus_number: [motor_id],
            },
        )
        servo = moteus.Controller(id=motor_id, transport=transport)    
        return servo
except:
    pass
