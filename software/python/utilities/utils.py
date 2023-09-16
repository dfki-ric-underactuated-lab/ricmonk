import os
import pandas as pd
import numpy as np
from utils_polynomials import (
    fit_polynomial, 
    extract_data_from_polynomial, 
    PiecewisePolynomial, 
    create_gain_arrays
    )
# from sklearn.metrics import mean_squared_error

def parent(path):
    return os.path.dirname(path)
from math import sqrt



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


def save_dict(dict, path):
    with open(path, "w") as f:
        for key, value in dict.items():
            f.write(f"{key}={value}\n")
    print(f"Hyperparameters saved in: {path}.")
    with open(path, "r") as f:
        print(f"Hyperparameters:\n{f.read()}")


def save_data(data, path_to_save):
    header = ",".join(data.keys())
    data = np.vstack(data.values()).T
    np.savetxt(path_to_save, data, delimiter=",", header=header, comments="")
    print(f"saved trajectory csv to {path_to_save}")


# def save_trajectory(
#     maneuver, x_trajectory, u_trajectory, frequency, hyper_params, controller_type=None, controller=None, meas_state=None, meas_torque_tail=None, meas_torque_arm2=None, tail_des_torque_traj=None, arm2_des_torque_traj=None
# ):
def save_trajectory(
    maneuver, x_trajectory, u_trajectory, frequency, hyper_params, controller_type=None, controller=None, meas_state=None, meas_torque_tail=None, meas_torque_arm2=None, x0_w_arm2=None
):
    if controller_type==None:    
        x0_d = x_trajectory.derivative(derivative_order=1)
        x0_dd = x_trajectory.derivative(derivative_order=2)
        # Extract State
        ricmonk_state, time_traj = extract_data_from_polynomial(
            x_trajectory, frequency
        )
        # Extract xd_trajectory
        x0_d_vec, _ = extract_data_from_polynomial(x0_d, frequency)
        # Extract xdd_trajectory
        x0_dd_vec, _ = extract_data_from_polynomial(x0_dd, frequency)
        # Extract desired input
    #    if tail_des_torque_traj == None:    #tab the following if uncommenting this line
        torque_des, _ = extract_data_from_polynomial(u_trajectory, frequency)
        
        arm2_Ang_pos = ricmonk_state[2, :] + ricmonk_state[1, :]
        arm2_Ang_vel = ricmonk_state[5, :] + ricmonk_state[4, :]
        arm2_Ang_acc = x0_d_vec[5, :] + x0_d_vec[4, :]
        arm2_Ang_jerk = x0_dd_vec[5, :] + x0_dd_vec[4, :]
        phi_theta3_diff = ricmonk_state[2, :] - arm2_Ang_pos
        
        data = {
            "time": time_traj,
            ###
            "arm1_Ang_pos": ricmonk_state[0, :],   #theta1
            "arm1_Ang_vel": ricmonk_state[3, :],   #theta1dot
            "arm1_Ang_acc": x0_d_vec[3, :],   #theta1ddot
            "arm1_Ang_jerk": x0_dd_vec[3, :],   #theta1dddot
            ###
            "tail_Ang_pos": ricmonk_state[1, :],   #theta2
            "tail_Ang_vel": ricmonk_state[4, :],   #theta2dot
            "tail_Ang_acc": x0_d_vec[4, :],   #theta2ddot
            "tail_Ang_jerk": x0_dd_vec[4, :],   #theta2dddot
            "tail_torque": torque_des[0,:],   #theta2_torque_des
            ###
            "arm2_Ang_pos": arm2_Ang_pos,   #theta3
            "arm2_Ang_vel": arm2_Ang_vel,   #theta3dot
            "arm2_Ang_acc": arm2_Ang_acc,   #theta3ddot
            "arm2_Ang_jerk": arm2_Ang_jerk,   #theta3dddot
            ###
            "phi_pos": ricmonk_state[2, :],
            "phi_vel": ricmonk_state[5, :],
            "phi_acc": x0_d_vec[5, :],
            "phi_jerk": x0_dd_vec[5, :],
            "phi_torque": torque_des[1,:],
            ###
            "phi_theta3_diff": phi_theta3_diff
        }


        parent_folder = "data/trajectories"
        folder_name = f"direct_collocation/{maneuver}"
        trajectory_path = make_parent_directory(parent_folder, folder_name, up_directory=5)
        file_name = f"/{maneuver}_traj.csv"
        save_path = trajectory_path + file_name
        save_data(data, save_path)
#         file_name_ref = f"/{maneuver}_ref_traj.csv"
#         save_path_ref = trajectory_path + file_name_ref
#         save_data(data, save_path_ref)
        params_path = trajectory_path + f"/hyperparameters_{maneuver}.txt"
        save_dict(hyper_params, params_path)
        
#         print(f'initial tail position: {ricmonk_state[1, 0]}')
#         print(f'initial tail velocity: {ricmonk_state[4, 0]}')
#         print(f'final tail position: {ricmonk_state[1, -1]}')
#         print(f'final tail velocity: {ricmonk_state[4, -1]}')
        
    else:
        x0_d = x_trajectory.derivative(derivative_order=1)
        x0_dd = x_trajectory.derivative(derivative_order=2)
        
        x0_d_w_arm2 = x0_w_arm2.derivative(derivative_order=1)
        x0_dd_w_arm2 = x0_w_arm2.derivative(derivative_order=2)
        # Extract State
        ricmonk_state, time_traj = extract_data_from_polynomial(
            x_trajectory, frequency
        )
        ricmonk_state_w_arm2, time_traj = extract_data_from_polynomial(
            x0_w_arm2, frequency
        )

        # Extract trajectory
        x0_d_vec, _ = extract_data_from_polynomial(x0_d, frequency)
        x0_dd_vec, _ = extract_data_from_polynomial(x0_dd, frequency)
        x0_d_vec_w_arm2, _ = extract_data_from_polynomial(x0_d_w_arm2, frequency)
        x0_dd_vec_w_arm2, _ = extract_data_from_polynomial(x0_dd_w_arm2, frequency)
        
        # Extract desired input
    #    if tail_des_torque_traj == None:    #tab the following if uncommenting this line
        torque_des, _ = extract_data_from_polynomial(u_trajectory, frequency)

#         phi_pos = ricmonk_state[2, :] - ricmonk_state[1, :]
#         phi_vel = ricmonk_state[5, :] - ricmonk_state[4, :]
#         phi_acc = x0_d_vec[5, :] - x0_d_vec[4, :]
#         phi_jerk = x0_dd_vec[5, :] - x0_dd_vec[4, :]
        phi_pos = ricmonk_state[2, :]
        phi_theta3_diff = phi_pos - ricmonk_state_w_arm2[2, :]
        data = {
            "time": time_traj,
            ###
            "arm1_Ang_pos": ricmonk_state[0, :],   #theta1
            "arm1_Ang_vel": ricmonk_state[3, :],   #theta1dot
            "arm1_Ang_acc": x0_d_vec[3, :],   #theta1ddot
            "arm1_Ang_jerk": x0_dd_vec[3, :],   #theta1dddot
            ###
            "tail_Ang_pos": ricmonk_state[1, :],   #theta2
            "tail_Ang_vel": ricmonk_state[4, :],   #theta2dot
            "tail_Ang_acc": x0_d_vec[4, :],   #theta2ddot
            "tail_Ang_jerk": x0_dd_vec[4, :],   #theta2dddot
            "tail_torque": torque_des[0,:],   #theta2_torque_des
            ###
            "arm2_Ang_pos": ricmonk_state_w_arm2[2, :],   #theta3
            "arm2_Ang_vel": ricmonk_state_w_arm2[5, :],   #theta3dot
            "arm2_Ang_acc": x0_d_vec_w_arm2[5, :],   #theta3ddot
            "arm2_Ang_jerk": x0_dd_vec_w_arm2[5, :],   #theta3dddot
            ###
            "phi_pos": ricmonk_state[2, :],
            "phi_vel": ricmonk_state[5, :],
            "phi_acc": x0_d_vec[5, :],
            "phi_jerk": x0_dd_vec[5, :],
            "phi_torque": torque_des[1,:],
            ###
            "phi_theta3_diff": phi_theta3_diff
        }
        # Extract_meas_state
        meas_time = meas_torque_tail.sample_times()
        # des_time = csv_data.time.reshape(csv_data.time.shape[0], -1)
        x_meas_desc = np.vstack(
            (
                meas_state.data()[0, :],
                meas_state.data()[1, :],
                meas_state.data()[2, :],
                meas_state.data()[3, :],
                meas_state.data()[4, :],
                meas_state.data()[5, :],
            )
        )
        u_meas_tail_desc = meas_torque_tail.data()
        u_meas_arm2_desc = meas_torque_arm2.data()
        u_meas_tail_traj = PiecewisePolynomial.FirstOrderHold(meas_time, u_meas_tail_desc)
        u_meas_arm2_traj = PiecewisePolynomial.FirstOrderHold(meas_time, u_meas_arm2_desc)
        
        x_meas_traj = PiecewisePolynomial.CubicShapePreserving(
            meas_time, x_meas_desc, zero_end_point_derivatives=True
        )
        ricmonk_meas_state, time_traj = extract_data_from_polynomial(
            x_meas_traj, frequency
        )
        tail_torque_meas, _ = extract_data_from_polynomial(u_meas_tail_traj, frequency)
        arm2_torque_meas, _ = extract_data_from_polynomial(u_meas_arm2_traj, frequency)
        
#         phi_pos = ricmonk_state[2, :] - ricmonk_state[1, :]
#         phi_vel = ricmonk_state[5, :] - ricmonk_state[4, :]
#         phi_acc = x0_d_vec[5, :] - x0_d_vec[4, :]
#         phi_jerk = x0_dd_vec[5, :] - x0_dd_vec[4, :]
#         phi_theta3_diff = phi_pos - ricmonk_state[2, :]
        
        arm2_Ang_meas_pos = ricmonk_meas_state[2, :] + ricmonk_meas_state[1, :]
        arm2_Ang_meas_vel = ricmonk_meas_state[5, :] + ricmonk_meas_state[4, :]
        
        data["time"] = time_traj
        data["arm1_Ang_meas_pos"] = ricmonk_meas_state[0, :]
        data["arm1_Ang_meas_vel"] = ricmonk_meas_state[3, :]
        data["tail_Ang_meas_pos"] = ricmonk_meas_state[1, :]
        data["tail_Ang_meas_vel"] = ricmonk_meas_state[4, :]
        data["phi_meas_pos"] = ricmonk_meas_state[2, :]
        data["phi_meas_vel"] = ricmonk_meas_state[5, :]
        data["tail_meas_torque"] = tail_torque_meas[0, :]
        data["phi_meas_torque"] = arm2_torque_meas[0, :]
        ###
        data["arm2_Ang_meas_pos"] = arm2_Ang_meas_pos
        data["arm2_Ang_meas_vel"] = arm2_Ang_meas_vel
        ###
        
        if controller_type == "tvlqr":
            K, k0, _, n_points = create_gain_arrays(controller, frequency)
            print(f"n_points shape: {n_points}")
            print(f"Kmat shape: {K.shape}")
            data["K1"] = (K[..., 0, 0],)
            data["K2"] = (K[..., 0, 1],)
            data["K3"] = (K[..., 0, 2],)
            data["K4"] = (K[..., 0, 3],)
            data["K5"] = (K[..., 0, 4],)
            data["K6"] = (K[..., 0, 5],)
            data["K7"] = (K[..., 1, 0],)
            data["K8"] = (K[..., 1, 1],)
            data["K9"] = (K[..., 1, 2],)
            data["K10"] = (K[..., 1, 3],)
            data["K11"] = (K[..., 1, 4],)
            data["K12"] = (K[..., 1, 5],)
            data["k0"] = (k0,)
            x_bar = ricmonk_state - ricmonk_meas_state
            costToGo = np.zeros((x_bar.shape[1]))
            #print(f'shape1: {x_bar.shape[1]}')
            for i in range(x_bar.shape[1]):
                t_input = i / frequency
                S_at_t = controller.S.value(t_input)
                x_bar_ = ricmonk_state[:,i] - ricmonk_meas_state[:,i]
                #cVal = np.matmul(x_bar_.T, np.matmul(S_at_t, x_bar_))
                #print(x_bar_)
                #print(cVal)
                costToGo[i] = np.matmul(x_bar_.T, np.matmul(S_at_t, x_bar_))
            data["costToGo"] = costToGo
            
        desList_rmse = [ricmonk_state[0, :], ricmonk_state[3, :], ricmonk_state[1, :], ricmonk_state[4, :], ricmonk_state[2, :], ricmonk_state[5, :]]
        measList_rmse = [ricmonk_meas_state[0, :], ricmonk_meas_state[3, :], ricmonk_meas_state[1, :], ricmonk_meas_state[4, :], ricmonk_meas_state[2, :], ricmonk_meas_state[5, :]]
        
        num_pairs = len(desList_rmse)
        
        rmse_list = []
        
#         for i in range(num_pairs):
#             rmse = np.sqrt(mean_squared_error(desList_rmse[i], measList_rmse[i]))
#             rmse_list.append(rmse)
            
#         list_names = ["arm1_pos", "arm1_vel", "tail_pos", "tail_vel", "phi_pos", "phi_vel"]
            
#         for i in range(num_pairs):
#             print(f'RMSE value for {controller_type} tracking of {list_names[i]} during {maneuver} trajectory is: {rmse_list[i]}')
                
        parent_folder = "data/trajectories"
        folder_name = "closed_loop"        
        trajectory_path = make_parent_directory(parent_folder, folder_name, up_directory=4)
        file_name = f"/{maneuver}_{controller_type}.csv"
        save_path = trajectory_path + file_name
        save_data(data, save_path)
        params_path = trajectory_path + f"/hyperparameters_{maneuver}_{controller_type}.txt"
        save_dict(hyper_params, params_path)            
    return data


def forward_kinematics(theta1, theta2, component):
    """
    Function to compute the forward kinematics of the AcroMonk Robot,
    i.e. compute the end-effector position given the joint angles.
    Returns ons the (y,z) coordinates as the AcroMonk can only move in a plane.
    """
    assert component in (["arm", "tail"])
    if component == "arm":
        l1 = 0.314645   #arm length
        l2 = 0.314645   #arm length
    elif component == "tail":
        l1 = 0.314645   #arm length
        l2 = 0.439   #tail length
    else:
        return 0
        
#     l1 = link_length
#     l2 = l1
    ee_y = (l1 * np.sin(theta1)) + (l2 * np.sin(theta1 + theta2))
    ee_z = -(l1 * np.cos(theta1) + (l2 * np.cos(theta1 + theta2)))

    return ee_y, ee_z

def forward_geometry_kinematic_constraint(vars, component):
    assert component in (["arm", "tail"])
    if component == "arm":
        l1 = 0.314645   #arm length
        l2 = 0.314645   #arm length
        theta1 = vars[0]
        theta2 = vars[1]
        phi3 = vars[2]
        theta3 = theta2 + phi3
        theta1_dot = vars[3]
        theta2_dot = vars[4]
        phi3_dot = vars[5]
        theta3_dot = theta2_dot + phi3_dot
    elif component == "tail":
        l1 = 0.314645   #arm length
        l2 = 0.439   #tail length
    else:
        return 0 
    
    ee_y = (l1 * np.sin(theta1)) + (l2 * np.sin(theta1 + theta3))
    ee_z = -(l1 * np.cos(theta1) + (l2 * np.cos(theta1 + theta3)))
    
#     ee = np.array([ee_y, ee_z])
#     req_ee = np.array([0.38, 0.05])
    
    ee_y_dot = (l1 * theta1_dot * np.cos(theta1)) + (
        l2 * (theta1_dot + theta3_dot) * np.cos(theta1 + theta3)
    )
    ee_z_dot = (l1 * theta1_dot * np.sin(theta1)) + (
        l2 * (theta1_dot + theta3_dot) * np.sin(theta1 + theta3)
    )  # changed the sign in ee_z_dot
    
#     ee_dot = np.array([ee_y_dot, ee_z_dot])
#     req_ee_dot = np.array([0.9, -0.2])

    ee_state = np.array([ee_y, ee_z, ee_y_dot, ee_z_dot])
    req_ee_state = np.array([0.38, 0.05, 1, -0.2])

    return req_ee_state - ee_state

def forward_kinematics_array(vars, component, fin_y_pos, fin_z_pos):
    """
    Function to compute the forward kinematics of the AcroMonk Robot,
    i.e. compute the end-effector position given the joint angles.
    Returns ons the (y,z) coordinates as the AcroMonk can only move in a plane.
    """
    theta1 = vars[0]
    theta2 = vars[1]
    phi3 = vars[2]
    theta3 = theta2 + phi3
    
    assert component in (["arm", "tail"])
    if component == "arm":
        l1 = 0.314645   #arm length
        l2 = 0.313645   #arm length
    elif component == "tail":
        l1 = 0.313645   #arm length
        l2 = 0.439   #tail length
    else:
        return 0
        
#     l1 = link_length
#     l2 = l1
    ee_y = (l1 * np.sin(theta1)) + (l2 * np.sin(theta1 + theta3))
    ee_z = -(l1 * np.cos(theta1) + (l2 * np.cos(theta1 + theta3)))
    
    ee = np.array([ee_y, ee_z])
    req_ee = np.array([fin_y_pos, fin_z_pos])

    return req_ee - ee

def forward_diff_kinematics_array(vars, component, fin_y_vel, fin_z_vel):
    """
    Function to compute the differential forward kinematics of the AcroMonk Robot,
    i.e. compute the end-effector velocity given the join position/velocities
    Returns ons the (y_dot,z_dot) coordinates as the AcroMonk can only move in a plane.
    """
    assert component in (["arm", "tail"])
    theta1 = vars[0]
    theta2 = vars[1]
    phi3 = vars[2]
    theta3 = theta2 + phi3
    theta1_dot = vars[3]
    theta2_dot = vars[4]
    phi3_dot = vars[5]
    theta3_dot = theta2_dot + phi3_dot
    
    if component == "arm":
        l1 = 0.314645   #arm length
        l2 = 0.314645   #arm length
    elif component == "tail":
        l1 = 0.314645   #arm length
        l2 = 0.439   #tail length
        
#     l1 = link_length
#     l2 = l1
    ee_y_dot = (l1 * theta1_dot * np.cos(theta1)) + (
        l2 * (theta1_dot + theta3_dot) * np.cos(theta1 + theta3)
    )
    ee_z_dot = (l1 * theta1_dot * np.sin(theta1)) + (
        l2 * (theta1_dot + theta3_dot) * np.sin(theta1 + theta3)
    )  # changed the sign in ee_z_dot
    
    ee_dot = np.array([ee_y_dot, ee_z_dot])
    req_ee_dot = np.array([fin_y_vel, fin_z_vel])
    
    
    return req_ee_dot - ee_dot


def forward_diff_kinematics(
    theta1, theta2, theta1_dot, theta2_dot, component):
    """
    Function to compute the differential forward kinematics of the AcroMonk Robot,
    i.e. compute the end-effector velocity given the join position/velocities
    Returns ons the (y_dot,z_dot) coordinates as the AcroMonk can only move in a plane.
    """
    assert component in (["arm", "tail"])
    if component == "arm":
        l1 = 0.314645   #arm length
        l2 = 0.314645   #arm length
    elif component == "tail":
        l1 = 0.314645   #arm length
        l2 = 0.439   #tail length
        
#     l1 = link_length
#     l2 = l1
    ee_y_dot = (l1 * theta1_dot * np.cos(theta1)) + (
        l2 * (theta1_dot + theta2_dot) * np.cos(theta1 + theta2)
    )
    ee_z_dot = (l1 * theta1_dot * np.sin(theta1)) + (
        l2 * (theta1_dot + theta2_dot) * np.sin(theta1 + theta2)
    )  # changed the sign in ee_z_dot
    return ee_y_dot, ee_z_dot

def inverse_geometry(y_pos, z_pos, component):
    """
    Function that uses the inverse geometric model of the AcroMonk Robot
    to compute the joint angles given the end-effector position.
    Returns the (theta1, theta2) joint coordinates of the AcroMonk.
    """
    assert component in (["arm", "tail"])
    if component == "arm":
        l1 = 0.314645   #arm length
        l2 = 0.314645   #arm length
    elif component == "tail":
        l1 = 0.314645   #arm length
        l2 = 0.439   #tail length
        
#     l1 = link_length
#     l2 = l1
    if y_pos < 0:
        y_pos = y_pos - 0.01
        z_pos = z_pos - 0.01
        theta2 = -np.arccos(((y_pos*y_pos) + (z_pos*z_pos) - (l1*l1) - (l2*l2))/(2*l1*l2))
    else:
        y_pos = y_pos - 0.02
        z_pos = z_pos + 0.02
        theta2 = np.arccos(((y_pos*y_pos) + (z_pos*z_pos) - (l1*l1) - (l2*l2))/(2*l1*l2))
                

    M = l1 + l2*np.cos(theta2)
    N = l2*np.sin(theta2)
    det = ((y_pos*y_pos*M*M) - (M*M + N*N)*(y_pos*y_pos - N*N))
#     print(f'det: {det}.')
    
    theta1 = np.arcsin(((y_pos*M) + sqrt(det))/(M*M + N*N))
    #print(f'sin: {((y_pos*M) + sqrt(det))/(M*M + N*N)}.')
    #print("Final desired joint angles, calculated without z in consideration")
    #print(theta1, theta2)
    
    #gamma = np.arctan2(y_pos,z_pos)
    #beta = np.arctan2((l2*np.sin(theta2)), (l1 + l2*np.cos(theta2)))
    
    #theta1 = gamma - beta
    #print("Final desired joint angles, calculated with z in consideration")
    #print(theta1, theta2)
    
    return theta1, theta2

def inverse_kinematics(
    theta1, theta2, ee_y_dot, ee_z_dot, component
):
    """
    Function that uses the inverse kinematic model of the AcroMonk Robot
    to compute the joint angle velocities given the end-effector position and end-effector velocities.
    Returns the (theta1_dot, theta2_dot) joint coordinates of the AcroMonk.
    """
    assert component in (["arm", "tail"])
    if component == "arm":
        l1 = 0.314645   #arm length
        l2 = 0.314645   #arm length
    elif component == "tail":
        l1 = 0.314645   #arm length
        l2 = 0.439   #tail length
        
#     l1 = link_length
#     l2 = l1
    
    jacobian = np.matrix([[((l1*np.cos(theta1)) + (l2*np.cos(theta1+theta2))), (l2*np.cos(theta1+theta2))],
                          [((l1*np.sin(theta1)) + (l2*np.sin(theta1+theta2))), (l2*np.sin(theta1+theta2))]])
    
    inverse_jacobian = np.linalg.inv(jacobian)
    
    ee_vel = np.matrix([[ee_y_dot],
                           [ee_z_dot]])
    
    joint_vel = np.matmul(inverse_jacobian, ee_vel)
    
    theta1_dot = joint_vel[0]
    theta2_dot = joint_vel[1]
    
    return theta1_dot, theta2_dot


def create_acromonk_plant(ladder_distance):
    from pydrake.all import Parser, DiagramBuilder, AddMultibodyPlantSceneGraph

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    urdf_folder = "data/simulation_models"
    
    #fileObject = open("acromonk.urdf", "r")
    #urdf=fileObject.read()
    #fileObject.close()
    #urdf = urdf.replace("VARIABLE", "ladder_distance")
    #with open('finalAcromonk.urdf', 'w') as file:
     #   file.write(urdf)
#     file_name = "ricMonk.urdf"
    up_directory = 4
#     urdf_path = generate_path(urdf_folder, file_name, up_directory)
#     with open(urdf_path) as file:
#         urdf = file.read()
#     urdf = urdf.replace("VARIABLE", str(ladder_distance))
    
    file_name = "finalRicMonkCurrent.urdf"
    urdf_path = generate_path(urdf_folder, file_name, up_directory)
#     with open(urdf_path, 'w') as file:
#         file.write(urdf) 
        
    parser.AddModels(urdf_path)
    plant.Finalize()
    context = plant.CreateDefaultContext()
    return plant, context, scene_graph, builder


def drake_visualizer(scene_graph, builder, initial_state, duration, visualize=None):
    from pydrake.all import Simulator, StartMeshcat, MeshcatVisualizer
    print('\n<<<<<Visualization started>>>>>\n')
    meshcat = StartMeshcat()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1)
    context_simulator = simulator.get_mutable_context()
    print(f"Continuous state number: {context_simulator.num_continuous_states()}")
    print(f"Total state number: {context_simulator.num_total_states()}")
    print(f"Discrete group state number: {context_simulator.num_discrete_state_groups()}")
    if visualize == 'pid':
        initial_state = np.append(initial_state,[[0]],axis=0)
        initial_state = np.append(initial_state,[[0]],axis=0) #two times because there are two inputs to the system
    if visualize != None:
        print(f"Initial state: {initial_state.shape}")
        context_simulator.SetContinuousState(initial_state)
    visualizer.StartRecording()
    simulator.AdvanceTo(duration)
    visualizer.PublishRecording()
    return simulator, diagram


def load_desired_trajectory(maneuver):
    trajectory_folder = f'data/trajectories/direct_collocation/{maneuver}'
    file_name = f'{maneuver}_traj.csv'
    up_directory = 4
    path_to_csv = generate_path(trajectory_folder, file_name, up_directory)
    print(f'[load_desired_trajectory]:loading {path_to_csv}.')
    data_csv = pd.read_csv(path_to_csv)
#     print(f"[load_desired_trajectory: max_u:] {max(abs(data_csv['tail_torque'].values))},{ max(abs(data_csv['phi_torque'].values))}")
    x0, u0, x0_d, x0_dd, x0_w_arm2 = fit_polynomial(data=data_csv)   
    
    file_name = f'hyperparameters_{maneuver}.txt'
    path_to_txt = generate_path(trajectory_folder, file_name, up_directory)
    mylines = []  
    with open(path_to_txt,'rt') as file:
        for myline in file:
            mylines.append(myline)            
    #print(mylines[5])
    str1=''
    for ele in mylines[5]:
        str1 += ele
    #print(str1)
    ladderDistance = float(str1.partition('=')[2])
    print(f'Ladder distance: {ladderDistance}')
    
    #index = string.find('=')
    #if index != -1:
    #    print("The character '=' is present in the string at: ", index)
    #    
    #else:
    #    print('The character is not present in the string')
        #hyperParamFile = file.read()
    #print(hyperParamFile)
    
    
    return x0, u0, x0_d, x0_dd, x0_w_arm2, ladderDistance
                     

def make_parent_directory(parent_folder, folder_name, up_directory):
    directory = generate_path(parent_folder, folder_name, up_directory)
    try:
        os.makedirs(directory)
    except BaseException:
        pass
    return directory
