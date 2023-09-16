import numpy as np
from pydrake.all import PiecewisePolynomial
from collections import namedtuple


def extract_data_from_polynomial(polynomial, frequency):
    n_points = int(polynomial.end_time() / (1 / frequency))
    time_traj = np.linspace(
        polynomial.start_time(), polynomial.end_time(), n_points
    )
    # extracted_time = time_traj.reshape(n_points, 1).T
    extracted_data = np.hstack(
        [
            polynomial.value(t)
            for t in np.linspace(
                polynomial.start_time(), polynomial.end_time(), n_points
            )
        ]
    )
    return extracted_data, time_traj


def create_gain_arrays(tvlqr_obj, frequency):
    n_points = int(tvlqr_obj.K.end_time() / (1 / frequency))
    time_stamp = np.linspace(
        tvlqr_obj.K.start_time(), tvlqr_obj.K.end_time(), n_points
    )
    
    K = [tvlqr_obj.K.value(i) for i in time_stamp]
    index = 0
    K_mat = np.zeros((n_points, 2, 6))
    for k_vec in K:
        #print(f"kVec shape: {k_vec.shape}")
        for j in range(2):
            K_mat[index][j] = k_vec[j]
        index += 1
    k0 = np.array([tvlqr_obj.k0.value(i)[0][0] for i in time_stamp])
    return K_mat, k0, time_stamp, n_points


def prepare_des_states(csv_data):
    time = csv_data["time"].values
    ## arm1
    arm1_Ang_pos = csv_data["arm1_Ang_pos"].values
    arm1_Ang_vel = csv_data["arm1_Ang_vel"].values
    ## tail
    tail_Ang_pos = csv_data["tail_Ang_pos"].values
    tail_Ang_vel = csv_data["tail_Ang_vel"].values
    tail_torque = csv_data["tail_torque"].values
    ## arm2
    arm2_Ang_pos = csv_data["arm2_Ang_pos"].values
    arm2_Ang_vel = csv_data["arm2_Ang_vel"].values
    ## phi
    phi_pos = csv_data["phi_pos"].values
    phi_vel = csv_data["phi_vel"].values
    phi_torque = csv_data["phi_torque"].values
    Data = namedtuple(
        "Data",
        [
            "time",
            "arm1_Ang_pos",
            "arm1_Ang_vel",
            "tail_Ang_pos",
            "tail_Ang_vel",
            "tail_torque",
            "arm2_Ang_pos",
            "arm2_Ang_vel",
            "phi_pos",
            "phi_vel",
            "phi_torque",
        ],
    )
    data = Data(
        arm1_Ang_pos = arm1_Ang_pos,
        arm1_Ang_vel = arm1_Ang_vel,
        tail_Ang_pos = tail_Ang_pos,
        tail_Ang_vel = tail_Ang_vel,
        tail_torque = tail_torque,
        arm2_Ang_pos = arm2_Ang_pos,
        arm2_Ang_vel = arm2_Ang_vel,
        time=time,
        phi_pos = phi_pos,
        phi_vel = phi_vel,
        phi_torque = phi_torque,
    )
    return data


def fit_polynomial(data):
    """
    This function takes a data as input and fit a polynomial of degree 1 to the torque and
    a cubic one to state, and derivative of order 1 and 2 of states and returns the polynomials
    """
    csv_data = prepare_des_states(data)
    tail_torque_des = csv_data.tail_torque.reshape(
        csv_data.tail_torque.shape[0], -1
    ).T
    phi_torque_des = csv_data.phi_torque.reshape(
        csv_data.phi_torque.shape[0], -1
    ).T
    des_time = csv_data.time.reshape(csv_data.time.shape[0], -1)
    x0_desc = np.vstack(
        (
            csv_data.arm1_Ang_pos,
            csv_data.tail_Ang_pos,
            csv_data.phi_pos,
            csv_data.arm1_Ang_vel,
            csv_data.tail_Ang_vel,
            csv_data.phi_vel,
        )
    )
    x0_desc_w_arm2 = np.vstack(
        (
            csv_data.arm1_Ang_pos,
            csv_data.tail_Ang_pos,
            csv_data.arm2_Ang_pos,
            csv_data.arm1_Ang_vel,
            csv_data.tail_Ang_vel,
            csv_data.arm2_Ang_vel,
        )
    )
#     u0_desc = tail_torque_des
#     u1_desc = phi_torque_des
#     u1 = PiecewisePolynomial.FirstOrderHold(des_time, u0_desc)
#     u2 = PiecewisePolynomial.FirstOrderHold(des_time, u1_desc)
    u0_desc = np.vstack((tail_torque_des, phi_torque_des))
    u0 = PiecewisePolynomial.FirstOrderHold(des_time, u0_desc)
#     u0 = np.vstack((u1, u2))
    x0 = PiecewisePolynomial.CubicShapePreserving(
        des_time, x0_desc, zero_end_point_derivatives=True
    )
    x0_w_arm2 = PiecewisePolynomial.CubicShapePreserving(
        des_time, x0_desc_w_arm2, zero_end_point_derivatives=True
    )
    x0_d = x0.derivative(derivative_order=1)
    x0_dd = x0.derivative(derivative_order=2)
    #return x0, u0, u1, u2, x0_d, x0_dd
    return x0, u0, x0_d, x0_dd, x0_w_arm2
       