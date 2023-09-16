import matplotlib.pyplot as plt
import os
from datetime import datetime
from utils import forward_kinematics, generate_path, forward_kinematics


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


def plot_closed_loop_control_data(directory, data, show=True):
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
            data["phi_meas_pos"],
            data["phi_pos"],
            
            
        ],
        labels=["Time (s)", "Position (rad)"],
        title="Position (rad) vs Time (s)",
        legends=["arm1_Ang_meas_pos", "arm1_Ang_pos", "tail_Ang_meas_pos", "tail_Ang_pos",  "phi_meas_pos", "phi_pos"],
        linestyle=["-", "--", "-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred", "magenta", "mediumorchid"],
        save_name=directory + "/pos.pdf",
    )
#     plot_data(
#         x_data=[
#             data["time"],
#             data["time"],
#             data["time"],
#             data["time"],
#         ],
#         y_data=[
#             data["phi_pos"],
#             data["arm2_Ang_pos"],
#             data["phi_meas_pos"],
#             data["arm2_Ang_meas_pos"],
#         ],
#         labels=["Time (s)", "Position (rad)"],
#         title="Position (rad) vs Time (s)",
#         legends=["phi_pos", "arm2_Ang_pos", "phi_meas_pos", "arm2_Ang_meas_pos"],
#         linestyle=["--", "--", "-", "-"],
#         colors=["blue", "red", "cornflowerblue", "indianred"],
#         save_name=directory + "/comPos.pdf",
#     )
#     plot_data(
#         x_data=[
#             data["time"],
#             data["time"],
#             data["time"],
#             data["time"],
#         ],
#         y_data=[
#             data["phi_vel"],
#             data["phi_meas_vel"],
#             data["arm2_Ang_vel"],
#             data["arm2_Ang_meas_vel"],
#         ],
#         labels=["Time (s)", "Velocity (rad/s)"],
#         title="Velocity (rad/s) vs Time (s)",
#         legends=["phi_vel", "phi_meas_vel", "arm2_Ang_vel", "arm2_Ang_meas_vel"],
#         linestyle=["--", "-", "--", "-"],
#         colors=["blue", "cornflowerblue", "red", "indianred"],
#         save_name=directory + "/comVel.pdf",
#s    )
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
            data["phi_meas_vel"],
            data["phi_vel"],            
        ],
        labels=["Time (s)", "Velocity (rad/s)"],
        title="Velocity (rad/s) vs Time (s)",
        legends=["arm1_Ang_meas_vel", "arm1_Ang_vel", "tail_Ang_meas_vel", "tail_Ang_vel", "phi_meas_vel", "phi_vel"],
        linestyle=["-", "--", "-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred", "magenta", "mediumorchid"],
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
    plot_data(
        x_data=[
            data["time"], 
            data["time"],
            ],
        y_data=[
            data["tail_meas_torque"],
            data["tail_torque"],
        ],
        labels=["Time (s)", "Torque (Nm)"],
        title="Torque (Nm) vs Time (s)",
        legends=["torque_meas_tail","torque_des_tail"],
        linestyle=["-", "--"],
        colors=["blue", "cornflowerblue"],
        save_name=directory + "/tailtau.pdf",
    )
    plot_data(
        x_data=[
            data["time"],
            data["time"],
            ],
        y_data=[
            data["phi_meas_torque"],
            data["phi_torque"],
        ],
        labels=["Time (s)", "Torque (Nm)"],
        title="Torque (Nm) vs Time (s)",
        legends=["torque_meas_phi","torque_des_phi"],
        linestyle=[ "-", "--"],
        colors=["red", "indianred"],
        save_name=directory + "/phitau.pdf",
    )   
    ee_y, ee_z = forward_kinematics(
        data["arm1_Ang_pos"], data["tail_Ang_pos"], "tail"
    )
    ee_y_meas, ee_z_meas = forward_kinematics(
        data["arm1_Ang_meas_pos"], data["tail_Ang_meas_pos"], "tail"
    )    
    plot_data(
        x_data=[
            ee_y_meas,
            ee_y
            ],
        y_data=[
            ee_z_meas,
            ee_z
        ],
        labels=["Y-component(m)", "Z-component(m)"],
        title="Tail end Trajectory",
        legends=["measured", "desired"],
        linestyle=["-", "--"],
        colors=["blue", "red"],
        save_name=directory + "/tail_ee_traj.pdf",
    ) 
    
    ee_y, ee_z = forward_kinematics(
        data["arm1_Ang_pos"], data["arm2_Ang_pos"], "arm"
    )
    ee_y_meas, ee_z_meas = forward_kinematics(
        data["arm1_Ang_meas_pos"], data["arm2_Ang_meas_pos"], "arm"
    )    
    plot_data(
        x_data=[
            ee_y_meas,
            ee_y
            ],
        y_data=[
            ee_z_meas,
            ee_z
        ],
        labels=["Y-component(m)", "Z-component(m)"],
        title="Arm2 end Trajectory",
        legends=["measured", "desired"],
        linestyle=["-", "--"],
        colors=["blue", "red"],
        save_name=directory + "/arm2_ee_traj.pdf",
    )
    if 'costToGo' in data:
        plot_data(
            x_data=[
                data["time"],
                ],
            y_data=[
                data["costToGo"],
            ],
            labels=["Time (s)", "Cost-to-go TVLQR"],
            title="Cost-to-go TVLQR (Nm) vs Time (s)",
            legends=["Cost-to-go TVLQR"],
            linestyle=[ "-"],
            colors=["orange"],
            save_name=directory + "/costToGo.pdf",
        ) 
    if show:
        plt.show()
    


def plot_traj(data, directory, show=True):
    plot_data(
    x_data=[
        data["time"],
        data["time"],
        data["time"],
    ],
    y_data=[
        data["arm1_Ang_pos"],
        data["tail_Ang_pos"],
        data["phi_pos"],        
    ],
    labels=["Time (s)", "Position (rad)"],
    title="Position (rad) vs Time (s)",
    legends=["arm1_des_Ang_pos", "tail_des_Ang_pos", "phi_pos"],
    linestyle=["-", "-", "-"],
    colors=["blue", "red", "orange"],
    save_name=directory + "/pos.pdf",
    )
    plot_data(
        x_data=[
            data["time"],
            data["time"],
            data["time"],
        ],
        y_data=[
            data["arm1_Ang_vel"],
            data["tail_Ang_vel"],
            data["phi_vel"],
        ],
        labels=["Time (s)", "Velocity (rad/s)"],
        title="Velocity (rad/s) vs Time (s)",
        legends=["arm1_des_Ang_vel", "tail_des_Ang_vel", "phi_vel"],
        linestyle=["-", "-", "-"],
        colors=["blue", "red", "orange"],
        save_name=directory + "/vel.pdf",
    )
    plot_data(
        x_data=[
            data["time"],
            data["time"],
            data["time"],
        ],
        y_data=[
            data["arm1_Ang_acc"],
            data["tail_Ang_acc"],
            data["phi_acc"],
        ],
        labels=["Time (s)", "Acceleration (rad/s)"],
        title="Acceleration (rad/s/s) vs Time (s)",
        legends=["arm1_Ang_acc", "tail_des_acc", "phi_acc"],
        linestyle=["-", "-", "-"],
        colors=["blue", "red", "green"],
        save_name=directory + "/acc.pdf",
    )
    plot_data(
        x_data=[
            data["time"],
            data["time"],
            data["time"],
        ],
        y_data=[
            data["arm1_Ang_jerk"],
            data["tail_Ang_jerk"],
            data["phi_jerk"],
        ],
        labels=["Time (s)", "You Jerk! (rad/s)"],
        title="Jerk (rad/s/s/s) vs Time (s)",
        legends=["arm1_Ang_jerk", "tail_des_jerk", "phi_jerk"],
        linestyle=["-", "-", "-"],
        colors=["blue", "red", "green"],
        save_name=directory + "/jer.pdf",
    )
    plot_data(
        x_data=[
            data["time"],
            data["time"],
            data["time"],
        ],
        y_data=[
            data["phi_theta3_diff"],
            data["arm2_Ang_pos"],
            data["phi_pos"],
        ],
        labels=["Time (s)", "phi_theta3_diff! (rad/s)"],
        title="phi_theta3_diff (rad) vs Time (s)",
        legends=["phi_theta3_diff","arm2_Ang_pos","phi_pos"],
        linestyle=["-","-","-"],
        colors=["blue","orange","green"],
        save_name=directory + "/phi_theta3_diff_arm2Pos_phiPos.pdf",
    )

    plot_data(
        x_data=[
            data["time"],
            data["time"],
            ],
        y_data=[
            data["tail_torque"],
            data["phi_torque"],
        ],
        labels=["Time (s)", "Torque (Nm)"],
        title="Torque (Nm) vs Time (s)",
        legends=["tail_torque_des", "phi_torque"],
        linestyle=["-", "-"],
        colors=["blue", "red"],
        save_name=directory + "/tau.pdf",
    ) 
    #ee_y_arm2, ee_z_arm2 = forward_kinematics(
    #    data["arm1_Ang_pos"], data["tail_Ang_pos"] + data["arm2_Ang_pos"], "arm"
    #)
    ee_y_arm2, ee_z_arm2 = forward_kinematics(
        data["arm1_Ang_pos"], data["arm2_Ang_pos"], "arm"
    )
    plot_data(
        x_data=[
            ee_y_arm2
            ],
        y_data=[
            ee_z_arm2
        ],
        labels=["Y-component(m)", "Z-component(m)"],
        title="Arm 2 end-Effector Trajectory",
        legends=["Arm 2 ee trajectory"],
        linestyle=["-"],
        colors=["blue"],
        save_name=directory + "/ee_2_traj.pdf",
    )
    ee_y_tail, ee_z_tail = forward_kinematics(
        data["arm1_Ang_pos"], data["tail_Ang_pos"], "tail"
    )
    plot_data(
        x_data=[
            ee_y_tail
            ],
        y_data=[
            ee_z_tail
        ],
        labels=["Y-component(m)", "Z-component(m)"],
        title="Tail end-Effector Trajectory",
        legends=["Tail ee trajectory"],
        linestyle=["-"],
        colors=["red"],
        save_name=directory + "/ee_t_traj.pdf",
    )
    
    if show:
        plt.show()