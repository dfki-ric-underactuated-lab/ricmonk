import sys
sys.path.append("../../utilities/")
from utils import (
    drake_visualizer,
    create_acromonk_plant,
    np,
    load_desired_trajectory,
    save_trajectory,
    make_parent_directory
)
from utils_plot import plot_closed_loop_control_data
from stabilizer_utils import load_controller
from datetime import datetime

ladder_distance = 0.34

#plant, context, scene_graph, builder = create_acromonk_plant(ladder_distance)
maneuver = input(
    "Enter the name of atomic behavior (ZB, ZF, BF, FB, OTHER): "
).upper()
# maneuver = "ZB"
assert maneuver in (["ZB", "ZF", "BF", "FB", "OTHER"])
controller_type = input("Enter controller type (PID, TVLQR): ").lower()
# controller_type = "tvlqr"
assert controller_type in (["tvlqr", "pid"])
x0, u0, _, _, x0_w_arm2, ladder_distance = load_desired_trajectory(maneuver)
plant, context, scene_graph, builder = create_acromonk_plant(ladder_distance)

# Desired trajectories
des_trajectories = (x0, u0)
# PID gains
Kp_tail = 120 * np.ones(1)
Ki_tail = np.ones(1)
Kd_tail = 2 * np.ones(1)
Kp_arm2 = 120 * np.ones(1)
Ki_arm2 = np.ones(1)
Kd_arm2 = 2 * np.ones(1)
pid_gains = (Kp_tail, Ki_tail, Kd_tail, Kp_arm2, Ki_arm2, Kd_arm2)

# ZF TVLQR control params
if maneuver == "ZF":
#     Q = np.diag([10, 20, 5, 5, 20, 5])
#     R = np.diag([35, 10])
#     Qf = np.diag([1, 4, 5, 1, 5, 0.5])
    Q = np.diag([1, 1, 15, 1, 2, 1])
    R = np.eye(2) * 5
    Qf = np.diag([1, 1, 1, 1, 1, 1])
    
# ZB TVLQR control params
elif maneuver == "BF":
#     Q = np.diag([1, 1, 10, 1, 2, 1])
#     R = np.eye(2) * 25
#     Qf = np.diag([1, 1, 5, 1, 1, 1])
    Q = np.diag([1, 1, 17, 1, 2, 0.2])
    R = np.eye(2) * 20
    Qf = np.diag([1, 1, 1, 1, 1, 1])
    
elif maneuver == "ZB":
#     Q = np.diag([1, 1, 10, 1, 2, 1])
#     R = np.eye(2) * 25
#     Qf = np.diag([1, 1, 5, 1, 1, 1])
    Q = np.diag([1, 1, 15, 1, 2, 1])
    R = np.eye(2) * 8
    Qf = np.diag([1, 1, 1, 1, 1, 1])
    
elif maneuver == "FB":
#     Q = np.diag([1, 1, 10, 1, 2, 1])
#     R = np.eye(2) * 25
#     Qf = np.diag([1, 1, 5, 1, 1, 1])
    Q = np.diag([1, 1, 15, 1, 2, 1])
    R = np.eye(2) * 20
    Qf = np.diag([1, 1, 1, 1, 1, 1])    

if controller_type == "pid":   #dummy values
    Q = 0
    R = 0
    Qf = 0
    
tvlqr_gains = (Q, R, Qf)
# Controller gains
controller_gains = (pid_gains, tvlqr_gains)
# Load the controller
(
    controller,
#    controller2,
    hyper_params,
    builder,
    state_logger,
    input_tail_logger,
    input_arm2_logger
) = load_controller(
    plant=plant,
    context=context,
    builder=builder,
    des_trajectories=des_trajectories,
    controller_type=controller_type,
    controller_gains=controller_gains,
    tau_limit=6
)
# Visualize the closed-loop control
duration = x0.end_time()
initial_state = x0.value(x0.start_time())
simulator, diagram = drake_visualizer(
    scene_graph,
    builder,
    initial_state=initial_state,
    duration=x0.end_time(),
    visualize=controller_type,
)
# Save the simulation results and hyper parameters
input_tail_log = input_tail_logger.FindLog(simulator.get_context())
input_arm2_log = input_arm2_logger.FindLog(simulator.get_context())
state_log = state_logger.FindLog(simulator.get_context())

print(state_log.data())
traj_data = save_trajectory(
    maneuver=maneuver,
    x_trajectory=x0,
    u_trajectory=u0,
    frequency=1000,
    hyper_params=hyper_params,
    controller_type=controller_type,    
    controller=controller,
    meas_state=state_log,
    meas_torque_tail=input_tail_log,
    meas_torque_arm2=input_arm2_log,
    x0_w_arm2=x0_w_arm2
)
parent_folder = f"results/simulation/{maneuver}_{controller_type}"
folder_name = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
directory = make_parent_directory(parent_folder, folder_name, up_directory=4)
plot_closed_loop_control_data(directory, traj_data, show=True)
