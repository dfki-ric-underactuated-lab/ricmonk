import sys
sys.path.append("direct_collocation/")

from trajopt_utils import (
    create_acromonk_plant,
    trajopt,
    DircolHypers,
    load_default_hyperparameters,
    visualize_traj_opt,
)

sys.path.append("../../../utilities/")
from utils import save_trajectory, make_parent_directory
from utils_plot import plot_traj


maneuver = input(
    "Enter the name of atomic behavior (ZB, ZF, BF, FB, OTHER): "
).upper()
#maneuver = "ZF"
assert maneuver in (["ZB", "ZF", "BF", "FB", "OTHER"])
if maneuver in (["ZB", "ZF", "BF", "FB"]):
    bar_seperation = float(input("Enter the ladder distance (default is 0.34): "))
#     bar_seperation = 0.34
    assert bar_seperation < 1.5
    hyper_params = load_default_hyperparameters(maneuver, bar_seperation)
else:
    print("\n<<<<<Insert your desired hyper parameters>>>>>\n")
    '''
    n               : Number of knot points
    tau_limit       : Input torque limit
    initial_state   : Initial state: [theta1, theta2,theta3, theta1_dot, theta2_dot, theta3_dot]
    theta_limit     : Position limits for [theta1, theta2, theta3]
    speed_limit     : Velocity limits for [theta1_dot, theta2_dot, theta3_dot]
    ladder_distance : Distance between two ladder bars in meter
    final_state     : Final state: [theta1, theta2,theta3, theta1_dot, theta2_dot, theta3_dot]
    theta2VelMultip : Theta 2 dot multiplier  (tail)
    theta3VelMultip : Theta 3 dot multiplier  (arm2)
    K               : Velocity regulator
    torque1_multip  : Torque 1 multiplier  (tail)
    torque2_multip  : Torque 2 multiplier  (arm2)
    R               : Input regulization term
    W               : Time panalization of the total trajectory
    init_guess      : Initial trajectory as initial guess for solver 
    '''
    hyper = DircolHypers(
        n=20,
        tau_limit=3.,
        initial_state=(0, 0, 0, 0, 0, 0),
        theta_limit=[2.0943951023931953, 2.8797932657906435, 2.8797932657906435],
        speed_limit=10,
        ladder_distance=0.34,
        final_state=(0., 0., 0., 0., 0., 0.),
        torque1_multip=1,
        torque2_multip=1,
        K=1,
        theta2VelMultip=1,
        theta3VelMultip=1,
        R=1,
        W=0,
        init_guess=[0.0, 2.5],
    )
    hyper_params = hyper.create_params_structure()

print(f'Ladder distance: {hyper_params.ladder_distance}')

plant, context, scene_graph = create_acromonk_plant(hyper_params.ladder_distance)

#endWait = input("Press enter to end wait")

(result, dircol, hyper_params) = trajopt(
    plant=plant,
    context=context,
    hyper_parameters=hyper_params,
    traj=maneuver
)
u_trajectory = dircol.ReconstructInputTrajectory(result)
x_trajectory = dircol.ReconstructStateTrajectory(result)
visualize_traj_opt(plant, scene_graph, x_trajectory)
traj_data = save_trajectory(
    maneuver,
    x_trajectory,
    u_trajectory,
    frequency=1000,
    hyper_params=hyper_params,
)
parent_folder = "data/trajectories"
folder_name = f"direct_collocation/{maneuver}"
directory = make_parent_directory(parent_folder, folder_name, up_directory=5)
plot_traj(traj_data, directory, show=True)
