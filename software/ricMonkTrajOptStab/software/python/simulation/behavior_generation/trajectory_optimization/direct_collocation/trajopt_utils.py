import sys
sys.path.append("../../../utilities/")
import pandas as pd
from utils import (
    drake_visualizer,
    generate_path,
    forward_kinematics,
    forward_diff_kinematics,
    inverse_geometry,
    inverse_kinematics,
    np,
    forward_kinematics_array,
    forward_diff_kinematics_array
)
from utils_polynomials import (
    fit_polynomial, 
)
from pydrake.all import DirectCollocation, PiecewisePolynomial, Solve
from collections import namedtuple
from functools import partial

import os

def trajopt(
    plant,
    context,
    hyper_parameters,
    traj
):
    n=hyper_parameters.n
    tau_limit=hyper_parameters.tau_limit
    initial_state=hyper_parameters.initial_state
    theta_limit=hyper_parameters.theta_limit
    speed_limit=hyper_parameters.speed_limit
    ladder_distance=hyper_parameters.ladder_distance
    final_state=hyper_parameters.final_state
    theta2VelMultip=hyper_parameters.theta2VelMultip
    theta3VelMultip=hyper_parameters.theta3VelMultip
    K=hyper_parameters.K
    torque1_multip=hyper_parameters.torque1_multip
    torque2_multip=hyper_parameters.torque2_multip
    R=hyper_parameters.R
    time_panalization=hyper_parameters.W
    init_guess=hyper_parameters.init_guess
    min_timestep = 0.005
    max_timestep = 0.5
    dircol = DirectCollocation(
        plant,
        context,
        num_time_samples=n,
        minimum_timestep=min_timestep,
        maximum_timestep=max_timestep,
        input_port_index=plant.get_actuation_input_port().get_index(),
    )
    useRefTraj=hyper_parameters.useRefTraj
    dircol.AddEqualTimeIntervalsConstraints()

    ## Constraints

    # Initial Torque
    torque_limit = tau_limit  # N*m.
    u_init = dircol.input(0)
    dircol.AddConstraintToAllKnotPoints(u_init[0] == 0)
    dircol.AddConstraintToAllKnotPoints(u_init[1] == 0) #Assuming u_init has two components because there are two actuators
    
    # Torque limit
    u = dircol.input()
    dircol.AddConstraintToAllKnotPoints(-torque_limit <= u[0])
    dircol.AddConstraintToAllKnotPoints(u[0] <= torque_limit)
    dircol.AddConstraintToAllKnotPoints(-torque_limit <= u[1])
    dircol.AddConstraintToAllKnotPoints(u[1] <= torque_limit)   
    
    prog = dircol.prog()
    
    # Speed limit
    speed_limit = speed_limit    
    state = dircol.state()
    #print(f'state 3: {state[3]}')
    dircol.AddConstraintToAllKnotPoints(-speed_limit <= state[3])  #limit on theta1dot (??)
    dircol.AddConstraintToAllKnotPoints(state[3] <= speed_limit)
    dircol.AddConstraintToAllKnotPoints(-speed_limit <= state[4])  #limit on theta2dot
    dircol.AddConstraintToAllKnotPoints(state[4] <= speed_limit)
    dircol.AddConstraintToAllKnotPoints(-speed_limit <= state[5])  #limit on theta3dot
    dircol.AddConstraintToAllKnotPoints(state[5] <= speed_limit)

    # Collision avoidance of arm2 for front bar   
    #state[2], i.e. theta3 is w.r.t. tail. So, in reality, link2 config w.r.t. hook1 is phi3 = theta2 + theta3
    theta3 = state[2] + state[1]
    ee_y, ee_z = forward_kinematics(state[0], theta3, "arm")  #theta1 and theta3 because they are the ones related to the arms
    #dircol.AddConstraintToAllKnotPoints(ee_z <= 0.075)
    
    #ee_y_dot, ee_z_dot = forward_diff_kinematics(
    #    state[0], phi3, state[3], state[5], "arm" 
    #)
    front_bar_coordinate = [ladder_distance, 0.0]  # [y, z]
    R_bar_radius = 0.03  # 0.001   #assumed as the radius of the ladder branch
    distanceR_arm2 = np.sqrt(
        (front_bar_coordinate[0] - ee_y) ** 2
        + (front_bar_coordinate[1] - ee_z) ** 2
    )
    dircol.AddConstraintToAllKnotPoints(R_bar_radius <= distanceR_arm2)
#     dircol.AddRunningCost(-1 * distanceR_arm2sq)

    # Collision avoidance of arm2 for back bar
    ee_y, ee_z = forward_kinematics(state[0], theta3, "arm")
    #ee_y_dot, ee_z_dot = forward_diff_kinematics(
    #    state[0], phi3, state[3], state[5], "arm"
    #)
    back_bar_coordinate = [-1 * ladder_distance, 0.0]  # [y, z]
    distanceL_arm2 = np.sqrt(
        (back_bar_coordinate[0] - ee_y) ** 2
        + (back_bar_coordinate[1] - ee_z) ** 2
    )
    dircol.AddConstraintToAllKnotPoints(R_bar_radius / 10 <= distanceL_arm2)
    
    # Collision avoidance of arm2 for swing bar
    ee_y, ee_z = forward_kinematics(state[0], theta3, "arm")
    #ee_y_dot, ee_z_dot = forward_diff_kinematics(
    #    state[0], phi3, state[3], state[5], "arm"
    #)
    back_bar_coordinate = [0.0, 0.0]  # [y, z]
    distanceS_arm2 = np.sqrt(
        (back_bar_coordinate[0] - ee_y) ** 2
        + (back_bar_coordinate[1] - ee_z) ** 2
    )
    dircol.AddConstraintToAllKnotPoints(R_bar_radius * 3 <= distanceS_arm2)
    
    # Collision avoidance of tail for front bar  
    ee_y, ee_z = forward_kinematics(state[0], state[1], "tail")  
    
    #theta1 and theta2 because they are the ones related to the arm and tail
    #ee_y_dot, ee_z_dot = forward_diff_kinematics(
    #    state[0], state[1], state[3], state[4], "tail"
    #)
    front_bar_coordinate = [ladder_distance, 0.0]  # [y, z]
    R_bar_radius = 0.01  # 0.001   #assumed as the radius of the ladder branch
    distanceR_tail = np.sqrt(
        (front_bar_coordinate[0] - ee_y) ** 2
        + (front_bar_coordinate[1] - ee_z) ** 2
    )
    dircol.AddConstraintToAllKnotPoints(R_bar_radius <= distanceR_tail)

    # Collision avoidance of tail for back bar
    ee_y, ee_z = forward_kinematics(state[0], state[1], "tail")
    #ee_y_dot, ee_z_dot = forward_diff_kinematics(
    #    state[0], state[1], state[3], state[4], "tail"
    #)
    back_bar_coordinate = [-1 * ladder_distance, 0.0]  # [y, z]
    distanceL_tail = np.sqrt(
        (back_bar_coordinate[0] - ee_y) ** 2
        + (back_bar_coordinate[1] - ee_z) ** 2
    )
    dircol.AddConstraintToAllKnotPoints(R_bar_radius * 2 <= distanceL_tail)

    # Collision avoidance of tail for swing bar
    ee_y, ee_z = forward_kinematics(state[0], state[1], "tail")
    #ee_y_dot, ee_z_dot = forward_diff_kinematics(
    #    state[0], state[1], state[3], state[4], "tail"
    #)COMMENTING BECAUSE WE ARE LIMITING THE TAIL MOVEMENT DIRECTLY
#     back_bar_coordinate = [0.0, 0.0]  # [y, z]
#     distanceS_tail = np.sqrt(
#         (back_bar_coordinate[0] - ee_y) ** 2
#         + (back_bar_coordinate[1] - ee_z) ** 2
#     )
#     dircol.AddConstraintToAllKnotPoints(R_bar_radius <= distanceS_tail)
    
    #Constraint to limit the motion of both tail and arm2 under the ladder
    #ee_y_arm, ee_z_arm = forward_kinematics(state[0], state[2], "arm")
    #dircol.AddConstraintToAllKnotPoints(ee_z_arm <= 0)   ######!!!!!!COMMENTING FOR TESTING, UNCOMMENT LATER
    #dircol.AddConstraintToAllKnotPoints(ee_y_arm <= ladder_distance + 0.0)
    
    
    # Elbow angle limits for collision avoidance to attatched bar
    theta_limit = theta_limit
    # arm1 bounds
    dircol.AddConstraintToAllKnotPoints(state[0] <= theta_limit[0])
    dircol.AddConstraintToAllKnotPoints(-theta_limit[0] <= state[0])
    # tail bounds
    dircol.AddConstraintToAllKnotPoints(state[1] <= theta_limit[1])
    dircol.AddConstraintToAllKnotPoints(-theta_limit[1] <= state[1])
    # arm2 bounds
    dircol.AddConstraintToAllKnotPoints(state[2] <= theta_limit[2])
    dircol.AddConstraintToAllKnotPoints(-theta_limit[2] <= state[2])
    
    initial_state = initial_state
    
    
    prog.AddBoundingBoxConstraint(
        initial_state, initial_state, dircol.initial_state()
    )

#     prog.AddBoundingBoxConstraint(
#         final_state, final_state, dircol.final_state()
#     )
    print(f'dircol.final_state(): {dircol.final_state()}')
#     print(f'dircol.final_state()[1]: {dircol.final_state()[1]}')
#     print(f'dircol.final_state()[1].value: {dircol.final_state()[1].value()}')
    
    
    prog.AddConstraint(partial(forward_kinematics_array, component='arm', 
                               fin_y_pos=final_state[0], fin_z_pos=final_state[1]), 
                       lb=[-0.005] * 2,
                       ub=[0.005] * 2,
                       vars = np.array([dircol.final_state()[0], dircol.final_state()[1], dircol.final_state()[2]])
                      )
    
    prog.AddConstraint(partial(forward_diff_kinematics_array, component='arm', 
                               fin_y_vel=final_state[2], fin_z_vel=final_state[3]), 
                       lb=[-0.01] * 2,
                       ub=[0.01] * 2,
                       vars = np.array([dircol.final_state()[0], dircol.final_state()[1], dircol.final_state()[2],
                                       dircol.final_state()[3], dircol.final_state()[4], dircol.final_state()[5]])
                      )
    
    ## Costs

    # Input
    dircol.AddRunningCost((R * torque1_multip * u[0] ** 2) + (R * torque2_multip * u[1] ** 2))
    
    # Velocities
    #dircol.AddRunningCost(state[3] ** 2)
    #dircol.AddRunningCost(state[1])
    #dircol.AddRunningCost(state[2])
    dircol.AddRunningCost(K * theta2VelMultip * state[4] ** 2)
    dircol.AddRunningCost(K * theta3VelMultip * state[5] ** 2)
    #ee_y_tail, ee_z_tail = forward_kinematics(state[0], state[1], "tail")
    #if state[2] > 1 or state[2] <-1 :
    #    dircol.AddRunningCost(1000 * state[2])

    # Total duration
    dircol.AddFinalCost(dircol.time() * time_panalization)
    # dircol.AddFinalCost(np.sum(dircol.time()) * time_panalization)
    
    #traj = 'FB'
    # Initial guess
    #useRefTraj = input("Use reference trajectory?")
    if useRefTraj == 1:
        useRefTraj_char = 'y'
    else:
        useRefTraj_char = 'n'
    
    #useRefTraj = 'y'
    
    if useRefTraj_char == 'y':
        guess_traj_folder = f"data/trajectories/direct_collocation/{traj}"
        up_directory = 5
        file_name = f"{traj}_ref_traj.csv"
        print(f"Using reference trajectory, location and file name: -{up_directory} dirs /{guess_traj_folder}/{file_name}")
        guess_traj_path = generate_path(guess_traj_folder, file_name, up_directory)
        data_csv = pd.read_csv(guess_traj_path)
        initial_x_trajectory, _, _, _, _ = fit_polynomial(data=data_csv)
        
    else:
        print("Not using reference trajectory")
        initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
            init_guess, np.column_stack((initial_state, final_state))
        )  
    
#     dircol.SetInitialTrajectory(traj_init_u=initial_u_trajectory, traj_init_x=initial_x_trajectory)
    dircol.SetInitialTrajectory(PiecewisePolynomial(), traj_init_x=initial_x_trajectory)
    result = Solve(prog)
    print(f'Solution found? {result.is_success()}.')
    print(f"For the original problem, the solver status is {result.get_solution_result()}")
    infeasible_constraints = result.GetInfeasibleConstraints(prog)
    for c in infeasible_constraints:
        print(f"infeasible constraint: {c}")    
    assert result.is_success()
    hyper_params_dict = {
        "n": n,
        "tau_limit": tau_limit,
        "initial_state": initial_state,
        "theta_limit": theta_limit,
        "speed_limit": speed_limit,
        "ladder_distance": ladder_distance,
        "final_state": final_state,
        "theta2VelMultip"  : theta2VelMultip,
        "theta3VelMultip"  : theta3VelMultip,
        "K"  : K,
        "torque1_multip"  : torque1_multip,
        "torque2_multip"  : torque2_multip,
        "R": R,
        "time_panalization": time_panalization,
        "init_guess": init_guess,
        "max_timestep": max_timestep,
        "min_timestep": min_timestep,
        "useRefTraj": useRefTraj
    }
    return result, dircol, hyper_params_dict
    


def traj_opt_hyper(maneuver, bar_seperation):
    '''
    n               : Number of knot points
    tau_limit       : Input torque limit
    initial_state   : Initial state: [theta1, theta2,theta3, theta1_dot, theta2_dot, theta3_dot]
    theta_limit     : Position limits for [theta1, theta2, theta3]
    speed_limit     : Velocity limits for [theta1_dot, theta2_dot, theta3_dot]
    ladder_distance : Distance between two ladder bars in meter
    final_state     : Final state: [theta1, theta2,theta3, theta1_dot, theta2_dot, theta3_dot]
    theta2VelMultip : Theta 2 dot multiplier
    theta3VelMultip : Theta 3 dot multiplier
    K               : Velocity regulator
    torque1_multip  : Torque 1 multiplier
    torque2_multip  : Torque 2 multiplier
    R               : Input regulization term
    W               : Time panalization of the total trajectory
    init_guess      : Initial trajectory as initial guess for solver 
    useRefTraj      : Command (1 = yes, 0 = no) whether to use the reference trajectory or not
    '''
              
    #Making an assumption that all the support bars at the same height(horizontal ladder)
    
    if maneuver == "ZB":
        initialTheta1 = 0
        initialTheta2 = 0
        initialTheta3 = 0
        initialPhi3 = 0
        #desFinalTheta2 = np.deg2rad(-30)
        #desTheta2FinVel = -1
        #desTheta1FinVel = -0.5
        #desTheta3FinVel = -3.5
        desFinalTheta2 = -0.023598775598298816
        desTheta2FinVel = -0 + 1.6653345369377348e-16
        desTheta1FinVel = -0.5
        desTheta3FinVel = -3
        desFinalTheta1, desFinalTheta3 = inverse_geometry(-bar_seperation - 0.01, 0.0, "arm")
        desFinalPhi3 = desFinalTheta3 - desFinalTheta2
        desPhi3FinVel = desTheta3FinVel - desTheta2FinVel - 0.2
        
        #ee_y_vel, ee_z_vel = forward_diff_kinematics(desFinalTheta1, desFinalTheta2, desTheta1FinVel, desTheta2FinVel, "tail")
        #print(f"tail y and z velocities: {ee_y_vel, ee_z_vel}")
        #ee_y_vel, ee_z_vel = forward_diff_kinematics(desFinalTheta1, desFinalPhi3, desTheta1FinVel, desPhi3FinVel, "arm")
        #print(f"arm y and z velocities: {ee_y_vel, ee_z_vel}")
        
        #ee_y, ee_z = forward_kinematics(desFinalTheta1, desFinalTheta2, "tail")
        #print(f"tail y and z final position: {ee_y, ee_z}")
        #ee_y, ee_z = forward_kinematics(desFinalTheta1, desFinalPhi3, "arm")
        #print(f"arm y and z final position: {ee_y, ee_z}")
        
    elif maneuver == "ZF":
        initialTheta1 = 0
        initialTheta2 = 0
        initialTheta3 = 0
        initialPhi3 = 0
        
        desFinalTheta2 = 0.22359877559829875
        desFinalTheta1, desFinalTheta3 = inverse_geometry(bar_seperation + 0.02, 0.02, "arm")
        desFinalPhi3 = desFinalTheta3 - desFinalTheta2
        desTheta2FinVel = 0.08299535782605165
        desTheta1FinVel = -3
        desTheta3FinVel = -2.5
        desPhi3FinVel = desTheta3FinVel - desTheta2FinVel
        
#         desFinalTheta1, desFinalTheta3 = inverse_geometry(bar_seperation + 0.03, 0.02, "arm")
#         desFinalTheta2 = 0.1640122440170115
#         desTheta2FinVel = 7.081415897589816
#         desTheta1FinVel = -3
#         desTheta3FinVel = -2.5
#         desFinalPhi3 = desFinalTheta3 - desFinalTheta2
#         desPhi3FinVel = desTheta3FinVel - desTheta2FinVel

        
    elif maneuver == "FB":
        initialTheta1, initialTheta3 = inverse_geometry(bar_seperation + 0.02, 0.02, "arm")
        #initialTheta3 = 1.4
        #initialTheta2 = -0.5630468864964989
        initialTheta2 = -initialTheta1
        initialPhi3 = initialTheta3 - initialTheta2
        desFinalTheta2 = -0.023598775598298816
        desTheta2FinVel = -0.2#1.426597297568497
        desTheta1FinVel = -0.5
        desTheta3FinVel = -3
        desPhi3FinVel = desTheta3FinVel - desTheta2FinVel
        desFinalTheta1, desFinalTheta3 = inverse_geometry(-bar_seperation - 0.02, 0.02, "arm")
        desFinalPhi3 = desFinalTheta3 - desFinalTheta2 - 0.1
        
    elif maneuver == "BF":        
        initialTheta1, initialTheta3 = inverse_geometry(-bar_seperation,0, "arm")
        initialTheta1 = initialTheta1 - 0.02
        initialTheta3 = initialTheta3 + 0.01
        #initialTheta3 = -1.4
        initialTheta2 = -initialTheta1
        initialPhi3 = initialTheta3 - initialTheta2
        desFinalTheta1, desFinalTheta3 = inverse_geometry(bar_seperation + 0.08, -0.01, "arm")
        desFinalTheta1 = desFinalTheta1 + 0.02
        desFinalTheta3 = desFinalTheta3 + 0.1
        desFinalTheta2 = 0.1640122440170115
        desTheta2FinVel = 7.081415897589816
        desTheta1FinVel = -3
        desTheta3FinVel = -2.5
        desFinalPhi3 = desFinalTheta3 - desFinalTheta2
        desPhi3FinVel = desTheta3FinVel - desTheta2FinVel 
        print(f'final theta1: {desFinalTheta1} final theta2: {desFinalTheta2} final theta3: {desFinalTheta3} final phi3: {desFinalPhi3}')
        
    print(f'desFinalTheta1, desFinalTheta2, desFinalTheta3: {(desFinalTheta1), (desFinalTheta2), (desFinalTheta3)}.')

#COPY 'DIAMOND' TRAJECTORIES FOR THESE VARIABLES
    hyper_dict = {
        "ZB": [
            40,
            6, 
            (0, 0, 0, 0, 0.0, 0),
            [2.0943951023931953, 0.9, 2.8797932657906435],
            12,            
            bar_seperation,
            (-0.345, 0.0, 0.83, 0.75),
            1,
            1,
            100,
            1,
            70,
            100,
            0,
            [0.0, 3.0],
            1,
        ],   #
        "ZF": [
            60,
            6.0,
            (0, 0, 0, 0, 0, 0),
            [2.0943951023931953, 0.9, 2.8797932657906435],
            12,
            bar_seperation,
            (0.35, 0.025, 0.74, -1.4),
            1.2,
            1,
            200,
            2.6,
            2.2,
            200,
            0,
            [0.0, 3.2],
            1,
        ],# Got a good one here, I think
        "FB": [
            35,
            6.0,
            (initialTheta1, initialTheta2, initialPhi3,  -0.634777, 0, 4.677687 - 0),
            [2.0943951023931953, 0.9, 2.8797932657906435],
            12,
            bar_seperation,
            (-0.35, 0.00, 0.83, 0.75),  
            1.75,
            2,          
            54,
            1,
            1,
            120,
            50,
            [0.0, 1.5],
            1,
        ], #The old one is slow smooth and OK may be. This one is fast, smooth, and OK too maybe
        "BF": [
            40,
            4.0,
            (initialTheta1, initialTheta2, initialPhi3, -0.633643, 0, 1.756502 - 0),
            [2.0943951023931953, 0.9, 2.8797932657906435],
            12,
            bar_seperation,
            (0.36, 0.03, 0.9, -1.2),
            5,
            2,          
            100,
            60,
            50,  
            100,
            0,
            [0.0, 4.0],
            1,
        ],#This is good I think
    }
    return hyper_dict[f"{maneuver}"]


class DircolHypers():
    def __init__(
        self,
        n,
        tau_limit,
        initial_state,
        theta_limit,
        speed_limit,
        ladder_distance,
        final_state,
        theta2VelMultip,
        theta3VelMultip,
        K,
        torque1_multip,
        torque2_multip,
        R,
        W,
        init_guess,
        useRefTraj
    ):
    #fin_sta_neg_thre: Final state negative threshold for theta 2
    #fin_sta_pos_thre: Final state pos threshold for theta 2
    #fin_sta_neg_vel : Final state minimum angular velocity of tail (theta 2 dot min)
    #fin_sta_pos_vel : Final state maximum angular velocity of tail (theta 2 dot max)
        self.n = n
        self.tau_limit = tau_limit
        self.initial_state = initial_state
        self.theta_limit = theta_limit
        self.speed_limit = speed_limit
        self.ladder_distance = ladder_distance
        self.final_state = final_state
        self.theta2VelMultip = theta2VelMultip
        self.theta3VelMultip = theta3VelMultip
        self.K = K
        self.torque1_multip = torque1_multip
        self.torque2_multip = torque2_multip
        self.R = R
        self.W = W
        self.init_guess = init_guess
        self.useRefTraj = useRefTraj
    def create_params_structure(self):
        HYPER = namedtuple(
            "hyper_parameters",
            [
                "n",
                "tau_limit",
                "initial_state",
                "theta_limit",
                "speed_limit",
                "ladder_distance",
                "final_state",
                "theta2VelMultip",
                "theta3VelMultip",
                "K",
                "torque1_multip",
                "torque2_multip",
                "R",
                "W",
                "init_guess",
                "useRefTraj"
            ]
        )
        hyper = HYPER(
            n=self.n,
            tau_limit=self.tau_limit,
            initial_state=self.initial_state,
            theta_limit=self.theta_limit,
            speed_limit=self.speed_limit,
            ladder_distance=self.ladder_distance,
            final_state=self.final_state,
            theta2VelMultip=self.theta2VelMultip,
            theta3VelMultip=self.theta3VelMultip,
            K=self.K,
            torque1_multip=self.torque1_multip,
            torque2_multip=self.torque2_multip,
            R=self.R,
            W=self.W,
            init_guess=self.init_guess,
            useRefTraj=self.useRefTraj
        )
        return hyper


def load_default_hyperparameters(maneuver, bar_seperation):
    dircol_hyper = traj_opt_hyper(maneuver, bar_seperation)
    hyper_params = DircolHypers(
        n=dircol_hyper[0],
        tau_limit=dircol_hyper[1],
        initial_state=dircol_hyper[2],
        theta_limit=dircol_hyper[3],
        speed_limit=dircol_hyper[4],
        ladder_distance=dircol_hyper[5],
        final_state=dircol_hyper[6],
        theta2VelMultip=dircol_hyper[7],
        theta3VelMultip=dircol_hyper[8],
        K=dircol_hyper[9],
        torque1_multip=dircol_hyper[10],
        torque2_multip=dircol_hyper[11],
        R=dircol_hyper[12],
        W=dircol_hyper[13],
        init_guess=dircol_hyper[14],
        useRefTraj=dircol_hyper[15],
    )
    return hyper_params.create_params_structure()


def create_acromonk_plant(ladder_distance):
    from pydrake.all import MultibodyPlant, SceneGraph, Parser

    plant = MultibodyPlant(time_step=0.0)
    scene_graph = SceneGraph()
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    parser = Parser(plant)
    urdf_folder = "data/simulation_models"

    #fileObject = open("acromonk.urdf", "r")
    #urdf=fileObject.read()
    #fileObject.close()
    #urdf = urdf.replace("VARIABLE", "ladder_distance")
    #with open('finalAcromonk.urdf', 'w') as file:
     #   file.write(urdf)
        
#     file_name = "ricMonk.urdf"
    up_directory = 5
#     urdf_path = generate_path(urdf_folder, file_name, up_directory)
#     with open(urdf_path) as file:
#         urdf = file.read()

#     urdf = urdf.replace("VARIABLE", str(ladder_distance))

    
    file_name = "finalRicMonkCurrent.urdf"
    urdf_path = generate_path(urdf_folder, file_name, up_directory)
#     with open(urdf_path, 'w') as file:
#         file.write(urdf)    

    parser.AddModelFromFile(urdf_path)
    plant.Finalize()
    context = plant.CreateDefaultContext()
    return plant, context, scene_graph


def visualize_traj_opt(plant, scene_graph, x_trajectory):
    from pydrake.all import (
        DiagramBuilder,
        MultibodyPositionToGeometryPose,
        TrajectorySource,
    )

    builder = DiagramBuilder()
    source = builder.AddSystem(TrajectorySource(x_trajectory))
    builder.AddSystem(scene_graph)
    pos_to_pose = builder.AddSystem(
        MultibodyPositionToGeometryPose(plant, input_multibody_state=True)
    )
    builder.Connect(source.get_output_port(0), pos_to_pose.get_input_port())
    builder.Connect(
        pos_to_pose.get_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()),
    )
    intial_state = x_trajectory.value(x_trajectory.start_time())
    drake_visualizer(
        scene_graph, builder, initial_state=intial_state ,duration=x_trajectory.end_time()
    )
