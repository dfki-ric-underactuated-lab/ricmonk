from pydrake.all import (
    PidController,
    Saturation,
    LogVectorOutput,
    TrajectorySource,
    Multiplexer, 
    Demultiplexer
)
def load_pid_controller(plant, context, builder, x0, u0, pid_gains, tau_limit):
    Kp_tail, Ki_tail, Kd_tail, Kp_arm2, Ki_arm2, Kd_arm2 = pid_gains
    pid_tail = PidController(Kp_tail, Ki_tail, Kd_tail)
    pid_arm2 = PidController(Kp_arm2, Ki_arm2, Kd_arm2)
    # Adding the controller to the builder
    regulator_theta2 = builder.AddSystem(pid_tail)
    regulator_theta2.set_name('PID_controller2')
    regulator_phi = builder.AddSystem(pid_arm2)
    regulator_phi.set_name('PID_controller3')

    # Connection of plant and controller
    saturation_theta2 = builder.AddSystem(Saturation(min_value=[-1 * tau_limit], max_value=[tau_limit]))
    saturation_theta2.set_name('saturation2')
    saturation_phi = builder.AddSystem(Saturation(min_value=[-1 * tau_limit], max_value=[tau_limit]))
    saturation_phi.set_name('saturation3')
    builder.Connect(regulator_theta2.get_output_port_control(), saturation_theta2.get_input_port())
    #builder.Connect(saturation_theta2.get_output_port(), plant.get_actuation_input_port())
    builder.Connect(regulator_phi.get_output_port_control(), saturation_phi.get_input_port())
    #builder.Connect(saturation_phi.get_output_port(), plant.get_actuation_input_port())
    
    #Giving input to the system
    mux_input = builder.AddSystem(Multiplexer(2))
    mux_input.set_name('inputMux')
    builder.Connect(saturation_theta2.get_output_port(), mux_input.get_input_port(0)) 
    builder.Connect(saturation_phi.get_output_port(), mux_input.get_input_port(1))
    
    builder.Connect(mux_input.get_output_port(), plant.get_actuation_input_port())
    
    # Spliting the states
    demux_state = builder.AddSystem(Demultiplexer(6 , 1))
    demux_state.set_name('state_splitter')
    builder.Connect(plant.get_state_output_port(), demux_state.get_input_port(0))
    mux_tail_states = builder.AddSystem(Multiplexer(2))
    mux_tail_states.set_name('tailStates')
    mux_arm2_states = builder.AddSystem(Multiplexer(2))
    mux_arm2_states.set_name('arm2States')
    # extract theta2
    builder.Connect(demux_state.get_output_port(1), mux_tail_states.get_input_port(0)) 
    # extract theta2_dot
    builder.Connect(demux_state.get_output_port(4), mux_tail_states.get_input_port(1)) 
    # extract phi
    builder.Connect(demux_state.get_output_port(2), mux_arm2_states.get_input_port(0)) 
    # extract phi_dot
    builder.Connect(demux_state.get_output_port(5), mux_arm2_states.get_input_port(1)) 
    # feed [theta2, theta2_dot] to PD    
    builder.Connect(mux_tail_states.get_output_port(), regulator_theta2.get_input_port_estimated_state())
    builder.Connect(mux_arm2_states.get_output_port(), regulator_phi.get_input_port_estimated_state())
    
    # Desired state for PID controller
    desired_trajectory = builder.AddSystem(TrajectorySource(x0))
    desired_trajectory.set_name('desired_states')
    
    # extract desired vectors
    demux_desired = builder.AddSystem(Demultiplexer(6 , 1))# spliting the states
    demux_desired.set_name('desired_state_splitter')
    builder.Connect(desired_trajectory.get_output_port(0), demux_desired.get_input_port(0))

    mux_tail_desired = builder.AddSystem(Multiplexer(2))
    mux_tail_desired.set_name('mux_tail_desired')
    mux_arm2_desired = builder.AddSystem(Multiplexer(2))
    mux_arm2_desired.set_name('mux_arm2_desired')
    builder.Connect(demux_desired.get_output_port(1), mux_tail_desired.get_input_port(0)) # extract theta2
    builder.Connect(demux_desired.get_output_port(4), mux_tail_desired.get_input_port(1)) # extract theta2_dot
    builder.Connect(demux_desired.get_output_port(2), mux_arm2_desired.get_input_port(0)) # extract phi
    builder.Connect(demux_desired.get_output_port(5), mux_arm2_desired.get_input_port(1)) # extract phi_dot
    builder.Connect(mux_tail_desired.get_output_port(), regulator_theta2.get_input_port_desired_state())# feed [theta2, theta2_dot] to PD 
    builder.Connect(mux_arm2_desired.get_output_port(), regulator_phi.get_input_port_desired_state())# feed [phi, phi_dot] to PD `
    # Save data loggers for sates and input torque
    input_tail_logger = LogVectorOutput(saturation_theta2.get_output_port(0), builder)
    input_arm2_logger = LogVectorOutput(saturation_phi.get_output_port(0), builder)
    state_logger = LogVectorOutput(plant.get_state_output_port(), builder)
    hyper_params_dict = {
        "Kp_tail": Kp_tail,
        "Ki_tail": Ki_tail,
        "Kd_tail": Kd_tail,
        "Kp_arm2": Kp_arm2,
        "Ki_arm2": Ki_arm2,
        "Kd_arm2": Kd_arm2,
        "Trajectory duration(seconds)":x0.end_time()
    }
    return (
     pid_tail,
#     pid_arm2,
     hyper_params_dict,
     builder,
     state_logger,
     input_tail_logger,
     input_arm2_logger
    )