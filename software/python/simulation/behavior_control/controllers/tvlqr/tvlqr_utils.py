import sys
sys.path.append("../../utilities/")
from utils_polynomials import (
    fit_polynomial, 
    extract_data_from_polynomial, 
    PiecewisePolynomial, 
    create_gain_arrays
    )
from pydrake.all import (
    Saturation,
    LogVectorOutput,
    Multiplexer,
    Demultiplexer
)
from pydrake.all import VectorSystem, PiecewisePolynomial
class TvlqrControllerSystem(VectorSystem):
    def __init__(self, plant, tvlqr):
        VectorSystem.__init__(self, 6, 2)
        self.tvlqr_obj = tvlqr
    def DoCalcVectorOutput(self, context_simulation, ricMonk_state, unused, output):
        trajTime = context_simulation.get_time()
#         print(f'u0.value at time {trajTime}: \n {self.tvlqr_obj.u0.value(trajTime)}')
        xbar = ricMonk_state - (self.tvlqr_obj.x0.value(trajTime)).reshape(6,)
        output[0] = (self.tvlqr_obj.u0.value(trajTime) - 
                    (self.tvlqr_obj.K.value(trajTime).dot(xbar)) -
                     self.tvlqr_obj.k0.value(trajTime))[0][0]
        output[1] = (self.tvlqr_obj.u0.value(trajTime) - 
                    (self.tvlqr_obj.K.value(trajTime).dot(xbar)) -
                     self.tvlqr_obj.k0.value(trajTime))[1][0]
#         print(f'k0 shape: {self.tvlqr_obj.k0.value(trajTime).shape}') #  ==> (2,)


def load_tvlqr_controller(plant, context, builder, x0, u0, Q, R, Qf, tau_limit): #removed u1, u2
    from pydrake.all import (
        FiniteHorizonLinearQuadraticRegulatorOptions, 
        FiniteHorizonLinearQuadraticRegulator, 
        )
    #initialState = x0.value(x0.start_time())
    #print(f"Initial State shape = {initialState.shape}")
    #initialInput = u0.value(u0.start_time())
    #print(f"Initial Input shape = {initialInput.shape}")
    options = FiniteHorizonLinearQuadraticRegulatorOptions()
    options.x0 = x0
    options.u0 = u0
    Q = Q
    R = R
    options.Qf = Qf
    options.input_port_index = plant.get_actuation_input_port().get_index()
    tvlqr = FiniteHorizonLinearQuadraticRegulator(
        plant,
        context,
        t0=x0.start_time(),
        tf=x0.end_time(),
        Q=Q,
        R=R,
        options=options
        )
    hyper_params_dict = {
        "Q": Q,
        "Qf": Qf,
        "R": R,
        "Trajectory duration(seconds)":x0.end_time()
    }
    # Connect the diagram for simulation  
#     saturation = builder.AddSystem(Saturation(min_value=[-1 * tau_limit, -1 * tau_limit], max_value=[tau_limit, tau_limit]))
    mux_input = builder.AddSystem(Multiplexer(2))
    mux_input.set_name('inputMux')
    saturation_tail = builder.AddSystem(Saturation(min_value=[-1 * tau_limit], max_value=[tau_limit]))
    saturation_arm2 = builder.AddSystem(Saturation(min_value=[-1 * tau_limit], max_value=[tau_limit]))
#     builder.Connect(saturation.get_output_port(0), plant.get_actuation_input_port())
    builder.Connect(saturation_tail.get_output_port(0), mux_input.get_input_port(0))
    builder.Connect(saturation_arm2.get_output_port(0), mux_input.get_input_port(1))
    builder.Connect(mux_input.get_output_port(), plant.get_actuation_input_port())
    
    controller = builder.AddSystem(TvlqrControllerSystem(plant, tvlqr))
    builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))
    #print(f"controller.get_output_port size = {controller.get_output_port(0).size()}")
    #print(f"saturation.get_output_port size = {saturation.get_output_port(0).size()}")
    #builder.Connect(controller.get_output_port(0), saturation.get_input_port(0))
    # Save data loggers for sates and input torque
    
    #demux_saturation = builder.AddSystem(Demultiplexer(2, 1))# spliting the states
    #demux_saturation.set_name('desired_input_splitter')
    #builder.Connect(saturation.get_output_port(0), demux_saturation.get_input_port(0))
    demux_saturation = builder.AddSystem(Demultiplexer(2, 1))# spliting the states
#     demux_saturation = builder.AddSystem(Demultiplexer(1, 2))# spliting the states
    demux_saturation.set_name('desired_input_splitter')
    builder.Connect(controller.get_output_port(0), demux_saturation.get_input_port(0))
    builder.Connect(demux_saturation.get_output_port(0), saturation_tail.get_input_port(0))
    builder.Connect(demux_saturation.get_output_port(1), saturation_arm2.get_input_port(0))
    
    
    #input_tail_logger = LogVectorOutput(demux_saturation.get_output_port(0), builder)
    #input_arm2_logger = LogVectorOutput(demux_saturation.get_output_port(1), builder)
    input_tail_logger = LogVectorOutput(saturation_tail.get_output_port(0), builder)
    input_arm2_logger = LogVectorOutput(saturation_arm2.get_output_port(0), builder)
    state_logger = LogVectorOutput(plant.get_state_output_port(), builder)
    return (
     tvlqr, 
     hyper_params_dict,
     builder,
     state_logger,
     input_tail_logger,
     input_arm2_logger
    )




