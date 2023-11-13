#include "ruckig_dev_interface.h"

ruckig_dev_interface::ruckig_dev_interface()
{

}


ruckig_c_data ruckig_dev_interface::ruckig_calculate_offline(ruckig_c_data in){

    // Ruckig example: 02_position_offline.cpp
    ruckig::InputParameter<1> input;
    input=ruckig_c_data_to_cpp(in);

    // We don't need to pass the control rate (cycle time) when using only offline features
    ruckig::Ruckig<1> otg;
    ruckig::Trajectory<1> trajectory;

    // Calculate the trajectory in an offline manner (outside of the control loop)
    ruckig::Result result = otg.calculate(input, trajectory);
    if (result == ruckig::Result::ErrorInvalidInput) {
        std::cout << "Invalid input!" << std::endl;
    }

    in.function_return_code=result;

    // Get duration of the trajectory
    in.duration=trajectory.get_duration();

    // Then, we can calculate the kinematic state at a given time
    std::array<double, 1> new_position, new_velocity, new_acceleration;
    trajectory.at_time(in.at_time, new_position, new_velocity, new_acceleration);

    in.curpos=new_position[0];
    in.curvel=new_velocity[0];
    in.curacc=new_acceleration[0];


    // Get some info about the position extrema of the trajectory
    std::array<ruckig::PositionExtrema, 1> position_extrema = trajectory.get_position_extrema();

    in.pos_extrema_min=position_extrema[0].min;
    in.pos_extrema_max=position_extrema[0].max;

    return in;
}

extern "C" struct ruckig_c_data ruckig_calculate_c_offline(struct ruckig_c_data in){
    struct ruckig_c_data out=ruckig_dev_interface().ruckig_calculate_offline(in);
    return out;
}

ruckig_c_data ruckig_dev_interface::ruckig_calculate_online(ruckig_c_data in){

    ruckig::Ruckig<1> otg {in.cycletime};

    ruckig::InputParameter<1> input;
    input=ruckig_c_data_to_cpp(in);

    ruckig::OutputParameter<1> output;

    ruckig::Result result = otg.update(input, output);

    double new_position, new_velocity, new_acceleration;
    output.trajectory.at_time(in.at_time, new_position, new_velocity, new_acceleration);

    //! Update input with new pos, acc, vel.
    output.pass_to_input(input);

    //! Transfer to c style.
    ruckig_c_data out=ruckig_cpp_data_to_c(input);

    //! Add code.
    out.function_return_code=result;

    //! Add duration.
    out.duration=output.trajectory.get_duration();

    return out;
}

extern "C" struct ruckig_c_data ruckig_calculate_c_online(struct ruckig_c_data in){
    struct ruckig_c_data out=ruckig_dev_interface().ruckig_calculate_online(in);
    return out;
}


ruckig_c_data ruckig_dev_interface::ruckig_cpp_data_to_c(ruckig::InputParameter<1> input){

    ruckig_c_data in;

    int i=0;

    //! Is the DoF considered for calculation?
    in.enable=input.enabled[i];

    //! Position, ///< Position-control: Full control over the entire kinematic state (Default)
    //! Velocity, ///< Velocity-control: Ignores the current position, target position, and velocity limits
    if(input.control_interface==ruckig::ControlInterface::Position){
        in.control_interfacetype=position;
    } else {
        in.control_interfacetype=velocity;
    }

    //! Continuous, ///< Every trajectory synchronization duration is allowed (Default)
    //! Discrete, ///< The trajectory synchronization duration must be a multiple of the control cycle
    if(input.duration_discretization==ruckig::DurationDiscretization::Continuous){
        in.durationdiscretizationtype=Continuous;
    } else {
        in.durationdiscretizationtype=Discrete;
    }

    //! Phase, ///< Phase synchronize the DoFs when possible, else fallback to "Time" strategy
    //! Time, ///< Always synchronize the DoFs to reach the target at the same time (Default)
    //! TimeIfNecessary, ///< Synchronize only when necessary (e.g. for non-zero target velocity or acceleration)
    //! None, ///< Calculate every DoF independently
    if(input.synchronization==ruckig::Synchronization::None){
        in.synchronizationtype=None;
    }
    if(input.synchronization==ruckig::Synchronization::Phase){
        in.synchronizationtype=Phase;
    }
    if(input.synchronization==ruckig::Synchronization::Time){
        in.synchronizationtype=Time;
    }
    if(input.synchronization==ruckig::Synchronization::TimeIfNecessary){
        in.synchronizationtype=TimeIfNecessary;
    }

    //! Current state.
    in.curpos=input.current_position[i];
    in.curvel=input.current_velocity[i];
    in.curacc=input.current_acceleration[i];

    //! Target state
    in.tarpos=input.target_position[i];
    in.tarvel=input.target_velocity[i];
    in.taracc=input.target_acceleration[i];

    //! Kinematic constraints
    in.maxvel=input.max_velocity[i];
    in.maxacc=input.max_acceleration[i] ;
    in.maxjerk=input.max_jerk[i];

    return in;
}

ruckig::InputParameter<1> ruckig_dev_interface::ruckig_c_data_to_cpp(ruckig_c_data in){

    ruckig::InputParameter<1> input;

    int i=0;

    //! Is the DoF considered for calculation?
    input.enabled[i]=in.enable;

    //! Position, ///< Position-control: Full control over the entire kinematic state (Default)
    //! Velocity, ///< Velocity-control: Ignores the current position, target position, and velocity limits
    if(in.control_interfacetype==position){
        input.control_interface=ruckig::ControlInterface::Position;
    } else {
        input.control_interface=ruckig::ControlInterface::Velocity;
    }

    //! Continuous, ///< Every trajectory synchronization duration is allowed (Default)
    //! Discrete, ///< The trajectory synchronization duration must be a multiple of the control cycle
    if(in.durationdiscretizationtype==Continuous){
        input.duration_discretization=ruckig::DurationDiscretization::Continuous;
    } else {
        input.duration_discretization=ruckig::DurationDiscretization::Discrete;
    }

    //! Phase, ///< Phase synchronize the DoFs when possible, else fallback to "Time" strategy
    //! Time, ///< Always synchronize the DoFs to reach the target at the same time (Default)
    //! TimeIfNecessary, ///< Synchronize only when necessary (e.g. for non-zero target velocity or acceleration)
    //! None, ///< Calculate every DoF independently
    if(in.synchronizationtype==None){
        input.synchronization=ruckig::Synchronization::None;
    }
    if(in.synchronizationtype==Phase){
        input.synchronization=ruckig::Synchronization::Phase;
    }
    if(in.synchronizationtype==Time){
        input.synchronization=ruckig::Synchronization::Time;
    }
    if(in.synchronizationtype==TimeIfNecessary){
        input.synchronization=ruckig::Synchronization::TimeIfNecessary;
    }

    //! Current state.
    input.current_position[i] = in.curpos;
    input.current_velocity[i] = in.curvel;
    input.current_acceleration[i] = in.curacc;

    //! Target state
    input.target_position[i] = in.tarpos;
    input.target_velocity[i] = in.tarvel;
    input.target_acceleration[i] = in.taracc;

    //! Kinematic constraints
    input.max_velocity[i] = in.maxvel;
    input.max_acceleration[i] = in.maxacc;
    input.max_jerk[i] = in.maxjerk;

    return input;
}

