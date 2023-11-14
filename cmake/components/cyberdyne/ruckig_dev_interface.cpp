#include "ruckig_dev_interface.h"

ruckig_dev_interface::ruckig_dev_interface()
{

}

extern "C" ruckig_dev_interface* ruckig_init_ptr(){
    return new ruckig_dev_interface();
}

extern "C" void ruckig_add_waypoint(ruckig_dev_interface *ptr, struct ruckig_c_waypoint point){
    ptr->pointvec.push_back(point);
}

extern "C" struct ruckig_c_waypoint ruckig_get_waypoint(ruckig_dev_interface *ptr, int index){
    return ptr->pointvec.at(index);
}

extern "C" int ruckig_waypoint_vector_size(ruckig_dev_interface *ptr){
    return ptr->pointvec.size();
}

extern "C" int ruckig_calculate_c(struct ruckig_c_data in, ruckig_c_data *out){
    return ruckig_dev_interface().ruckig_calculate(in,*out);
}

ruckig::Trajectory<1> trajectory;

int ruckig_dev_interface::ruckig_calculate(ruckig_c_data in, ruckig_c_data &out){

    out=in;
    out.at_time+=out.cycletime;

    //! Check for all kind of interupts.
    if(out.maxvel!=out.oldmaxvel || out.maxacc!=out.oldmaxacc ||  out.maxjerk!=out.oldmaxjerk ||
            out.tarvel!=out.oldtarvel || out.taracc!=out.oldtaracc || out.tarpos!=out.oldtarpos  ){
        // printf("Ruckig interupt. \n");

        //! For new calculation, set actual positions.
        out.curpos=out.newpos;
        out.curacc=out.newacc;
        out.curvel=out.newvel;

        out.initialized=0;
    }

    //! Ruckig input data format.
    ruckig::InputParameter<1> input;
    //! Convert c++ to c struct.
    input=ruckig_c_data_to_cpp(out);

    //! Calculate new motion.
    if(!out.initialized){

        // printf("Ruckig calculate new motion. \n");

        //! Calculate the trajectory in an offline manner (outside of the control loop)
        //! This is done to avoid a velocity end error when using the online trajectory.
        ruckig::Ruckig<1> otg;
        ruckig::Result result = otg.calculate(input,trajectory);

        //! Add code.
        out.function_return_code=result;

        out.duration=trajectory.get_duration();

        //! Update oldvel to trigger interupt next cycle.
        out.oldmaxvel=out.maxvel;
        out.oldmaxacc=out.maxacc;
        out.oldmaxjerk=out.maxjerk;
        out.oldtarvel=out.tarvel;
        out.oldtaracc=out.taracc;
        out.oldtarpos=out.tarpos;

        out.at_time=0;
        out.initialized=1;
    }

    //! Motion finished.
    if(out.at_time+0.001>out.duration){
        out.at_time=out.duration;
        out.function_return_code=1;

        //! Set to the given tar values.
        out.curpos=out.tarpos;
        out.curacc=out.taracc;
        out.curvel=out.tarvel;

        //! Return finished.
        return Finished;
    }

    //! Then, we can calculate the kinematic state at a given time
    std::array<double, 1> new_position, new_velocity, new_acceleration;
    trajectory.at_time(out.at_time, new_position, new_velocity, new_acceleration);

    //! Update out new pos, acc, vel.
    out.newpos=new_position[0];
    out.newvel=new_velocity[0];
    out.newacc=new_acceleration[0];

    return Working;
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
