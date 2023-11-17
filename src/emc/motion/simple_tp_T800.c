/********************************************************************
* Description: simple_tp_T800.c
*   A simple single axis trajectory planner.  See simple_tp.h for API.
*
* Authors: Skynet Cyberdyne alias Grotius
* License: GPL Version 2
* Created on:
* System: Linux
*
* Copyright (c) 2023 All rights reserved.
********************************************************************/

#include "simple_tp.h"
#include "rtapi_math.h"
#include "stdio.h"
#include "ruckig_dev_format.h"
#include "ruckig_dev_online_interface.h"

extern struct ruckig_c_data wrapper_get_pos(struct ruckig_c_data input);
struct ruckig_c_data r;

//! For every joint this function is called.
void simple_tp_update(simple_tp_t *tp, double period)
{
    // printf("simple_tp_update. \n");

    //! When jog button pressed. The pos_cmd seems to be 500mm away from current tp position.
    r.cycletime=period;
    r.tarpos=tp->pos_cmd;
    r.curpos=tp->curr_pos;
    r.curvel=tp->curr_vel;
    r.curacc=tp->curr_acc;
    r.maxvel=tp->max_vel;
    r.maxacc=tp->max_acc;
    r.enable=1;
    r.maxjerk=tp->max_jerk;
    r.synchronizationtype=3;
    r.durationdiscretizationtype=0;
    r.taracc=0;
    r.tarvel=0;
    r.reset=0;

    if(tp->enable){ //! Button pressed, try to move.
        r.control_interfacetype=0;
    } else { //! Button released, try to stop.
        r.control_interfacetype=1;
    }

    r=wrapper_get_pos(r); //! Calculate.

    //! When ruckig input's are invalid, ruckig will give error message.
    //! This is also the case when curpos=tarpos.
    if(r.function_return_code<0){
        // printf("ruckig error code: %i \n",r.function_return_code);
        r.function_return_code=1;
    }

    if(r.function_return_code==1){
        tp->active=false;
        //! Repaired a error when homed in auto without using jogging, after program stop, machine position
        //! set to x0 y0 z0.
        tp->pos_cmd = tp->curr_pos;
    }
    if(r.function_return_code==0){
        tp->curr_pos = r.curpos;
        tp->curr_vel = r.curvel;
        tp->curr_acc = r.curacc;

        tp->active=true;
        // printf("simple_tp active with max_jerk: %f \n",tp->max_jerk);
    }
}
