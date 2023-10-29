/********************************************************************
* Description: simple_tp.c
*   A simple single axis trajectory planner.  See simple_tp.h for API.
*
* Authors:  TheRealBeef
*           Skynet Cyberdyne alias Grotius
* License: GPL Version 2
* Created on:
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
********************************************************************/

#include "simple_tp.h"
#include "rtapi_math.h"
#include "stdio.h"
#include "ruckig_format.h"

extern struct result wrapper_get_pos(struct result input);
struct result r, restore;
bool use_no_scurve;
bool use_scurve_coded_skynet;

//! For every joint this function is called.
void simple_tp_update(simple_tp_t *tp, double period)
{
    // printf("simple_tp_update. \n");

    // printf("jerk: %f \n",tp->max_jerk);

    //! When jog button pressed. The pos_cmd seems to be 500mm away from current tp position.
    r.period=period;
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

    if(tp->enable){ //! Button pressed, try to move.
        r.interfacetype=0;
    } else { //! Button released, try to stop.
        r.interfacetype=1;
    }

    r=wrapper_get_pos(r); //! Calculate.

    //! When ruckig input's are invalid, ruckig will give error message.
    //! This is also the case when curpos=tarpos.
    if(r.error){
        // printf("ruckig error code: %i \n",r.function_return_code);
        r.finished=true;
    }

    if(r.finished){
        tp->active=false;
        //! Repaired a error when homed in auto without using jogging, after program stop, machine position
        //! set to x0 y0 z0.
        tp->pos_cmd = tp->curr_pos;
    }
    if(!r.finished){
        tp->curr_pos = r.curpos;
        tp->curr_vel = r.curvel;
        tp->curr_acc = r.curacc;

        tp->active=true;

        // printf("simple_tp active with max_jerk: %f \n",tp->max_jerk);
    }
}
