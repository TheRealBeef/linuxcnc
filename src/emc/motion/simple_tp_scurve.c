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

        if(r.curpos!=r.tarpos){
            printf("simple_tp.c => ruckig input error: %i \n",r.function_return_code);
        }
        /*
            if(r.curpos==r.tarpos){ //! This is error code -100.
                printf("ruckig in position.\n");
            } else {
                printf("ruckig error code: %i \n",r.function_return_code);
                printf("period: %f \n",r.period);
                printf("tarpos: %f \n",r.tarpos);
                printf("curpos: %f \n",r.curpos);
                printf("curvel: %f \n",r.curvel);
                printf("curacc: %f \n",r.curacc);
                printf("maxvel: %f \n",r.maxvel);
                printf("maxacc: %f \n",r.maxacc);
                printf("maxjerk: %f \n",r.maxjerk);
            } */
    }

    if(r.finished){
        tp->active=false;
    }
    if(!r.finished){
        tp->curr_pos = r.curpos;
        tp->curr_vel = r.curvel;
        tp->curr_acc = r.curacc;

        tp->active=true;

        // printf("simple_tp active with max_jerk: %f \n",tp->max_jerk);
    }
}
