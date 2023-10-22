/********************************************************************
* Description: simple_tp.c
*   A simple single axis trajectory planner.  See simple_tp.h for API.
*
* Authors:  jmkasunich
*           TheRealBeef
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
    //! Choose one of these configurations :
    use_no_scurve=0;            //! Use original lcnc source code, trapezium velocity profile.
    use_scurve_coded_skynet=1;  //! Scurve jogging example coded by skynet cyberdyne.

    if(use_scurve_coded_skynet){

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

    if(use_no_scurve){ // Use the original linuxcnc code:

        double max_dv, tiny_dp, pos_err, vel_req;

        tp->active = 0;
        /* compute max change in velocity per servo period */
        max_dv = tp->max_acc * period;
        /* compute a tiny position range, to be treated as zero */
        tiny_dp = TINY_DP(tp->max_acc, period);
        /* calculate desired velocity */
        if (tp->enable) {

            // printf("tp enabled \n");

            /* planner enabled, request a velocity that tends to drive
               pos_err to zero, but allows for stopping without position
               overshoot */
            pos_err = tp->pos_cmd - tp->curr_pos;
            /* positive and negative errors require some sign flipping to
               avoid sqrt(negative) */
            if (pos_err > tiny_dp) {
                vel_req = -max_dv +
                        sqrt(2.0 * tp->max_acc * pos_err + max_dv * max_dv);
                /* mark planner as active */
                tp->active = 1;
            } else if (pos_err < -tiny_dp) {
                vel_req =  max_dv -
                        sqrt(-2.0 * tp->max_acc * pos_err + max_dv * max_dv);
                /* mark planner as active */
                tp->active = 1;
            } else {
                /* within 'tiny_dp' of desired pos, no need to move */
                vel_req = 0.0;
            }


        } else {

            // printf("tp disabled \n");

            /* planner disabled, request zero velocity */
            vel_req = 0.0;
            /* and set command to present position to avoid movement when
               next enabled */
            tp->pos_cmd = tp->curr_pos;
        }

        /* limit velocity request */
        if (vel_req > tp->max_vel) {
            vel_req = tp->max_vel;
        } else if (vel_req < -tp->max_vel) {
            vel_req = -tp->max_vel;
        }

        /* ramp velocity toward request at accel limit */
        if (vel_req > tp->curr_vel + max_dv) {
            tp->curr_vel += max_dv;
        } else if (vel_req < tp->curr_vel - max_dv) {
            tp->curr_vel -= max_dv;
        } else {
            tp->curr_vel = vel_req;
        }
        /* check for still moving */
        if (tp->curr_vel != 0.0) {
            /* yes, mark planner active */
            tp->active = 1;
        }

        /* integrate velocity to get new position */
        tp->curr_pos += tp->curr_vel * period;
    }
}
