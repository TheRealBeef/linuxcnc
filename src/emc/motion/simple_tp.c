/********************************************************************
* Description: simple_tp.c
*   A simple single axis trajectory planner.  See simple_tp.h for API.
*
* Author: jmkasunich & skynet cyberdyne for scurve tests 2023.
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
bool use_scurve;

//! For every joint this function is called.
void simple_tp_update(simple_tp_t *tp, double period)
{
    use_scurve=true;

    if(use_scurve){
        // Determine a min distance that we aren't going to worry about
        double tiny_dp = TINY_DP(tp->max_acc, period);
        // printf("period: %f \n", period); // Seems to be : 0.001
        // printf("max_jerk: %f \n", tp->max_jerk);

        //! When jog button pressed. The pos_cmd seems to be 500mm away from current tp position.
        //! Does it looks for machine limits?

        r.period=period;
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

        restore=r;

        if(tp->enable){ //! Button pressed, try to move.
            // For the target position, check if the error is less than tiny_dp
            // and if so, set target position to current position to stop
            if (fabs(tp->pos_cmd - tp->curr_pos) <= tiny_dp) {
                r.tarpos = tp->curr_pos;
            } else {
                r.tarpos=tp->pos_cmd;
            }
            r.interfacetype=0;
        } else { //! Button released, try to stop.
            r.tarpos=tp->curr_pos;
            r.interfacetype=1;
        }

        r=wrapper_get_pos(r); //! Calculate.

        //! When a nan uccur's, restore values.
        // It appears that  nan's appear more frequently when period is too short...but may be another cause
        if(isnan(r.curpos)){
            printf("recieved nan value, fixing. \n");
            r=restore;
        }

        // If the desired position is further than
        if(fabs(r.tarpos - r.curpos) > tiny_dp) {
            // Ensure that ruckig doesn't somehow give too high vel values
            if (r.curvel > tp->max_vel) {
                r.curvel = tp->max_vel;
            } else if (r.curvel < -tp->max_vel) {
                r.curvel = -tp->max_vel;
            }
            // Ensure that ruckig doesn't somehow give too high accel  values
            if (r.curacc > tp->max_acc) {
                r.curacc = tp->max_acc;
            } else if (r.curacc < -tp->max_acc) {
                r.curacc = -tp->max_acc;
            }

            //! Update results.
            tp->curr_pos = r.curpos;
            tp->curr_vel = r.curvel;
            tp->curr_acc = r.curacc;
            tp->active=1;
        } else {
            tp->active=0;
        }

//        if(r.curvel != 0.0) {
//            tp->active=1;
//        } else {
//            tp->active=0;
//        }


    } else {

        // Use the original linuxcnc code:

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
