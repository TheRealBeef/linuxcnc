
#include "rtapi.h"
#include "rtapi_ctype.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "rtapi_errno.h"
#include "rtapi_math64.h"
#include <rtapi_io.h>
#include "hal.h"
#include "stdio.h"

#include "emcpose.h"
#include "motion.h"
#include "tc.h"
#include "tp_scurve.h"
#include "tp_vector.h"
#include "tp_conversion.h"
#include "tp_arcs.h"
#include "tp_lines.h"

#include "ruckig_format.h"

/* module information */
MODULE_AUTHOR("Skynet_Cyberdyne");
MODULE_DESCRIPTION("tpmod_scurve");
MODULE_LICENSE("GPL2");

static int comp_idx;

typedef struct {
    bool ok;
} skynet_t;
skynet_t *skynet;

typedef struct {
    hal_float_t *Pin;
} float_data_t;
float_data_t *tp_curvel, *tp_curacc;
//! Pins
typedef struct {
    hal_bit_t *Pin;
} bit_data_t;

typedef struct { //! Int.
    hal_s32_t *Pin;
} s32_data_t;

typedef struct { //! Param int.
    hal_s32_t Pin;
} param_s32_data_t;

typedef struct { //! Uint.
    hal_u32_t *Pin;
} u32_data_t;

typedef struct { //! Param Uint.
    hal_u32_t Pin;
} param_u32_data_t;

typedef struct {
    hal_port_t *Pin;
} port_data_t;

//! Params
typedef struct {
    hal_float_t Pin;
} param_float_data_t;

typedef struct {
    hal_bit_t Pin;
} param_bit_data_t;
param_bit_data_t *done;

static int comp_idx; /* component ID */

static void the_function();
static int setup_pins();

int rtapi_app_main(void) {

    int r = 0;
    comp_idx = hal_init("tpmod_scurve");
    if(comp_idx < 0) return comp_idx;
    r = hal_export_funct("tpmod_scurve", the_function, &skynet,0,0,comp_idx);

    r+=setup_pins();

    if(r) {
        hal_exit(comp_idx);
    } else {
        hal_ready(comp_idx);
    }
    return 0;
}

void rtapi_app_exit(void){
    hal_exit(comp_idx);
}

//! Perforn's every ms.
static void the_function(){

}

//! Setup hal pins.
static int setup_pins(){
    int r=0;

    //! Pins to be motitored by halscope.
    tp_curvel = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("tpmod_scurve.curvel",HAL_OUT,&(tp_curvel->Pin),comp_idx);

    tp_curacc = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("tpmod_scurve.curacc",HAL_OUT,&(tp_curacc->Pin),comp_idx);

    return r;
}

//! Status and config from motion.h
static emcmot_status_t *emcmotStatus;
static emcmot_config_t *emcmotConfig;

//==========================================================
// tp module interface
// motmod function ptrs for functions called by tp:
static void(  *_DioWrite)(int,char);
static void(  *_AioWrite)(int,double);
static void(  *_SetRotaryUnlock)(int,int);
static int (  *_GetRotaryIsUnlocked)(int);
static double(*_axis_get_vel_limit)(int);
static double(*_axis_get_acc_limit)(int);

void tpMotFunctions(void(  *pDioWrite)(int,char)
                    ,void(  *pAioWrite)(int,double)
                    ,void(  *pSetRotaryUnlock)(int,int)
                    ,int (  *pGetRotaryIsUnlocked)(int)
                    ,double(*paxis_get_vel_limit)(int)
                    ,double(*paxis_get_acc_limit)(int)
                    )
{
    _DioWrite            = *pDioWrite;
    _AioWrite            = *pAioWrite;
    _SetRotaryUnlock     = *pSetRotaryUnlock;
    _GetRotaryIsUnlocked = *pGetRotaryIsUnlocked;
    _axis_get_vel_limit  = *paxis_get_vel_limit;
    _axis_get_acc_limit  = *paxis_get_acc_limit;
}

void tpMotData(emcmot_status_t *pstatus
               ,emcmot_config_t *pconfig
               )
{
    emcmotStatus = pstatus;
    emcmotConfig = pconfig;
}

//! To use functions from tp_vector.cpp we need to declare them here:
extern struct tp_vector* vector_init_ptr();
extern int vector_size_c(struct tp_vector *ptr);
extern void vector_clear(struct tp_vector *ptr);
extern int vector_at_id(struct tp_vector *ptr, int n);
extern struct tp_segment vector_at(struct tp_vector *ptr, int index);
extern void vector_add_segment(struct tp_vector *ptr, struct tp_segment b);

extern double arc_lenght_c(struct sc_pnt start, struct sc_pnt way, struct sc_pnt end);
extern double line_lenght_c(struct sc_pnt start, struct sc_pnt end);
extern void interpolate_line_c(struct sc_pnt p0, struct sc_pnt p1, double progress, struct  sc_pnt *pi);
extern void interpolate_dir_c(struct sc_dir p0, struct sc_dir p1, double progress, struct sc_dir *pi);
extern void interpolate_ext_c(struct sc_ext p0, struct sc_ext p1, double progress, struct sc_ext *pi);
extern void interpolate_arc_c(struct sc_pnt p0, struct sc_pnt p1, struct sc_pnt p2, double progress, struct sc_pnt *pi);
extern void sc_arc_get_mid_waypoint_c(struct sc_pnt start, struct sc_pnt center, struct sc_pnt end, struct sc_pnt *waypoint);
extern void vector_interpolate_traject_c(struct tp_vector *ptr, double traject_progress, double traject_lenght, double *curve_progress, int *curve_nr);

//! Gcode vector dynamic.
struct tp_vector *vector_ptr;

//! Ruckig scurve.
extern struct result wrapper_get_pos(struct result input);
struct result r={};

void update_gui(TP_STRUCT * const tp);
void update_ruckig(TP_STRUCT * const tp);

struct sc_pnt xyz;
struct sc_dir abc;
struct sc_ext uvw;

//! Create a empty queue.
int tpInit(TP_STRUCT * const tp)
{
    printf("tpInit. \n");

    return 0;
}

int tpRunCycle(TP_STRUCT * const tp, long period)
{
    //! printf("tpRunCycle. \n");
    // tp->cycleTime=period;

    //! Plan motion if the segment vector > 0
    update_ruckig(tp);

    //! Interpolate tp position.
    update_gui(tp);

    return 0;
}

//! The first function call.
int tpCreate(TP_STRUCT * const tp, int _queueSize,int id)
{
    if (_queueSize <= 0) {
        tp->queueSize = TP_DEFAULT_QUEUE_SIZE;
    } else {
        tp->queueSize = _queueSize;
    }

    //! Set the queue size to the c++ vector.
    vector_ptr=vector_init_ptr();

    printf("tpCreate. set tp->queuesize to: %i \n", tp->queueSize);

    return 0;
}

int tpSetMaxJerk(TP_STRUCT * const tp, double max_jerk)
{
    if (!tp || max_jerk <= 0.0) {
        return -1;
    }

    tp->max_jerk=max_jerk;

    printf("tpSetMaxJerk to: %f \n",max_jerk);
    return 0;
}

int tpClear(TP_STRUCT * const tp)
{
    printf("tpClear. \n");

    vector_clear(vector_ptr);
    vector_ptr=NULL;

    return 0;
}

int tpSetCycleTime(TP_STRUCT * const tp, double secs)
{
    if (!tp || secs <= 0.0) {
        return -1;
    }

    tp->cycleTime = secs;
    printf("tpSetCycleTime to: %f \n",tp->cycleTime);
    return 0;
}

int tpSetVmax(TP_STRUCT * const tp, double vMax, double ini_maxvel)
{
    if (!tp || vMax <= 0.0 || ini_maxvel <= 0.0) {
        return -1;
    }

    tp->vMax = vMax;
    tp->ini_maxvel = ini_maxvel;

    printf("tpSetVmax to: %f ",tp->vMax);
    printf(" , ini_maxvel to: %f \n",tp->ini_maxvel);
    return 0;
}

int tpSetVlimit(TP_STRUCT * const tp, double vLimit)
{
    if(!tp){ return -1;}

    if (vLimit < 0.0){
        tp->vLimit = 0.;
    } else {
        tp->vLimit = vLimit;
    }

    printf("tpSetVlimit. to: %f \n",tp->vLimit);
    return 0;
}

int tpSetAmax(TP_STRUCT * const tp, double aMax)
{
    if (!tp || aMax <= 0.0) {
        return -1;
    }

    tp->aMax=aMax;
    printf("tpSetAmax to: %f \n",tp->aMax);
    return 0;
}

int tpSetId(TP_STRUCT * const tp, int id)
{
    if (!tp) {
        return -1;
    }

    //! printf("tpSetId. \n");

    //! Set gcode line nr for upcoming new line, arc.
    tp->gcode_upcoming_line_nr=id;

    return 0;
}

int tpGetExecId(TP_STRUCT * const tp)
{
    //! printf("tpGetExecId. \n");

    //! This is the executed gcode line nr. The gui's gcode preview
    //! uses this to set the line.

    return tp->gcode_current_executed_line_nr;
}

int tpSetTermCond(TP_STRUCT * const tp, int cond, double tolerance)
{
    return 0;
}

/**
 * Used to tell the tp the initial position.
 * It sets the current position AND the goal position to be the same.  Used
 * only at TP initialization and when switching modes.
 */
int tpSetPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (0 == tp) {
        return -1;
    }

    // printf("tpSetPos. \n");
    printf("tpSetPos x: %f y: %f z: %f \n",pos->tran.x,pos->tran.y,pos->tran.z);

    tp->currentPos=*pos;

    // printf("vector size: %i \n",vector_size_c(vector_ptr));
    // printf("tpCurrentPos x: %f y: %f z: %f \n",tp->currentPos.tran.x,tp->currentPos.tran.y,tp->currentPos.tran.z);

    return 0;
}

int tpGetPos(TP_STRUCT const * const tp, EmcPose * const pos)
{
    if (0 == tp) {
        return -1;
    }

    //! The gui toolposition tp is updated from here.
    *pos = tp->currentPos;
    printf("tpGetPos x: %f y: %f z: %f \n",pos->tran.x,pos->tran.y,pos->tran.z);


    return 0;
}

int tpErrorCheck(TP_STRUCT const * const tp) {

    return 0;
}

int tpSetSpindleSync(TP_STRUCT * const tp, int spindle, double sync, int mode) {
    return 0;
}

int tpPause(TP_STRUCT * const tp)
{
    printf("tpPause. \n");

    tp->pausing=1;

    return 0;
}

int tpResume(TP_STRUCT * const tp)
{
    printf("tpResume. \n");

    tp->pausing=0;

    return 0;
}

int tpAbort(TP_STRUCT * const tp)
{
    printf("tpAbort. \n");

    vector_clear(vector_ptr);
    tp->vector_size=0;

    return 0;
}

int tpGetMotionType(TP_STRUCT * const tp)
{
    return tp->motionType;
}

int tpIsDone(TP_STRUCT * const tp)
{
    if(tp->vector_size==0){
        tp->vector_current_exec=0;
        tp->segment_progress=0;
        tp->cur_pos=0;
        tp->tar_pos=0;
        return 1;
    }
    return 0;
}

int tpQueueDepth(TP_STRUCT * const tp)
{
    return 0;
}

int tpActiveDepth(TP_STRUCT * const tp)
{
    return 0;
}

int tpSetAout(TP_STRUCT * const tp, unsigned char index, double start, double end) {
    return 0;
}

int tpSetDout(TP_STRUCT * const tp, int index, unsigned char start, unsigned char end) {
    return 0;
}

int tpSetRunDir(TP_STRUCT * const tp, tc_direction_t dir){

    printf("tpSetRunDir, motion reverse : %i \n",dir);

    tp->reverse_run=dir;
    return 0;
}

int tpAddRigidTap(TP_STRUCT * const tp,
                  EmcPose end,
                  double vel,
                  double ini_maxvel,
                  double acc,
                  unsigned char enables,
                  double scale,
                  struct state_tag_t tag) {

    printf("tpAddRigidTap \n");

    return 0;
}

int tpAddLine(TP_STRUCT *
              const tp,
              EmcPose end,
              int canon_motion_type,
              double vel,
              double ini_maxvel,
              double acc,
              unsigned char enables,
              char atspeed,
              int indexer_jnum,
              struct state_tag_t tag)


{
    printf("tpAddLine \n");

    if(tp->vector_size==0){
        tp->gcode_lastPos=tp->currentPos;
    }

    struct tp_segment b;
    b.primitive_id=sc_line;
    b.type=canon_motion_type;
    b.pnt_s=emc_pose_to_sc_pnt(tp->gcode_lastPos);
    b.pnt_w.x=0;
    b.pnt_w.y=0;
    b.pnt_w.z=0;
    b.pnt_c.x=0;
    b.pnt_c.y=0;
    b.pnt_c.z=0;
    b.angle_end_deg=0;

    b.pnt_e=emc_pose_to_sc_pnt(end);

    b.dir_s=emc_pose_to_sc_dir(tp->gcode_lastPos);
    b.dir_e=emc_pose_to_sc_dir(end);

    b.ext_s=emc_pose_to_sc_ext(tp->gcode_lastPos);
    b.ext_e=emc_pose_to_sc_ext(end);

    b.gcode_line_nr=tp->gcode_upcoming_line_nr;

    b.vo=0;
    b.vm=vel;
    b.ve=0;

    b.path_lenght=line_lenght_c(b.pnt_s,b.pnt_e);

    vector_add_segment(vector_ptr,b);
    tp->vector_size=vector_size_c(vector_ptr);
    printf("vector size: %i \n",tp->vector_size);

    //! Update last pose to end of gcode block.
    tp->gcode_lastPos=end;

    tp->traject_lenght+=b.path_lenght;
    printf("lengt of this segment: %f \n",b.path_lenght);
    printf("traject lenght now: %f \n",tp->traject_lenght);

    //! Clear.
    tp->vector_current_exec=0;
    tp->segment_progress=0;
    tp->cur_pos=0;
    tp->tar_pos=0;

    return 0;
}

int tpAddCircle(TP_STRUCT * const tp,
                EmcPose end,
                PmCartesian center,
                PmCartesian normal,
                int turn,
                int canon_motion_type, //! arc_3->lin_2->GO_1
                double vel,
                double ini_maxvel,
                double acc,
                unsigned char enables,
                char atspeed,
                struct state_tag_t tag)
{
    printf("tpAddCircle. \n");

    if(tp->vector_size==0){
        tp->gcode_lastPos=tp->currentPos;
    }

    struct tp_segment b;
    b.primitive_id=sc_arc;
    b.type=canon_motion_type;
    b.pnt_s=emc_pose_to_sc_pnt(tp->gcode_lastPos);

    b.dir_s=emc_pose_to_sc_dir(tp->gcode_lastPos);
    b.dir_e=emc_pose_to_sc_dir(end);

    b.ext_s=emc_pose_to_sc_ext(tp->gcode_lastPos);
    b.ext_e=emc_pose_to_sc_ext(end);

    //! Create a 3d arc using waypoint technique.
    sc_arc_get_mid_waypoint_c(emc_pose_to_sc_pnt(tp->gcode_lastPos),
                              emc_cart_to_sc_pnt(center),
                              emc_pose_to_sc_pnt(end),&b.pnt_w);

    b.pnt_e=emc_pose_to_sc_pnt(end);

    b.gcode_line_nr=tp->gcode_upcoming_line_nr;

    b.vo=0;
    b.vm=vel;
    b.ve=0;

    b.path_lenght=arc_lenght_c(b.pnt_s,b.pnt_w,b.pnt_e);

    vector_add_segment(vector_ptr,b);
    tp->vector_size=vector_size_c(vector_ptr);
    printf("vector size: %i \n",tp->vector_size);

    //! Update last pose to end of gcode block.
    tp->gcode_lastPos=end;

    tp->traject_lenght+=b.path_lenght;
    printf("lengt of this segment: %f \n",b.path_lenght);
    printf("traject lenght now: %f \n",tp->traject_lenght);

    tp->vector_current_exec=0;
    tp->segment_progress=0;
    tp->cur_pos=0;
    tp->tar_pos=0;

    return 0;
}

void tpToggleDIOs(TC_STRUCT * const tc) {

}

struct state_tag_t tpGetExecTag(TP_STRUCT * const tp)
{
    if (0 == tp) {
        struct state_tag_t empty = {0};
        return empty;
    }
    return tp->execTag;
}

//! This function is responsible for long startup delay if return=1.
int tcqFull(TC_QUEUE_STRUCT const * const tcq)
{
    return 0;
}

//! A Inline functinn is compiled in between the upper-level function. So
//! its not called every time, but compiled inbetween. This makes it faster.
inline void update_gui(TP_STRUCT * const tp){

    if(tp->vector_size>0){

        int id=tp->vector_current_exec;

        if(vector_at_id(vector_ptr,id)==sc_line){
            interpolate_line_c(vector_at(vector_ptr,id).pnt_s,
                               vector_at(vector_ptr,id).pnt_e,
                               tp->segment_progress,
                               &xyz);
        }
        if(vector_at_id(vector_ptr,id)==sc_arc){
            interpolate_arc_c(vector_at(vector_ptr,id).pnt_s,
                              vector_at(vector_ptr,id).pnt_w,
                              vector_at(vector_ptr,id).pnt_e,
                              tp->segment_progress,
                              &xyz);
        }
        tp->currentPos.tran.x=xyz.x;
        tp->currentPos.tran.y=xyz.y;
        tp->currentPos.tran.z=xyz.z;

        interpolate_dir_c(vector_at(vector_ptr,id).dir_s,
                          vector_at(vector_ptr,id).dir_e,
                          tp->segment_progress,
                          &abc);
        tp->currentPos.a=abc.a;
        tp->currentPos.b=abc.b;
        tp->currentPos.c=abc.c;

        interpolate_ext_c(vector_at(vector_ptr,id).ext_s,
                          vector_at(vector_ptr,id).ext_e,
                          tp->segment_progress,
                          &uvw);
        tp->currentPos.u=uvw.u;
        tp->currentPos.v=uvw.v;
        tp->currentPos.w=uvw.w;

        //! Update emc with some values.
        emcmotConfig->trajCycleTime=tp->cycleTime;

        //! Dtg in this move.
        emcmotStatus->distance_to_go=tp->tar_pos-tp->cur_pos;

        //! What this part of code does is unclear for me now.
        EmcPose pose;
        pose.tran.x=vector_at(vector_ptr,id).pnt_e.x-xyz.x;
        pose.tran.y=vector_at(vector_ptr,id).pnt_e.y-xyz.y;
        pose.tran.z=vector_at(vector_ptr,id).pnt_e.z-xyz.z;
        emcmotStatus->dtg=pose;

        emcmotStatus->current_vel=tp->cur_vel;
    }
}

//! A Inline functinn is compiled in between the upper-level function. So
//! its not called every time, but compiled inbetween. This makes it faster.
inline void update_ruckig(TP_STRUCT * const tp){

    // Check the vector. Load first segment into the ruckig planner.
    if(tp->vector_size>0){

        //! Gcode exec line nr.
        //! Used by funtion tpGetExecId to set the gui's current executed gcode line.
        tp->gcode_current_executed_line_nr=vector_at(vector_ptr,tp->vector_current_exec).gcode_line_nr;
        tp->tar_pos=vector_at(vector_ptr,tp->vector_current_exec).path_lenght;


        //! Calculate ruckig's next step.

        r.curacc=tp->cur_acc;
        r.curpos=tp->cur_pos;
        r.curvel=tp->cur_vel;

        r.maxacc=tp->aMax;
        r.maxjerk=tp->max_jerk;

        double gcode_vel=vector_at(vector_ptr,tp->vector_current_exec).vm;
        if(gcode_vel>tp->vLimit){
            gcode_vel=tp->vLimit;
        }

        r.maxvel = gcode_vel;

        r.enable=1;
        r.durationdiscretizationtype=Continuous;
        r.synchronizationtype=None;

        //! MENTION: tp->cycletime is not set to 0.001, or it has a long to double conversion error.
        //! We set it fixed for now.
        r.period=0.001;
        r.tarpos=tp->tar_pos;
        r.taracc=0;
        r.tarvel=0;

        //! Change to motion reverse. Set tarpos to begin of segment.
        if(tp->reverse_run){
            r.tarpos=0;
        }

        //! When pausing, goto velocity 0. See the component motdot
        //! how a jog stop is done.
        if(tp->pausing){
            r.interfacetype=velocity;
        } else {
            r.interfacetype=position;
        }

        r=wrapper_get_pos(r);

        if(!r.error){
            tp->cur_pos=r.curpos;
            tp->cur_acc=r.curacc;
            tp->cur_vel=r.curvel;

            //! Update hal pins for monitoring by halscope.
            *tp_curvel->Pin=r.curvel;
            *tp_curacc->Pin=r.curacc;
        }

        //! printf("curvel: %f \n",tp->cur_vel);
        //! printf("curpos: %f \n",tp->cur_pos);
        //! printf("curacc: %f \n",tp->cur_acc);

        //! Ruckig error. For example error 101 is not readlly a error.
        //! We get error 101 when slow down in pause, interfacetype::velocity.
        if(r.error){
            // printf("ruckig error. %i \n",r.function_return_code);
        }

        if(tp->pausing){
            tp->segment_progress=tp->cur_pos/tp->tar_pos;
            return;
        }

        //! Ruckig is finished, motion is forward.
        if(r.finished && !tp->reverse_run){
            // printf("ruckig finished. \n");

            //! Set next gcode segment if we are not at the end yet.
            if(tp->vector_current_exec<tp->vector_size-1){

                //! Todo : check if moving forward or moving reverse.
                tp->vector_current_exec++;
                tp->cur_pos=0;
                tp->tar_pos=vector_at(vector_ptr,tp->vector_current_exec).path_lenght;
            }

            //! We are finished and completed the last gcode segment. Traject is done !
            if(tp->vector_current_exec==tp->vector_size-1){
                vector_clear(vector_ptr);
                tp->vector_size=0;

                tp->vector_current_exec=0;
                tp->segment_progress=0;
                tp->cur_pos=0;
                tp->tar_pos=0;
            }
        }

        //! Ruckig is finished, motion is forward.
        if(r.finished && tp->reverse_run){
            // printf("ruckig finished. \n");

            //! Set next gcode segment if we are not at zero.
            if(tp->vector_current_exec>0){

                //! Todo : check if moving forward or moving reverse.
                tp->vector_current_exec--;
                tp->cur_pos=vector_at(vector_ptr,tp->vector_current_exec).path_lenght;
                tp->tar_pos=0;
            }

            //! We are finished and completed the last gcode segment. Traject is done !
            if(tp->vector_current_exec==0){
                vector_clear(vector_ptr);
                tp->vector_size=0;

                tp->vector_current_exec=0;
                tp->segment_progress=0;
                tp->cur_pos=0;
                tp->tar_pos=0;
            }
        }

        //! Ruckig is not finsihed, Just for info.
        if(!r.finished){
            // printf("ruckig busy. \n");
        }

        //! Progress 0-1.
        tp->segment_progress=tp->cur_pos/tp->tar_pos;
    }
}

EXPORT_SYMBOL(tpMotFunctions);
EXPORT_SYMBOL(tpMotData);

EXPORT_SYMBOL(tpSetMaxJerk); //! Added to get it to run.
EXPORT_SYMBOL(tpAbort);
EXPORT_SYMBOL(tpActiveDepth);
EXPORT_SYMBOL(tpAddCircle);
EXPORT_SYMBOL(tpAddLine);
EXPORT_SYMBOL(tpAddRigidTap);
EXPORT_SYMBOL(tpClear);
EXPORT_SYMBOL(tpCreate);
EXPORT_SYMBOL(tpGetExecId);
EXPORT_SYMBOL(tpGetExecTag);
EXPORT_SYMBOL(tpGetMotionType);
EXPORT_SYMBOL(tpGetPos);
EXPORT_SYMBOL(tpIsDone);
EXPORT_SYMBOL(tpPause);
EXPORT_SYMBOL(tpQueueDepth);
EXPORT_SYMBOL(tpResume);
EXPORT_SYMBOL(tpRunCycle);
EXPORT_SYMBOL(tpSetAmax);
EXPORT_SYMBOL(tpSetAout);
EXPORT_SYMBOL(tpSetCycleTime);
EXPORT_SYMBOL(tpSetDout);
EXPORT_SYMBOL(tpSetId);
EXPORT_SYMBOL(tpSetPos);
EXPORT_SYMBOL(tpSetRunDir);
EXPORT_SYMBOL(tpSetSpindleSync);
EXPORT_SYMBOL(tpSetTermCond);
EXPORT_SYMBOL(tpSetVlimit);
EXPORT_SYMBOL(tpSetVmax);
EXPORT_SYMBOL(tcqFull);






































