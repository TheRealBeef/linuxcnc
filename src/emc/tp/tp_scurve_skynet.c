/********************************************************************
* Description: tp_scurve_skynet.c
*
*   scurve trajectory planning using ruckig.
*
* Author: Skynet Cyberdyne
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2023 All rights reserved.
*
********************************************************************/
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
#include "tp_corners.h"

#include "ruckig_format.h"

/* module information */
MODULE_AUTHOR("Skynet_Cyberdyne");
MODULE_DESCRIPTION("tpmod_scurve_skynet");
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
bit_data_t *reverse_run;

typedef struct { //! Int.
    hal_s32_t *Pin;
} s32_data_t;

typedef struct { //! Param int.
    hal_s32_t Pin;
} param_s32_data_t;
param_s32_data_t *max_look_ahead;

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
param_float_data_t *tp_progress;

typedef struct {
    hal_bit_t Pin;
} param_bit_data_t;
param_bit_data_t *clear_vec, *done;

static int comp_idx; /* component ID */

static void the_function();
static int setup_pins();

int rtapi_app_main(void) {

    int r = 0;
    comp_idx = hal_init("tpmod_scurve_skynet");
    if(comp_idx < 0) return comp_idx;
    r = hal_export_funct("tpmod_scurve_skynet", the_function, &skynet,0,0,comp_idx);

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
    r+=hal_pin_float_new("tpmod_scurve_skynet.curvel",HAL_OUT,&(tp_curvel->Pin),comp_idx);

    tp_curacc = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("tpmod_scurve_skynet.curacc",HAL_OUT,&(tp_curacc->Pin),comp_idx);

    reverse_run = (bit_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_bit_new("tpmod_scurve_skynet.reverse",HAL_IN,&(reverse_run->Pin),comp_idx);

    max_look_ahead = (param_s32_data_t*)hal_malloc(sizeof(param_s32_data_t));
    r+=hal_param_s32_new("tpmod_scurve_skynet.look_ahead",HAL_RW,&(max_look_ahead->Pin),comp_idx);

    tp_progress = (param_float_data_t*)hal_malloc(sizeof(param_float_data_t));
    r+=hal_param_float_new("tpmod_scurve_skynet.progress",HAL_RW,&(tp_progress->Pin),comp_idx);

    clear_vec = (param_bit_data_t*)hal_malloc(sizeof(param_bit_data_t));
    r+=hal_param_bit_new("tpmod_scurve_skynet.clear_vec",HAL_RW,&(clear_vec->Pin),comp_idx);

    done = (param_bit_data_t*)hal_malloc(sizeof(param_bit_data_t));
    r+=hal_param_bit_new("tpmod_scurve_skynet.done",HAL_RW,&(done->Pin),comp_idx);

    return r;
}

int min(float a, float b){

    if(a==b){
        return a;
    }

    if(a<b){
        return a;
    }

    if(b<a){
        return b;
    }

    return a;
}

int max(float a, float b){

    if(a==b){
        return a;
    }

    if(a>b){
        return a;
    }

    if(b>a){
        return b;
    }

    return a;
}

#define TOL 0.001
bool near(double a, double b, double dist){

    if(a<(b+dist) && a>(b-dist)){
        return 1;
    }
    return 0;
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
extern void vector_set_end_angle(tp_vector *ptr, int index, double angle_deg);

extern double arc_lenght_c(struct sc_pnt start, struct sc_pnt way, struct sc_pnt end);
extern double line_lenght_c(struct sc_pnt start, struct sc_pnt end);
extern void interpolate_line_c(struct sc_pnt p0, struct sc_pnt p1, double progress, struct  sc_pnt *pi);
extern void interpolate_dir_c(struct sc_dir p0, struct sc_dir p1, double progress, struct sc_dir *pi);
extern void interpolate_ext_c(struct sc_ext p0, struct sc_ext p1, double progress, struct sc_ext *pi);
extern void interpolate_arc_c(struct sc_pnt p0, struct sc_pnt p1, struct sc_pnt p2, double progress, struct sc_pnt *pi);
extern void sc_arc_get_mid_waypoint_c(struct sc_pnt start, struct sc_pnt center, struct sc_pnt end, struct sc_pnt *waypoint);
extern void vector_interpolate_traject_c(struct tp_vector *ptr, double traject_progress, double traject_lenght, double *curve_progress, int *curve_nr);

extern double line_line_angle(struct sc_pnt p0, struct sc_pnt p1, struct sc_pnt p2);
extern double line_arc_angle(struct sc_pnt p0,struct sc_pnt p1, struct sc_pnt p2, struct sc_pnt p3);
extern double arc_line_angle(struct sc_pnt p0, struct sc_pnt p1, struct sc_pnt p2, struct sc_pnt p3);
extern double arc_arc_angle(struct sc_pnt p0, struct sc_pnt p1, struct sc_pnt p2, struct sc_pnt p3, struct sc_pnt p4);
extern double segment_angle(struct tp_segment s0, struct tp_segment s1);
extern double arc_radius( struct sc_pnt arc_way, struct sc_pnt arc_center);

//! Gcode vector dynamic.
struct tp_vector *vector_ptr;

//! Ruckig scurve.
extern struct result wrapper_get_pos(struct result input);
extern double wrapper_stop_lenght(struct result input);
struct result r={};
struct result restore={};

void update_gui(TP_STRUCT * const tp);
void update_ruckig(TP_STRUCT * const tp);
void update_hal(TP_STRUCT * const tp);

void update_look_ahead(TP_STRUCT * const tp);
bool pathrules_forward_stop(int i);
bool pathrules_reverse_stop(int i);

struct sc_pnt xyz;
struct sc_dir abc;
struct sc_ext uvw;

//! Create a empty queue.
int tpInit(TP_STRUCT * const tp)
{
    printf("tpInit. \n");

    return 0;
}

//! When program run's this is the cycle function.
int tpRunCycle(TP_STRUCT * const tp, long period)
{
    //! Update hal pin's once a cycle.
    update_hal(tp);

    //! Goto the target position.
    update_ruckig(tp);

    //! Get the netto look ahead segments for the trajectory based on
    //! the current executed segment : tp->vector_current_exec.
    update_look_ahead(tp);

    //! Interpolate tp position given a 0-1 trajectory progress.
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

    if(max_look_ahead->Pin==0){
        max_look_ahead->Pin=10;
        printf("tpCreate, set look_ahead to : %i \n",max_look_ahead->Pin);
    }

    printf("tpCreate. set tp->queuesize to: %i \n", tp->queueSize);

    return 0;
}

//! Set the max jerk for the scurve.
int tpSetMaxJerk(TP_STRUCT * const tp, double max_jerk)
{
    if (!tp || max_jerk <= 0.0) {
        return -1;
    }

    tp->max_jerk=max_jerk;

    printf("tpSetMaxJerk to: %f \n",max_jerk);
    return 0;
}

//! When you close lcnc.
int tpClear(TP_STRUCT * const tp)
{
    printf("tpClear. \n");

    vector_clear(vector_ptr);
    vector_ptr=NULL;

    return 0;
}

//! Set the cycletime. Not used.
int tpSetCycleTime(TP_STRUCT * const tp, double secs)
{
    if (!tp || secs <= 0.0) {
        return -1;
    }

    tp->cycleTime = secs;
    printf("tpSetCycleTime to: %f \n",tp->cycleTime);
    return 0;
}

//! Set the maximum velocity's. Not used.
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

//! Set the max velocity for the program.
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

//! Set gcode line nr for upcoming new line, arc.
int tpSetId(TP_STRUCT * const tp, int id)
{
    if (!tp) {
        return -1;
    }

    //! printf("tpSetId. \n");
    tp->gcode_upcoming_line_nr=id;

    return 0;
}

//! This is the executed gcode line nr. The gui's gcode preview
//! uses this to set the line.
int tpGetExecId(TP_STRUCT * const tp)
{
    if (!tp) {
        return -1;
    }

    //! printf("tpGetExecId. \n");

    return tp->gcode_current_executed_line_nr;
}

//! Not used.
int tpSetTermCond(TP_STRUCT * const tp, int cond, double tolerance)
{
    return 0;
}


//! Used to tell the tp the initial position.
//! It sets the current position AND the goal position to be the same.  Used
//! only at TP initialization and when switching modes.
int tpSetPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (!tp) {
        return -1;
    }

    // printf("tpSetPos x: %f y: %f z: %f \n",pos->tran.x,pos->tran.y,pos->tran.z);

    tp->currentPos=*pos;

    // printf("vector size: %i \n",vector_size_c(vector_ptr));
    // printf("tpCurrentPos x: %f y: %f z: %f \n",tp->currentPos.tran.x,tp->currentPos.tran.y,tp->currentPos.tran.z);

    return 0;
}

//! The gui's toolposition tp is updated from here.
int tpGetPos(TP_STRUCT const * const tp, EmcPose * const pos)
{
    if (!tp) {
        return -1;
    }

    *pos = tp->currentPos;
    // printf("tpGetPos x: %f y: %f z: %f \n",pos->tran.x,pos->tran.y,pos->tran.z);
    return 0;
}

//! Not used.
int tpErrorCheck(TP_STRUCT const * const tp) {

    if (!tp) {
        return -1;
    }

    return 0;
}

//! Not used.
int tpSetSpindleSync(TP_STRUCT * const tp, int spindle, double sync, int mode) {

    if (!tp) {
        return -1;
    }

    return 0;
}

//! Set pause.
int tpPause(TP_STRUCT * const tp)
{
    // printf("tpPause. \n");

    tp->pausing=1;

    return 0;
}

//! Set Pause resume.
int tpResume(TP_STRUCT * const tp)
{
    // printf("tpResume. \n");

    tp->pausing=0;

    return 0;
}

//! Set abort.
int tpAbort(TP_STRUCT * const tp)
{
    // printf("tpAbort. \n");

    vector_clear(vector_ptr);
    tp->vector_size=0;

    return 0;
}

//! Not used.
int tpGetMotionType(TP_STRUCT * const tp)
{
    return tp->motionType;
}

//! To tell the interpreter (gcode reader) we are ready
//! with the path.
int tpIsDone(TP_STRUCT * const tp)
{
    if(tp->vector_size==0){
        tp->vector_current_exec=0;
        tp->cur_pos=0;
        tp->tar_pos=0;
        tp->traject_lenght=0;
        tp->traject_progress=0;
        return 1;
    }
    return 0;
}

//! Not used.
int tpQueueDepth(TP_STRUCT * const tp)
{
    if (!tp) {
        return -1;
    }

    return 0;
}

//! Not used.
int tpActiveDepth(TP_STRUCT * const tp)
{
    if (!tp) {
        return -1;
    }

    return 0;
}

//! Not used.
int tpSetAout(TP_STRUCT * const tp, unsigned char index, double start, double end) {

    if (!tp) {
        return -1;
    }

    return 0;
}

//! Not used.
int tpSetDout(TP_STRUCT * const tp, int index, unsigned char start, unsigned char end) {

    if (!tp) {
        return -1;
    }

    return 0;
}

//! Set the motion forward or reverse.
//! This is now done by set the hal pin.
int tpSetRunDir(TP_STRUCT * const tp, tc_direction_t dir){

    if (!tp) {
        return -1;
    }

    // printf("tpSetRunDir, motion reverse : %i \n",dir);

    tp->reverse_run=dir;

    return 0;
}

//! Not used.
int tpAddRigidTap(TP_STRUCT * const tp,
                  EmcPose end,
                  double vel,
                  double ini_maxvel,
                  double acc,
                  unsigned char enables,
                  double scale,
                  struct state_tag_t tag) {

    if (!tp) {
        return -1;
    }

    // printf("tpAddRigidTap \n");

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
              struct state_tag_t tag){

    if (!tp) {
        return -1;
    }

    // printf("tpAddLine \n");

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
    b.angle_begin=0;
    b.angle_end=0;

    b.pnt_e=emc_pose_to_sc_pnt(end);

    b.dir_s=emc_pose_to_sc_dir(tp->gcode_lastPos);
    b.dir_e=emc_pose_to_sc_dir(end);

    b.ext_s=emc_pose_to_sc_ext(tp->gcode_lastPos);
    b.ext_e=emc_pose_to_sc_ext(end);

    b.gcode_line_nr=tp->gcode_upcoming_line_nr;

    b.path_lenght=line_lenght_c(b.pnt_s,b.pnt_e);

    b.vo=0;
    b.vm=vel;
    b.ve=0;

    b.radius=0;

    //! Calculate previous segment to current segment path transition corners in degrees.
    if(vector_size_c(vector_ptr)>0){
        struct tp_segment previous=vector_at(vector_ptr,vector_size_c(vector_ptr)-1);
        double angle_deg=segment_angle(previous,b);

        b.angle_begin=angle_deg;
        vector_set_end_angle(vector_ptr,vector_size_c(vector_ptr)-1,angle_deg);
    }

    vector_add_segment(vector_ptr,b);
    tp->vector_size=vector_size_c(vector_ptr);
    // printf("vector size: %i \n",tp->vector_size);

    //! Update last pose to end of gcode block.
    tp->gcode_lastPos=end;

    tp->traject_lenght+=b.path_lenght;
    // printf("lengt of this segment: %f \n",b.path_lenght);
    // printf("traject lenght now: %f \n",tp->traject_lenght);

    //! Clear.
    tp->vector_current_exec=0;
    tp->segment_progress=0;
    tp->traject_progress=0;
    tp->cur_pos=0;
    tp->tar_pos=tp->traject_lenght;

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
                struct state_tag_t tag){

    if (!tp) {
        return -1;
    }

    // printf("tpAddCircle. \n");

    if(tp->vector_size==0){
        tp->gcode_lastPos=tp->currentPos;
    }

    struct tp_segment b;
    b.primitive_id=sc_arc;
    b.type=canon_motion_type;
    b.pnt_s=emc_pose_to_sc_pnt(tp->gcode_lastPos);
    b.pnt_e= emc_pose_to_sc_pnt(end);

    b.dir_s=emc_pose_to_sc_dir(tp->gcode_lastPos);
    b.dir_e=emc_pose_to_sc_dir(end);

    b.ext_s=emc_pose_to_sc_ext(tp->gcode_lastPos);
    b.ext_e=emc_pose_to_sc_ext(end);

    b.pnt_c=emc_cart_to_sc_pnt(center);

    //! Create a 3d arc using waypoint technique.
    sc_arc_get_mid_waypoint_c(emc_pose_to_sc_pnt(tp->gcode_lastPos),
                              b.pnt_c,
                              b.pnt_e,
                              &b.pnt_w);

    b.angle_begin=0;
    b.angle_end=0;

    b.gcode_line_nr=tp->gcode_upcoming_line_nr;

    b.vo=0;
    b.vm=vel;
    b.ve=0;

    b.path_lenght=arc_lenght_c(b.pnt_s,b.pnt_w,b.pnt_e);

    //! Calculate the arc radius, we can use this for look ahead of tiny arc's.
    b.radius=arc_radius(b.pnt_w,b.pnt_c);

    //! Calculate previous segment to current segment path transition corners in degrees.
    if(vector_size_c(vector_ptr)>0){
        struct tp_segment previous=vector_at(vector_ptr,vector_size_c(vector_ptr)-1);
        double angle_deg=segment_angle(previous,b);

        b.angle_begin=angle_deg;
        vector_set_end_angle(vector_ptr,vector_size_c(vector_ptr)-1,angle_deg);
    }

    vector_add_segment(vector_ptr,b);
    tp->vector_size=vector_size_c(vector_ptr);
    // printf("vector size: %i \n",tp->vector_size);

    //! Update last pose to end of gcode block.
    tp->gcode_lastPos=end;

    tp->traject_lenght+=b.path_lenght;
    // printf("lengt of this segment: %f \n",b.path_lenght);
    // printf("traject lenght now: %f \n",tp->traject_lenght);

    tp->vector_current_exec=0;
    tp->segment_progress=0;
    tp->traject_progress=0;
    tp->cur_pos=0;
    tp->tar_pos=tp->traject_lenght;

    return 0;
}

//! Not used.
void tpToggleDIOs(TC_STRUCT * const tc) {

}

//! Not used.
struct state_tag_t tpGetExecTag(TP_STRUCT * const tp)
{
    if (!tp) {
        struct state_tag_t empty = {0};
        return empty;
    }
    return tp->execTag;
}

//! Not used.
int tcqFull(TC_QUEUE_STRUCT const * const tcq)
{
    return 0;
}

//! A Inline functinn is compiled in between the upper-level function. So
//! its not called every time, but compiled inbetween. This makes it faster.
inline void update_gui(TP_STRUCT * const tp){

    if(tp->vector_size>0){

        //! Given a 0-1 trajectory progress will give the tp point.
        vector_interpolate_traject_c(vector_ptr,
                                     tp->traject_progress,
                                     tp->traject_lenght,
                                     &tp->segment_progress,
                                     &tp->vector_current_exec);

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

        //! To prevent motion reverse from halting at segment start position.
        if(tp->reverse_run && near(tp->cur_pos,tp->tar_pos,TOL)){
            // if(tp->reverse_run && tp->cur_pos<tp->tar_pos+0.001 && tp->cur_pos>tp->tar_pos-0.001){
            // printf("in motion reverse, at segment start. \n");
            tp->tar_pos=0;
        }

        // printf("segment nr: %i",tp->vector_current_exec);
        // printf(" segment progress: %f \n",tp->segment_progress);

        //! Gcode exec line nr.
        //! Used by funtion tpGetExecId to set the gui's current executed gcode line.
        tp->gcode_current_executed_line_nr=vector_at(vector_ptr,tp->vector_current_exec).gcode_line_nr;

        r.curacc=tp->cur_acc;
        r.curpos=tp->cur_pos;
        r.curvel=tp->cur_vel;

        r.maxacc=tp->aMax;
        r.maxjerk=tp->max_jerk;

        // printf("rapid overide: %f \n",emcmotStatus->rapid_scale);     //! 1.0 is 100%
        // printf("feed overide: %f \n",emcmotStatus->net_feed_scale);   //! 1.0 is 100%

        double vm=vector_at(vector_ptr,tp->vector_current_exec).vm;
        if(vm>tp->vLimit){
            vm=tp->vLimit;
        }

        if(vector_at(vector_ptr,tp->vector_current_exec).type==1){ //! G0
            vm*=emcmotStatus->rapid_scale;
        } else { //! It's a G1,G2,G3.
            vm*=emcmotStatus->net_feed_scale;
        }

        r.maxvel = vm;
        if(r.maxvel==0){ //! Ruckig's maxvel may not be zero. Invalid.
            r.maxvel=0.01;
        }

        r.enable=1;
        r.durationdiscretizationtype=Continuous;
        r.synchronizationtype=None;

        //! MENTION: tp->cycletime is not set to 0.001, or it has a long to double conversion error.
        //! We set it fixed for now.
        r.period=0.001;
        r.tarpos=tp->tar_pos;

        r.taracc=0;
        r.tarvel=0;

        //! When pausing, goto velocity 0. See the component motdot
        //! how a jog stop is done.
        if(tp->pausing || vm==0 ){
            r.interfacetype=velocity;
        } else {
            r.interfacetype=position;
        }

        restore=r;
        r=wrapper_get_pos(r);

        if(r.function_return_code==Working){
            tp->cur_pos=r.curpos;
            tp->cur_acc=r.curacc;
            tp->cur_vel=r.curvel;

            if(isnanf(tp->cur_pos)){
                r=restore;
            }

            //! Update hal pins for monitoring by halscope.
            *tp_curvel->Pin=r.curvel;
            *tp_curacc->Pin=r.curacc;

            tp->traject_progress=tp->cur_pos/tp->traject_lenght;
        }

        if(r.function_return_code==Finished && !tp->pausing){

            // printf("curpos: %f trajectlenght: %f \n",tp->cur_pos,tp->traject_lenght);

            tp->tar_pos=tp->traject_lenght;

            //! End of traject.
            if(tp->cur_pos>tp->traject_lenght-0.001){
                tp->vector_size=0;
                vector_clear(vector_ptr);
            }
        }

        if(r.function_return_code<0){
            // printf("ruckig error code %i \n",r.function_return_code);
        }

        // printf("ruckig code %i \n",r.function_return_code);
    }
}

//! A Inline functinn is compiled in between the upper-level function. So
//! its not called every time, but compiled inbetween. This makes it faster.
inline void update_hal(TP_STRUCT * const tp){
    //! For info, a pin uses a * before, a parameter not.
    tp->reverse_run=*reverse_run->Pin;
    tp->max_look_ahead=max_look_ahead->Pin;

    //! Force clear.
    if(clear_vec->Pin){
        vector_clear(vector_ptr);
        tp->vector_size=0;
        //! Reset pin directly.
        clear_vec->Pin=0;
    }
}

//! A Inline functinn is compiled in between the upper-level function. So
//! its not called every time, but compiled inbetween. This makes it faster.
//!
//! This function calculates how much gcode segments can be optimized moving forward and
//! or moving backwards.
inline void update_look_ahead(TP_STRUCT * const tp){

    //! If gcode vector is empty, skip.
    //! If ruckig is state::working, go on.
    if(tp->vector_size>0 && r.function_return_code==0){

        int count=0;

        if(!tp->reverse_run){
            //! Motion forward. Look forward, ahead.
            //! min is calculating the minimal of 2 input values.
            for(int i=tp->vector_current_exec; i< min(vector_size_c(vector_ptr),tp->vector_current_exec+tp->max_look_ahead); i++){

                if(pathrules_forward_stop(i)){
                    break;
                }

                count++;
                // printf("look ahead forward, checking vector segment : %i \n",i);
            }
            // printf("\n");

        }

        if(tp->reverse_run){
            //! Motion reverse, look back.
            //! max is calculating the maximum of 2 input values.
            for(int i=tp->vector_current_exec; i> max(0,tp->vector_current_exec-tp->max_look_ahead); i--){

                if(pathrules_reverse_stop(i)){
                    break;
                }

                count++;
                // printf("look ahead backward, checking vector segment : %i \n",i);

            }
            // printf("\n");
        }

        double lbegin=0, lend=0;
        //! Calculate traject positions begin & end for current segment.
        for(int i=0; i<tp->vector_current_exec; i++){
            lbegin+=vector_at(vector_ptr,i).path_lenght;
        }

        lend+=lbegin;
        lend+=vector_at(vector_ptr,tp->vector_current_exec).path_lenght;

        // printf("traject lenght to current segment start: %f end: %f \n",lbegin,lend);

        if(!tp->reverse_run){
            //! Add look_ahead distance.
            for(int i=min(tp->vector_current_exec+1,tp->vector_size); i<min(tp->vector_current_exec+1+count,tp->vector_size); i++){
                lend+=vector_at(vector_ptr,i).path_lenght;
            }

            // printf("look ahead tarpos: %f \n",lend);
            tp->tar_pos=lend;
        }

        if(tp->reverse_run){
            //! Add look_back distance.
            for(int i=max(0,tp->vector_current_exec-1); i>max(0,tp->vector_current_exec-1-count); i--){
                lbegin-=vector_at(vector_ptr,i).path_lenght;
            }

            // printf("look back tarpos: %f \n",lbegin);
            tp->tar_pos=lbegin;
        }

        // printf("look ahead: %i current_pos: %f \n",count,tp->cur_pos);
        // printf("calculated tarpos: %f \n",tp->tar_pos);
    }
}

//! Add your pathrules to stop motion at a segment nr here.
//! These pathrules apply for forward motion.
//! Looking forward the path, i=tp->max_look_ahead.
//! return 1 = stop.
inline bool pathrules_forward_stop(int i){

    //! Is next segment colinair?
    //! 180 degrees is 3d coliniar.
    if(vector_at(vector_ptr,i).angle_end<170){
        return 1;
    }

    //! When segment is a G0 rapid, stop optimizing.
    if(vector_at(vector_ptr,i).type==1){
        return 1;
    }

    //! A colineair arc, how about to stop if arc has tiny radius, and the arc pathlenght is tiny.
    //! You can even activate this function by a hal pin if you want to.
    // if(vector_at(vector_ptr,i).primitive_id==sc_arc && vector_at(vector_ptr,i).radius<5 && vector_at(vector_ptr,i).path_lenght<5){
    //   return 1;
    //}

    // You can add extra pathrules here for forward motion ..

    return 0;
}

//! Add your pathrules to stop motion at a segment nr here.
//! These pathrules apply for reverse motion.
//! Looking backwards the path, i=tp->max_look_ahead.
//! return 1 = stop.
inline bool pathrules_reverse_stop(int i){

    //! When segment is a G0 rapid, stop optimizing.
    if(vector_at(vector_ptr,i).type==1){
        return 1;
    }

    //! Is next segment colinair?
    //! Pointing at angle begin!
    if(vector_at(vector_ptr,i).angle_begin<170){
        return 1;
    }

    //! A colineair arc, how about to stop if arc has tiny radius, and the arc pathlenght is tiny.
    //! You can even activate this function by a hal pin if you want to.
    // if(vector_at(vector_ptr,i).primitive_id==sc_arc && vector_at(vector_ptr,i).radius<5 && vector_at(vector_ptr,i).path_lenght<5){
    //   return 1;
    // }

    // You can add extra pathrules here for backward motion ..

    return 0;
}

EXPORT_SYMBOL(tpMotFunctions);
EXPORT_SYMBOL(tpMotData);

EXPORT_SYMBOL(tpSetMaxJerk);
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






































