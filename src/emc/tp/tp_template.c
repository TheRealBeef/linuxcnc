
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
//! Visualise and increment active function calls.
float_data_t *tpRunCycleHal;
float_data_t *tpGetPosHal;

//! Data.
float_data_t *tpCurrentPosXHal;
float_data_t *tpCurrentPosYHal;
float_data_t *tpCurrentPosZHal;
float_data_t *vector_size;

//! Pins
typedef struct {
    hal_bit_t *Pin;
} bit_data_t;
bit_data_t *simulate_tp_move_x;
bit_data_t *simulate_tp_move_y;
bit_data_t *simulate_tp_move_z;

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
param_bit_data_t *tpDoneHal;
param_bit_data_t *reset_vector;

static int comp_idx; /* component ID */

static void the_function();
static int setup_pins();

int rtapi_app_main(void) {

    int r = 0;
    comp_idx = hal_init("tpmod_template");
    if(comp_idx < 0) return comp_idx;
    r = hal_export_funct("tpmod_template", the_function, &skynet,0,0,comp_idx);

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

    //! Parameter bit.
    //!
    //! In halshow set this pin high to load gcode line's.
    //!
    tpDoneHal = (param_bit_data_t*)hal_malloc(sizeof(param_bit_data_t));
    r+=hal_param_bit_new("tpmod_template.tpDone",HAL_RW,&(tpDoneHal->Pin),comp_idx);

    reset_vector = (param_bit_data_t*)hal_malloc(sizeof(param_bit_data_t));
    r+=hal_param_bit_new("tpmod_template.reset_vector_to_zero",HAL_RW,&(reset_vector->Pin),comp_idx);

    vector_size = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("tpmod_template.vector_size",HAL_OUT,&(vector_size->Pin),comp_idx);

    tpRunCycleHal = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("tpmod_template.tpRunCycle",HAL_OUT,&(tpRunCycleHal->Pin),comp_idx);

    tpGetPosHal = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("tpmod_template.tpGetPos",HAL_OUT,&(tpGetPosHal->Pin),comp_idx);

    tpCurrentPosXHal = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("tpmod_template.tpCurrentPosX",HAL_OUT,&(tpCurrentPosXHal->Pin),comp_idx);

    tpCurrentPosYHal = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("tpmod_template.tpCurrentPosY",HAL_OUT,&(tpCurrentPosYHal->Pin),comp_idx);

    tpCurrentPosZHal = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("tpmod_template.tpCurrentPosZ",HAL_OUT,&(tpCurrentPosZHal->Pin),comp_idx);

    simulate_tp_move_x = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("tpmod_template.simulate_tp_move_x",HAL_IN,&(simulate_tp_move_x->Pin),comp_idx);

    simulate_tp_move_y = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("tpmod_template.simulate_tp_move_y",HAL_IN,&(simulate_tp_move_y->Pin),comp_idx);

    simulate_tp_move_z = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("tpmod_template.simulate_tp_move_z",HAL_IN,&(simulate_tp_move_z->Pin),comp_idx);

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

//! Create a empty queue.
int tpInit(TP_STRUCT * const tp)
{
    printf("tpInit. \n");


    return 0;
}

//! This component should give you an insight how the trajectory planner program flow should be,
//! without freezing linuxcnc and to avoid unwanted behaviour.
//!
//! tpDone is set true or 1, when vector_size (queue) == 0.
//! This say's oke all gcode is done, we emptied the queue to zero.
//! The vector_size represent the gcode lines.

// Usage :
//
// Load axis_ini.mm with this component turned on in the [TRAJ] section.
// Home the machine, jog some around.
// Press program start, simulate hal pins to change tp position, after that, set reset_vector pin to say ok traject is finished.
// Swith to mdi mode, and simulate tp positions again, set the reset_vecor pin again, and see what happens.

int tpRunCycle(TP_STRUCT * const tp, long period)
{
    // printf("tpRunCycle. \n");

    //! Update currentpos to hal.
    *tpCurrentPosXHal->Pin=tp->currentPos.tran.x;
    *tpCurrentPosYHal->Pin=tp->currentPos.tran.y;
    *tpCurrentPosZHal->Pin=tp->currentPos.tran.z;

    //! Increment value if this function is called.
    *tpRunCycleHal->Pin+=0.001;

    //! Simulate tp move by hal pin.
    //! Do this when program runs in auto or mdi mode.
    if(*simulate_tp_move_x->Pin){
        tp->currentPos.tran.x+=0.01;
    }
    if(*simulate_tp_move_y->Pin){
        tp->currentPos.tran.y+=0.01;
    }
    if(*simulate_tp_move_z->Pin){
        tp->currentPos.tran.z+=0.01;

    }

    //! Simulate halpin=true we are done with the path.
    //! Reset path vector to zero.
    //! This then informs the tpIsDone function to be able to switch mode mdi, auto etc.
    if(reset_vector->Pin){
        printf("vector reset to size: 0");
        tp->vector_size=0;
        //! Update hal pin.
        reset_vector->Pin=tp->vector_size;
    }

    //! Update hal pin.
    *vector_size->Pin=tp->vector_size;

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

    tp->currentPos=*pos;

    printf("tpSetPos. \n");

    return 0;
}

int tpSetCurrentPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (0 == tp) {
        return -1;
    }

    printf("tpSetCurrentPos. \n");

    return 0;
}

int tpAddCurrentPos(TP_STRUCT * const tp, EmcPose const * const disp)
{
    printf("tpAddCurrentPos. \n");
}

int tpGetPos(TP_STRUCT const * const tp, EmcPose * const pos)
{
    // printf("tpGetPos. \n");
    // printf("x: %f y: %f z: %f \n",pos->tran.x,pos->tran.y,pos->tran.z);

    //! The gui toolposition tp is updated from here.
    if (0 == tp) {
        ZERO_EMC_POSE((*pos));
        return TP_ERR_FAIL;
    } else {
        *pos = tp->currentPos;
    }

    //! Increment value if this function is called.
    *tpGetPosHal->Pin+=0.001;

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
    printf("tpResume, reset abort. \n");

    tp->pausing=0;

    return 0;
}

int tpAbort(TP_STRUCT * const tp)
{
    printf("tpAbort. \n");

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

int tpSetRunDir(TP_STRUCT * const tp, tc_direction_t dir)
{
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

    tp->vector_size++;

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

    tp->vector_size++;

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

    tp->vector_size++;

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






































