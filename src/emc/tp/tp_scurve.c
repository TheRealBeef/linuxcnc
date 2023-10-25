
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
#include "tp.h"
#include "tp_debug.h"
#include "math.h"

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
float_data_t *queuedepth;

//! Pins
typedef struct {
    hal_bit_t *Pin;
} bit_data_t;
bit_data_t
*module;

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
port_data_t *port;

//! Params
typedef struct {
    hal_float_t Pin;
} param_float_data_t;

typedef struct {
    hal_bit_t Pin;
} param_bit_data_t;
param_bit_data_t
*done;

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
    *module->Pin=1;
}

static int setup_pins(){
    int r=0;
    module = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("tp_mod_scurve.module",HAL_OUT,&(module->Pin),comp_idx);

    done = (param_bit_data_t*)hal_malloc(sizeof(param_bit_data_t));
    r+=hal_param_bit_new("tp_mod_scurve.remove_segment",HAL_RW,&(done->Pin),comp_idx);




    queuedepth = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("tp_mod_scurve.queuedepth",HAL_OUT,&(queuedepth->Pin),comp_idx);

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



/* space for trajectory planner queues, plus 10 more for safety */
/*! \todo FIXME-- default is used; dynamic is not honored */
TC_STRUCT queueTcSpace[DEFAULT_TC_QUEUE_SIZE + 10];

/**
 * Add a newly created motion segment to the tp queue.
 * Returns an error code if the queue operation fails, otherwise adds a new
 * segment to the queue and updates the end point of the trajectory planner.
 */
STATIC inline int tpAddSegmentToQueue(TP_STRUCT * const tp, TC_STRUCT * const tc, int inc_id) {

    tc->id = tp->nextId;
    if (tcqPut(&tp->queue, tc) == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tcqPut failed.\n");
        return TP_ERR_FAIL;
    }
    if (inc_id) {
        tp->nextId++;
    }

    // Store end of current move as new final goal of TP
    // KLUDGE: endpoint is garbage for rigid tap since it's supposed to retract past the start point.
    if (tc->motion_type != TC_RIGIDTAP) {
        tcGetEndpoint(tc, &tp->goalPos);
    }
    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);
    //Fixing issue with duplicate id's?
    tp_debug_print("Adding TC id %d of type %d, total length %0.08f\n",tc->id,tc->motion_type,tc->target);

    return TP_ERR_OK;
}

STATIC int tpGetMachineAccelBounds(PmCartesian  * const acc_bound) {
    if (!acc_bound) {
        return TP_ERR_FAIL;
    }

    acc_bound->x = _axis_get_acc_limit(0); //0==>x
    acc_bound->y = _axis_get_acc_limit(1); //1==>y
    acc_bound->z = _axis_get_acc_limit(2); //2==>z
    return TP_ERR_OK;
}

STATIC int tpGetMachineActiveLimit(double * const act_limit, PmCartesian const * const bounds) {
    if (!act_limit) {
        return TP_ERR_FAIL;
    }
    //Start with max accel value
    *act_limit = fmax(fmax(bounds->x,bounds->y),bounds->z);

    // Compare only with active axes
    if (bounds->x > 0) {
        *act_limit = fmin(*act_limit, bounds->x);
    }
    if (bounds->y > 0) {
        *act_limit = fmin(*act_limit, bounds->y);
    }
    if (bounds->z > 0) {
        *act_limit = fmin(*act_limit, bounds->z);
    }
    tp_debug_print(" arc blending a_max=%f\n", *act_limit);
    return TP_ERR_OK;
}

STATIC int tpGetMachineVelBounds(PmCartesian  * const vel_bound) {
    if (!vel_bound) {
        return TP_ERR_FAIL;
    }

    vel_bound->x = _axis_get_vel_limit(0); //0==>x
    vel_bound->y = _axis_get_vel_limit(1); //1==>y
    vel_bound->z = _axis_get_vel_limit(2); //2==>z
    return TP_ERR_OK;
}

/**
 * Clears any potential DIO toggles and anychanged.
 * If any DIOs need to be changed: dios[i] = 1, DIO needs to get turned on, -1
 * = off
 */
int tpClearDIOs(TP_STRUCT * const tp) {

    printf("tpClearDIOs \n");

    //XXX: All IO's will be flushed on next synced aio/dio! Is it ok?
    int i;
    tp->syncdio.anychanged = 0;
    tp->syncdio.dio_mask = 0;
    tp->syncdio.aio_mask = 0;
    for (i = 0; i < emcmotConfig->numDIO; i++) {
        tp->syncdio.dios[i] = 0;
    }
    for (i = 0; i < emcmotConfig->numAIO; i++) {
        tp->syncdio.aios[i] = 0;
    }

    return TP_ERR_OK;
}

/**
 *    "Soft initialize" the trajectory planner tp.
 *    This is a "soft" initialization in that TP_STRUCT configuration
 *    parameters (cycleTime, vMax, and aMax) are left alone, but the queue is
 *    cleared, and the flags are set to an empty, ready queue. The currentPos
 *    is left alone, and goalPos is set to this position.  This function is
 *    intended to put the motion queue in the state it would be if all queued
 *    motions finished at the current position.
 */
int tpClear(TP_STRUCT * const tp)
{
    printf("tpClear \n");

    tcqInit(&tp->queue);
    tp->queueSize = 0;
    tp->goalPos = tp->currentPos;
    // Clear out status ID's
    tp->nextId = 0;
    tp->execId = 0;
    struct state_tag_t tag = {0};
    tp->execTag = tag;
    tp->motionType = 0;
    tp->termCond = TC_TERM_COND_PARABOLIC;
    tp->tolerance = 0.0;
    tp->done = 1;
    tp->depth = tp->activeDepth = 0;
    tp->aborting = 0;
    tp->pausing = 0;
    tp->reverse_run = 0;
    tp->synchronized = 0;
    tp->uu_per_rev = 0.0;
    emcmotStatus->current_vel = 0.0;
    emcmotStatus->requested_vel = 0.0;
    emcmotStatus->distance_to_go = 0.0;
    ZERO_EMC_POSE(emcmotStatus->dtg);

    // equivalent to: SET_MOTION_INPOS_FLAG(1):
    emcmotStatus->motionFlag |= EMCMOT_MOTION_INPOS_BIT;

    return tpClearDIOs(tp);
}

/**
 * Fully initialize the tp structure.
 * Sets tp configuration to default values and calls tpClear to create a fresh,
 * empty queue.
 */
int tpInit(TP_STRUCT * const tp)
{
    printf("tpInit \n");

    tp->cycleTime = 0.0;
    //Velocity limits
    tp->vLimit = 0.0;
    tp->ini_maxvel = 0.0;
    tp->max_jerk = 0.0;
    tp->cur_acc = 0.0;

    //Accelerations
    tp->aLimit = 0.0;
    PmCartesian acc_bound;
    //FIXME this acceleration bound isn't valid (nor is it used)
    if (emcmotStatus == 0) {
        rtapi_print("!!!tpInit: NULL emcmotStatus, bye\n\n");
        return -1;
    }
    tpGetMachineAccelBounds(&acc_bound);
    tpGetMachineActiveLimit(&tp->aMax, &acc_bound);
    //Angular limits
    tp->wMax = 0.0;
    tp->wDotMax = 0.0;

    tp->spindle.offset = 0.0;
    tp->spindle.revs = 0.0;
    tp->spindle.waiting_for_index = MOTION_INVALID_ID;
    tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;

    tp->reverse_run = TC_DIR_FORWARD;

    ZERO_EMC_POSE(tp->currentPos);

    PmCartesian vel_bound;
    tpGetMachineVelBounds(&vel_bound);
    tpGetMachineActiveLimit(&tp->vMax, &vel_bound);

    return tpClear(tp);
}

int tpCreate(TP_STRUCT * const tp, int _queueSize,int id){

    printf("tpCreate \n");

    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (_queueSize <= 0) {
        tp->queueSize = TP_DEFAULT_QUEUE_SIZE;
    } else {
        tp->queueSize = _queueSize;
    }
    TC_STRUCT * const tcSpace = queueTcSpace;

    /* create the queue */
    if (-1 == tcqCreate(&tp->queue, tp->queueSize, tcSpace)) {
        return TP_ERR_FAIL;
    }

    /* init the rest of our data */
    return tpInit(tp);
}

int tpRunCycle(TP_STRUCT * const tp, long period){

    //! Set halpin with halshow to see how the interpreter gives new lines & arcs.
    //! When done is true, the intepreter gives new meat if there is.
    //! This can be one segment, but can also be 100+ segments.
    if(done->Pin==true){

        tp->goalPos = tp->currentPos;
        tp->done = 1;
        tp->depth = tp->activeDepth = 0;
        tp->aborting = 0;
        tp->execId = 0;
        tp->motionType = 0;
        printf("done. \n");

        //! All segments are executed, empty queue. Reset queue depth to 0.
        tcqInit(&tp->queue);

        done->Pin=false;
    }

    return 0;
}

int tpSetMaxJerk(TP_STRUCT * const tp, double max_jerk)
{
    printf("tpSetMaxJerk \n");
    return 0;
}

int tpSetCycleTime(TP_STRUCT * const tp, double secs)
{
    printf("tpSetCycleTime \n");
    return 0;
}

int tpSetVmax(TP_STRUCT * const tp, double vMax, double ini_maxvel)
{
    printf("tpSetVmax \n");
    return 0;
}

int tpSetVlimit(TP_STRUCT * const tp, double vLimit)
{
    printf("tpSetVlimit \n");
    return 0;
}

int tpSetAmax(TP_STRUCT * const tp, double aMax)
{
    printf("tpSetAmax \n");
    return 0;
}

int tpSetId(TP_STRUCT * const tp, int id)
{
    printf("tpSetId \n");
    return 0;
}

int tpGetExecId(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    // printf("tpGetExecId %i \n",tp->execId);
    return tp->execId;
}

/**
 * Sets the termination condition for all subsequent queued moves.
 * If cond is TC_TERM_COND_STOP, motion comes to a stop before a subsequent move
 * begins. If cond is TC_TERM_COND_PARABOLIC, the following move is begun when the
 * current move slows below a calculated blend velocity.
 */
int tpSetTermCond(TP_STRUCT * const tp, int cond, double tolerance)
{
    if (!tp) {
        return TP_ERR_FAIL;
    }

    switch (cond) {
    //Purposeful waterfall for now
    case TC_TERM_COND_PARABOLIC:
    case TC_TERM_COND_TANGENT:
    case TC_TERM_COND_EXACT:
    case TC_TERM_COND_STOP:
        tp->termCond = cond;
        tp->tolerance = tolerance;
        break;
    default:
        //Invalid condition
        return  -1;
    }

    return TP_ERR_OK;
}

/**
 * Used to tell the tp the initial position.
 * It sets the current position AND the goal position to be the same.  Used
 * only at TP initialization and when switching modes.
 */
int tpSetPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    int res_invalid = tpSetCurrentPos(tp, pos);
    if (res_invalid) {
        return TP_ERR_FAIL;
    }

    tp->goalPos = *pos;
    return TP_ERR_OK;
}

/**
 * Check for valid tp before queueing additional moves.
 */
int tpErrorCheck(TP_STRUCT const * const tp) {

    if (!tp) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is null\n");
        return TP_ERR_FAIL;
    }
    if (tp->aborting) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is aborting\n");
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}

int tpSetSpindleSync(TP_STRUCT * const tp, int spindle, double sync, int mode) {
    if(sync) {
        if (mode) {
            tp->synchronized = TC_SYNC_VELOCITY;
        } else {
            tp->synchronized = TC_SYNC_POSITION;
        }
        tp->uu_per_rev = sync;
        tp->spindle.spindle_num = spindle;
    } else
        tp->synchronized = 0;

    return TP_ERR_OK;
}

int tpPause(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->pausing = 1;
    return TP_ERR_OK;
}

int tpResume(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->pausing = 0;
    return TP_ERR_OK;
}

int tpAbort(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (!tp->aborting) {
        /* const to abort, signal a pause and set our abort flag */
        tpPause(tp);
        tp->aborting = 1;
    }
    return TP_ERR_OK;
}

int tpGetMotionType(TP_STRUCT * const tp)
{
    return tp->motionType;
}

int tpGetPos(TP_STRUCT const * const tp, EmcPose * const pos)
{
    if (0 == tp) {
        ZERO_EMC_POSE((*pos));
        return TP_ERR_FAIL;
    } else {
        *pos = tp->currentPos;
    }
    // printf("tpGetPos x %f %f %f \n", tp->currentPos.tran.x ,tp->currentPos.tran.y ,tp->currentPos.tran.z);
    return TP_ERR_OK;
}

int tpIsDone(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }
    // printf("tpIsDone %i \n",tp->done);
    return tp->done;
}

int tpQueueDepth(TP_STRUCT * const tp){

    if (0 == tp) {
        return TP_ERR_OK;
    }
    // printf("tpQueueDepth %i \n",tp->depth);
    *queuedepth->Pin=tp->depth;
    return tp->depth;
}

int tpActiveDepth(TP_STRUCT * const tp){

    if (0 == tp) {
        return TP_ERR_OK;
    }
    // printf("tpActiveDepth %i \n",tp->activeDepth);
    return tp->activeDepth;
}

int tpSetAout(TP_STRUCT * const tp, unsigned char index, double start, double end) {
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->syncdio.anychanged = 1; //something has changed
    tp->syncdio.aio_mask |= (1 << index);
    tp->syncdio.aios[index] = start;
    return TP_ERR_OK;
}

int tpSetDout(TP_STRUCT * const tp, int index, unsigned char start, unsigned char end) {
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->syncdio.anychanged = 1; //something has changed
    tp->syncdio.dio_mask |= (1 << index);
    if (start > 0)
        tp->syncdio.dios[index] = 1; // the end value can't be set from canon currently, and has the same value as start
    else
        tp->syncdio.dios[index] = -1;
    return TP_ERR_OK;
}

int tpSetRunDir(TP_STRUCT * const tp, tc_direction_t dir)
{
    // Can't change direction while moving
    if (tpIsMoving(tp)) {
        return TP_ERR_FAIL;
    }

    switch (dir) {
    case TC_DIR_FORWARD:
    case TC_DIR_REVERSE:
        tp->reverse_run = dir;
        return TP_ERR_OK;
    default:
        rtapi_print_msg(RTAPI_MSG_ERR,"Invalid direction flag in SetRunDir");
        return TP_ERR_FAIL;
    }
}

int tpAddRigidTap(TP_STRUCT * const tp,
                  EmcPose end,
                  double vel,
                  double ini_maxvel,
                  double acc,
                  double max_jerk,
                  unsigned char enables,
                  double scale,
                  struct state_tag_t tag) {

    //! Leave this for now.
    return 0;
}

int tpAddLine(TP_STRUCT *
              const tp,
              EmcPose end,
              int canon_motion_type,
              double vel,
              double ini_maxvel,
              double acc,
              double max_jerk,
              unsigned char enables,
              char atspeed,
              int indexer_jnum,
              struct state_tag_t tag){

    printf("tpAddLine \n");

    if (tpErrorCheck(tp) < 0) {
        return TP_ERR_FAIL;
    }

    // Initialize new tc struct for the line segment
    TC_STRUCT tc = {0};
    tcInit(&tc,
           TC_LINEAR,
           canon_motion_type,
           tp->cycleTime,
           enables,
           atspeed);
    tc.tag = tag;

    // Copy over state data from the trajectory planner
    tcSetupState(&tc, tp);

    // Copy in motion parameters
    tcSetupMotion(&tc,
                  vel,
                  ini_maxvel,
                  acc,
                  max_jerk);

    // Setup line geometry
    pmLine9Init(&tc.coords.line,
                &tp->goalPos,
                &end);
    tc.target = pmLine9Target(&tc.coords.line);
    if (tc.target < TP_POS_EPSILON) {
        rtapi_print_msg(RTAPI_MSG_DBG,"failed to create line id %d, zero-length segment\n",tp->nextId);
        return TP_ERR_ZERO_LENGTH;
    }
    tc.nominal_length = tc.target;
    tcClampVelocityByLength(&tc);

    // For linear move, set joint corresponding to a locking indexer axis
    tc.indexer_jnum = indexer_jnum;

    //TODO refactor this into its own function
    TC_STRUCT *prev_tc;
    prev_tc = tcqLast(&tp->queue);

    tcFinalizeLength(prev_tc);
    tcFlagEarlyStop(prev_tc, &tc);

    int retval = tpAddSegmentToQueue(tp, &tc, true);

    printf("depth: %i \n",tp->depth);

    return retval;
}

/**
 * Adds a circular (circle, arc, helix) move from the end of the
 * last move to this new position.
 *
 * @param end is the xyz/abc point of the destination.
 *
 * see pmCircleInit for further details on how arcs are specified. Note that
 * degenerate arcs/circles are not allowed. We are guaranteed to have a move in
 * xyz so the target is always the circle/arc/helical length.
 */
int tpAddCircle(TP_STRUCT * const tp,
                EmcPose end,
                PmCartesian center,
                PmCartesian normal,
                int turn,
                int canon_motion_type,
                double vel,
                double ini_maxvel,
                double acc,
                double max_jerk,
                unsigned char enables,
                char atspeed,
                struct state_tag_t tag)
{
    if (tpErrorCheck(tp)<0) {
        return TP_ERR_FAIL;
    }

    printf("tpAddCircle \n");

    TC_STRUCT tc = {0};

    tcInit(&tc,
           TC_CIRCULAR,
           canon_motion_type,
           tp->cycleTime,
           enables,
           atspeed);
    tc.tag = tag;
    // Setup any synced IO for this move
    // tpSetupSyncedIO(tp, &tc);

    // Copy over state data from the trajectory planner
    tcSetupState(&tc, tp);

    // Setup circle geometry
    int res_init = pmCircle9Init(&tc.coords.circle,
                                 &tp->goalPos,
                                 &end,
                                 &center,
                                 &normal,
                                 turn);

    if (res_init) return res_init;

    // Update tc target with existing circular segment
    tc.target = pmCircle9Target(&tc.coords.circle);
    if (tc.target < TP_POS_EPSILON) {
        return TP_ERR_ZERO_LENGTH;
    }
    tp_debug_print("tc.target = %f\n",tc.target);
    tc.nominal_length = tc.target;

    // Copy in motion parameters
    tcSetupMotion(&tc,
                  vel,
                  ini_maxvel,
                  acc,
                  max_jerk);

    //Reduce max velocity to match sample rate
    tcClampVelocityByLength(&tc);

    TC_STRUCT *prev_tc;
    prev_tc = tcqLast(&tp->queue);

    // handleModeChange(prev_tc, &tc);
    // if (emcmotConfig->arcBlendEnable){
    ////    tpHandleBlendArc(tp, &tc);
    //     findSpiralArcLengthFit(&tc.coords.circle.xyz, &tc.coords.circle.fit);
    // }
    tcFinalizeLength(prev_tc);
    tcFlagEarlyStop(prev_tc, &tc);

    int retval = tpAddSegmentToQueue(tp, &tc, true);

    printf("depth: %i \n",tp->depth);

    // tpRunOptimization(tp);
    return retval;
}

struct state_tag_t tpGetExecTag(TP_STRUCT * const tp)
{
    if (0 == tp) {
        struct state_tag_t empty = {0};
        return empty;
    }
    return tp->execTag;
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






































