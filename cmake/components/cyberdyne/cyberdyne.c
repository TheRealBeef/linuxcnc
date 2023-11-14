#include "rtapi.h"
#include "rtapi_ctype.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "rtapi_errno.h"
#include "rtapi_math64.h"
#include <rtapi_io.h>
#include "hal.h"
#include "ruckig_dev_format.h"
#include "ruckig_dev_interface.h"
#include "stdio.h"

/* module information */
MODULE_AUTHOR("Skynet");
MODULE_DESCRIPTION("Halmodule test");
MODULE_LICENSE("GPL");

static int comp_idx;

typedef struct {
    bool ok;
} skynet_t;
skynet_t *skynet;

typedef struct {
    hal_float_t *Pin;
} float_data_t;
float_data_t *vel, *pos, *acc, *return_code;
float_data_t *maxjerk, *maxacc, *tarpos, *tarvel, *cycletime, *tarvel, *maxvel;

//! Pins
typedef struct {
    hal_bit_t *Pin;
} bit_data_t;
bit_data_t *module;

typedef struct {
    hal_s32_t *Pin;
} s32_data_t;

typedef struct {
    hal_u32_t *Pin;
} u32_data_t;

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

static int comp_idx; /* component ID */

static void the_function();
static int setup_pins();
void set_ruckig_values();
void set_ruckig_waypoints();

struct ruckig_c_data r,s;
struct ruckig_dev_interface *ruckig_ptr;
extern ruckig_dev_interface* ruckig_init_ptr();
extern int ruckig_calculate_c(struct ruckig_c_data in, struct ruckig_c_data *out);
extern void ruckig_add_waypoint(ruckig_dev_interface *ptr, struct ruckig_c_waypoint point);
extern struct ruckig_c_waypoint ruckig_get_waypoint(ruckig_dev_interface *ptr, int index);
extern int ruckig_waypoint_vector_size(ruckig_dev_interface *ptr);

int waypoint_nr;
bool mode_auto;

int rtapi_app_main(void) {

    int r = 0;
    comp_idx = hal_init("cyberdyne");
    if(comp_idx < 0) return comp_idx;
    r = hal_export_funct("the_function", the_function, &skynet,0,0,comp_idx);

    r+=setup_pins();

    if(r) {
        hal_exit(comp_idx);
    } else {
        hal_ready(comp_idx);
    }
    printf("component %i ready.\n",comp_idx);

    ruckig_ptr=ruckig_init_ptr();

    set_ruckig_values();
    set_ruckig_waypoints();

    //! Return 0=ok.
    return r;
}

void set_ruckig_values(){

    //! Set some values.
    *maxvel->Pin=50;
    *maxjerk->Pin=1100;
    *maxacc->Pin=500;
    *tarpos->Pin=300;
    *tarvel->Pin=10;
    *cycletime->Pin=0.001;
}

void set_ruckig_waypoints(){

    struct ruckig_c_waypoint waypoint;
    waypoint.goalpos=100;
    waypoint.ve=10;
    ruckig_add_waypoint(ruckig_ptr,waypoint);

    waypoint.goalpos=200;
    waypoint.ve=25;
    ruckig_add_waypoint(ruckig_ptr,waypoint);

    waypoint.goalpos=500;
    waypoint.ve=0;
    ruckig_add_waypoint(ruckig_ptr,waypoint);

    printf("waypoint vector size: %i \n",ruckig_waypoint_vector_size(ruckig_ptr));

    waypoint_nr=0;

    //! Turn off to use halpin values, see : set_ruckig_values();
    mode_auto=1;
}

void rtapi_app_exit(void){
    ruckig_ptr=NULL;
    hal_exit(comp_idx);
}

//! Perforn's every ms.
static void the_function(){

    if(*module->Pin==1){}

    *vel->Pin=r.newvel;
    *acc->Pin=r.newacc;
    *pos->Pin=r.newpos;

    //! Everything is ok.
    *return_code->Pin=r.function_return_code;

    r.enable=1;
    r.control_interfacetype=position;
    r.durationdiscretizationtype=Continuous;
    r.synchronizationtype=None;

    r.cycletime=*cycletime->Pin;    // 0.001
    r.maxacc=*maxacc->Pin;          // 500
    r.maxjerk=*maxjerk->Pin;        // 1100
    r.maxvel=*maxvel->Pin;          // 50

    if(!mode_auto){
        r.tarpos=*tarpos->Pin;
        r.tarvel=*tarvel->Pin;
    } else {
        r.tarpos=ruckig_get_waypoint(ruckig_ptr,waypoint_nr).goalpos;
        r.tarvel=ruckig_get_waypoint(ruckig_ptr,waypoint_nr).ve;
    }

    if(ruckig_calculate_c(r,&r)){}

    if(r.function_return_code==1){

        int size=ruckig_waypoint_vector_size(ruckig_ptr);
        if(waypoint_nr<size-1){
            waypoint_nr++;
            printf("waypoint nr: %i \n",waypoint_nr);
        }
    }
}

static int setup_pins(){
    int r=0;

    module = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("cyberdyne.enable",HAL_IN,&(module->Pin),comp_idx);

    vel = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cyberdyne.vel",HAL_OUT,&(vel->Pin),comp_idx);
    
    acc = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cyberdyne.acc",HAL_OUT,&(acc->Pin),comp_idx);

    pos = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cyberdyne.pos",HAL_OUT,&(pos->Pin),comp_idx);

    return_code = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cyberdyne.code",HAL_OUT,&(return_code->Pin),comp_idx);

    tarvel = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cyberdyne.tarvel",HAL_IN,&(tarvel->Pin),comp_idx);

    maxvel = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cyberdyne.maxvel",HAL_IN,&(maxvel->Pin),comp_idx);

    maxacc = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cyberdyne.maxacc",HAL_IN,&(maxacc->Pin),comp_idx);

    maxjerk = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cyberdyne.maxjerk",HAL_IN,&(maxjerk->Pin),comp_idx);

    tarpos = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cyberdyne.tarpos",HAL_IN,&(tarpos->Pin),comp_idx);

    cycletime = (float_data_t*)hal_malloc(sizeof(float_data_t));
    r+=hal_pin_float_new("cyberdyne.cycletime",HAL_IN,&(cycletime->Pin),comp_idx);

    return r;
}







































