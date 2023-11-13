#include "rtapi.h"
#include "rtapi_ctype.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "rtapi_errno.h"
#include "rtapi_math64.h"
#include <rtapi_io.h>
#include "hal.h"
#include "ruckig_dev_format.h"
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
float_data_t *vel, *pos, *acc;

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

struct ruckig_c_data r,s;
extern struct ruckig_c_data ruckig_calculate_c_online(struct ruckig_c_data in);
extern struct ruckig_c_data ruckig_calculate_c_offline(struct ruckig_c_data in);

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
    return 0;
}

void rtapi_app_exit(void){
    hal_exit(comp_idx);
}

//! Perforn's every ms.
static void the_function(){

    if(*module->Pin==1){}

    r.enable=1;
    r.control_interfacetype=position;
    r.durationdiscretizationtype=Continuous;
    r.synchronizationtype=None;

    r.maxacc=500;
    r.maxjerk=1100;
    r.maxvel=50;

    r.tarpos=100;
    r.at_time=0.001;
    r.cycletime=0.001;

    s=r;
    s=ruckig_calculate_c_offline(s);
    // printf("offline vel: %f acc: %f pos: %f \n",s.curvel,s.curacc,s.curpos);

    r=ruckig_calculate_c_online(r);
    // printf("online vel: %f acc: %f pos: %f \n",r.curvel,r.curacc,r.curpos);

    *vel->Pin=r.curvel;
    *acc->Pin=r.curacc;
    *pos->Pin=r.curpos;

    if(r.curpos==r.tarpos){
        r.curpos=0;
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

    return r;
}







































