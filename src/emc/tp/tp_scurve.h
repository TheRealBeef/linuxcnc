﻿#ifndef TP_SCURVE_H
#define TP_SCURVE_H

#include "emcpose.h"
#include "stdbool.h"
#include "math.h"

struct sc_pnt {
    double x, y, z;
};

struct sc_dir {
    double a, b, c;
};

struct sc_ext {
    double u, v, w;
};

enum sc_primitive_id {
    sc_line,
    sc_arc,
};

//! Inherent numbers to canon motion type.
enum sc_motion_type {
    sc_rapid=1,
    sc_linear=2,
    sc_circle=3,
};

//float line_lenght(struct sc_pnt p0, struct sc_pnt p1){
//   return sqrt(pow(p1.x-p0.x,2)+pow(p1.y-p0.y,2)+pow(p1.z-p0.z,2));
//}

//! One gcode line, cq segment.
//! Can be expanded to hold digital io values.
struct tp_segment {

    enum sc_primitive_id primitive_id;
    enum sc_motion_type type;

    int gcode_line_nr;

    struct sc_pnt pnt_s, pnt_e, pnt_w, pnt_c;
    struct sc_dir dir_s, dir_e;
    struct sc_ext ext_s, ext_e;

    //! The look ahead angle to next primitive,
    //! to calculate acceptable end velocity.
    double angle_end_deg;

    //! If arc's velmax is reduced by gforce impact value, this is maxvel.
    //! Otherwise the velmax is set to program velmax.
    double vo;
    double vm;
    double ve;

    //! Store the lenght for the scurve planner.
     double path_lenght;
};

#endif

















