#include "tp_lines.h"

tp_lines::tp_lines()
{

}

void tp_lines::sc_interpolate_lin(sc_pnt p0, sc_pnt p1, double progress, sc_pnt &pi){
    sc_interpolate_lenght(p0.x,p1.x,progress,pi.x);
    sc_interpolate_lenght(p0.y,p1.y,progress,pi.y);
    sc_interpolate_lenght(p0.z,p1.z,progress,pi.z);
}

//! Progress 0-1.
extern "C" void interpolate_line_c(struct sc_pnt p0, struct sc_pnt p1, double progress, struct  sc_pnt *pi){
    struct sc_pnt p;
    tp_lines().sc_interpolate_lin(p0,p1,progress,p);
    *pi=p;
}


void tp_lines::sc_interpolate_dir(sc_dir d0, sc_dir d1, double progress, sc_dir &di){
    sc_interpolate_lenght(d0.a,d1.a,progress,di.a);
    sc_interpolate_lenght(d0.b,d1.b,progress,di.b);
    sc_interpolate_lenght(d0.c,d1.c,progress,di.c);
}

//! Progress 0-1.
extern "C" void interpolate_dir_c(struct sc_dir p0, struct sc_dir p1, double progress, struct sc_dir *pi){
    struct sc_dir p;
    tp_lines().sc_interpolate_dir(p0,p1,progress,p);
    *pi=p;
}

void tp_lines::sc_interpolate_ext(sc_ext e0, sc_ext e1, double progress, sc_ext &ei){
    sc_interpolate_lenght(e0.u,e1.u,progress,ei.u);
    sc_interpolate_lenght(e0.v,e1.v,progress,ei.v);
    sc_interpolate_lenght(e0.w,e1.w,progress,ei.w);
}

//! Progress 0-1.
extern "C" void interpolate_ext_c(struct sc_ext p0, struct sc_ext p1, double progress, struct sc_ext *pi){
    struct sc_ext p;
    tp_lines().sc_interpolate_ext(p0,p1,progress,p);
    *pi=p;
}

double netto_difference_of_2_values(double a, double b){

    double diff=0;
    if(a<0 && b<0){
        a=fabs(a);
        b=fabs(b);
        diff=fabs(a-b);
    }
    if(a>=0 && b>=0){
        diff=fabs(a-b);
    }
    if(a<=0 && b>=0){;
        diff=fabs(a)+b;
    }
    if(a>=0 && b<=0){
        diff=a+fabs(b);
    }
    return diff;
}

void tp_lines::sc_interpolate_lenght(double start, double end, double progress, double &li){
    if(start<end){
        li=start+(progress*netto_difference_of_2_values(start,end));
        return;
    }
    if(start>end){
        li=start-(progress*netto_difference_of_2_values(start,end));
        return;
    }
    if(start==end){
        li=end;
    }
}

double tp_lines::sc_line_lenght(sc_pnt p0, sc_pnt p1){
    return sqrt(pow(p1.x-p0.x,2)+pow(p1.y-p0.y,2)+pow(p1.z-p0.z,2));
}

extern "C" double line_lenght_c(struct sc_pnt start, struct sc_pnt end){

    double l=tp_lines().sc_line_lenght(start,end);
    if(isnanf(l)){
        return 0;
    }
    return l;
}

















