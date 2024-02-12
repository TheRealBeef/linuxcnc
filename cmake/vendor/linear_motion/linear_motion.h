#ifndef LINEAR_MOTION_H
#define LINEAR_MOTION_H

#ifdef __cplusplus

#include "iostream"
#include <iomanip>
#include "tuple"
#include "math.h"
#include "vector"

//! A class to calculate linear motion profiles.
//! If "ve" velocity end can not be reached, given s "displacement", a custom "ve" is given
//! as output.
class linear_motion
{
public:
    linear_motion();

    void set_debug(bool state, bool state_at_time);

    // Position mode.
    void set_curve_values(double velocity_begin,
                          double velocity_end,
                          double velocity_max,
                          double acceleration_max,
                          double displacment);

    // Position mode.
    void set_curve_values(double velocity_begin,
                          double velocity_end,
                          double velocity_max,
                          double acceleration_max,
                          double displacment_begin,
                          double displacment_end);

    double get_curve_ve();

    double get_curve_total_time();

    void get_curve_at_time(double t, double &sr, double &vr, double &ar);

    double get_s_given_so_se(double so, double se);
    double get_sr_given_so_s(double so_now, double s_now);

    void unit_test_displacement();
    void unit_test_position_mode(bool gnu_plot);
    void unit_test_velocity_mode_run_forward(bool gnu_plot);
    void unit_test_velocity_mode_run_reverse(bool gnu_plot);
    void unit_test_velocity_mode_run_stop(bool gnu_plot);

    // Velocity mode.
    void run_velocity_mode(double velocity_begin,
                           double velocity_max,
                           double displacement_begin,
                           double acceleration_max,
                           double machine_limit_max,
                           double machine_limit_min,
                           bool direction_reverse,
                           bool stop,                   // Stop=1, No stop=0.
                           double time_interval,        // Servo period.
                           double &displacement_end,
                           double &velocity_end,
                           double &acceleration_end
                           );

    void plot_gnu_graph(double interval, std::string graph_header_text);

private:
    double s=0;
    double so=0;
    double se=0;
    double t=0;
    double a=0;
    double d=0;
    double ve=0;
    double vo=0;
    double vm=0;
    bool debug=0;
    bool debug_time_request=0;
    double t1=0,t2=0,t3=0,s1=0,s2=0,s3=0;
    double ttot=0, stot=0;
    bool dir_rev=0;
    enum las { // Linear acceleration state.
        acc,
        steady,
        dcc,
        none
    };
    std::tuple<las,las,las> res; // Holds calculated curve states for las.
    void curve_flow_positive();
    void curve_flow_negative();
    void print_curve_info();
    double s_stop(double v, double a);
    double t_acc_dcc(double vo, double ve, double a);
    double s_acc_dcc(double vo, double ve, double a);
    double ve_acc_dcc_at_time(double vo, double a, double t);
    double ve_acc_given_s_vo_a(double s, double vo, double a);
    double ve_dcc_given_s_vo_a(double s, double vo, double a);
    double s_steady(double v, double t);
    double t_steady(double s, double v);
    double a_sign(double vo, double ve, double a);
    double a_given_s_vo_t(double s, double vo, double t);
    double s_given_a_vo_t(double a, double vo, double t);
};

//! Here it tells if this code is used in c, convert the class to a struct. This is handy!
#else
typedef struct linear_motion linear_motion;
#endif //! cplusplus

#endif // LINEAR_MOTION_H























