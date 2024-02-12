#include "linear_motion.h"
#include <cassert>

linear_motion::linear_motion()
{

}

/*
 * Example how to use this c++ code in c :
 *
    // Add a copy of the exter "C" functions :
    struct linear_motion *linear_ptr;
    extern linear_motion* lm_init_ptr();
    extern void lm_set_values(linear_motion *ptr,
                           double velocity_begin,
                           double velocity_end,
                           double velocity_max,
                           double acceleration_max,
                           double displacment,
                           bool debug);
    extern double lm_get_curve_ve(linear_motion *ptr);
    extern double lm_get_curve_total_time(linear_motion *ptr);
    extern const double* lm_get_curve_at_time(linear_motion *ptr, double t);

    // Init pointer :
    linear_ptr=lm_init_ptr();

    // Use functions :
    lm_set_values(linear_ptr, .. ..)
    double ve=lm_get_vurve_ve(linear_ptr);

    const double* array_ptr=lm_get_curve_at_time(linear_ptr,0.5*lm_get_curve_total_time(linear_ptr));
    double s=array_ptr[0];
    double v=array_ptr[1];
    double a=array_ptr[2];

    printf("s %f \n",s);
    printf("v %f \n",v);
    printf("a %f \n",a);

    done!
*/

extern "C" linear_motion* lm_init_ptr(){
    return new linear_motion();
}

extern "C" void lm_set_values(linear_motion *ptr,
                              double velocity_begin,
                              double velocity_end,
                              double velocity_max,
                              double acceleration_max,
                              double displacment,
                              bool debug){
    ptr->set_debug(debug,debug);
    ptr->set_curve_values(velocity_begin, velocity_end, velocity_max, acceleration_max, displacment);
}

extern "C" double lm_get_curve_ve(linear_motion *ptr){
    return ptr->get_curve_ve();
}

extern "C"  double lm_get_curve_total_time(linear_motion *ptr){
    return ptr->get_curve_total_time();
}

extern "C" const double* lm_get_curve_at_time(linear_motion *ptr, double t){

    static double result[3];  // Static array to hold the result
    ptr->get_curve_at_time(t, result[0], result[1], result[2]);
    return result;
}

extern "C" const double* lm_run_velocity_mode(linear_motion *ptr,
                                              double velocity_begin,
                                              double velocity_max,
                                              double displacement_begin,
                                              double acceleration_max,
                                              double machine_limit_max, // 0=Infinite, else use value.
                                              double machine_limit_min, // 0=Infinite, else use value.
                                              bool direction_reverse,   // Forward=0, reverse=1.
                                              bool stop,                // Stop=1, No stop=0.
                                              double time_interval      // Servo period.
                                              ){

    static double result[3];  // Static array to hold the result
    ptr->run_velocity_mode(velocity_begin,
                           velocity_max,
                           displacement_begin,
                           acceleration_max,
                           machine_limit_max,
                           machine_limit_min,
                           direction_reverse,
                           stop,
                           time_interval,
                           result[0], result[1], result[2] );
    return result;
}

extern "C" void perform_unit_test(linear_motion *ptr){
    ptr->unit_test_displacement();
}

//! Set debug state.
void linear_motion::set_debug(bool state, bool state_at_time){
    debug=state;
    debug_time_request=state_at_time;
}

//! Set curve values and calculate curve.
void linear_motion::set_curve_values(double velocity_begin,
                                     double velocity_end,
                                     double velocity_max,
                                     double acceleration_max,
                                     double displacment){
    vo=velocity_begin;
    ve=velocity_end;
    vm=velocity_max;
    a=acceleration_max;
    s=displacment;
    so=0;
    se=s;

    res={none,none,none}; // Reset.

    if(s>=0){
        curve_flow_positive();
    }
    if(s<0){
        curve_flow_negative();
    }

    print_curve_info();
}

//! Set curve value's.
void linear_motion::set_curve_values(double velocity_begin,
                                     double velocity_end,
                                     double velocity_max,
                                     double acceleration_max,
                                     double displacment_begin,
                                     double displacment_end){
    s=get_s_given_so_se(displacment_begin,displacment_end);
    so=displacment_begin;
    se=displacment_end;

    vo=velocity_begin;
    ve=velocity_end;
    vm=velocity_max;
    a=acceleration_max;

    res={none,none,none}; // Reset.

    if(s>=0){
        curve_flow_positive();
    }
    if(s<0){
        curve_flow_negative();
    }

    print_curve_info();
}

//! Calculate total curve time.
double linear_motion::get_curve_total_time(){
    return t1+t2+t3;
}

double linear_motion::get_curve_ve(){
    if(debug){
        std::cout<<"curve ve:"<<ve<<std::endl;
    }
    return ve;
}

//! Interpolate curve at a certain time.
void linear_motion::get_curve_at_time(double t, double &sr, double &vr, double &ar){

    //! Period t1.
    if(t<=t1 && std::get<0>(res)!=3){

        if(std::get<0>(res)==0){
            // std::cout << "acc" << std::endl;
            vr=ve_acc_dcc_at_time(vo,a,t);
            sr=s_acc_dcc(vo,vr,a);
            ar=a;
        }
        if(std::get<0>(res)==1){
            //std::cout << "steady" << std::endl;
            vr=vm;
            sr=s_steady(vr,t);
            ar=0;
        }
        if(std::get<0>(res)==2){
            //std::cout << "dcc" << std::endl;
            vr=ve_acc_dcc_at_time(vo,-a,t);
            sr=s_acc_dcc(vo,vr,-a);
            ar=-a;
        }
    }
    //! Period t2.
    if(t>=t1 && t<=t1+t2 && std::get<1>(res)!=3){

        double ti=t-t1;
        if(std::get<1>(res)==0){
            // std::cout << "acc" << std::endl;
            vr=ve_acc_dcc_at_time(vm,a,ti);
            sr=s_acc_dcc(vm,vr,a)+s1;
            ar=a;
        }
        if(std::get<1>(res)==1){
            //std::cout << "steady" << std::endl;
            vr=vm;
            sr=s_steady(vr,ti)+s1;
            ar=0;
        }
        if(std::get<1>(res)==2){
            //std::cout << "dcc" << std::endl;
            vr=ve_acc_dcc_at_time(vm,-a,ti);
            sr=s_acc_dcc(vm,vr,-a)+s1;
            ar=-a;
        }
    }
    //! Period t3.
    if(t>=t1+t2 && t<=t1+t2+t3 && std::get<1>(res)!=3){

        double ti=t-t1-t2;
        if(std::get<2>(res)==0){
            // std::cout << "acc" << std::endl;
            vr=ve_acc_dcc_at_time(vm,a,ti);
            sr=s_acc_dcc(vm,vr,a)+s1+s2;
            ar=a;
        }
        if(std::get<2>(res)==1){
            //std::cout << "steady" << std::endl;
            vr=vm;
            sr=s_steady(vr,ti)+s1+s2;
            ar=0;
        }
        if(std::get<2>(res)==2){
            //std::cout << "dcc" << std::endl;
            vr=ve_acc_dcc_at_time(vm,-a,ti);
            sr=s_acc_dcc(vm,vr,-a)+s1+s2;
            ar=-a;
        }
    }

    // so=-100
    // se=-200
    // s=-100

    sr=get_sr_given_so_s(so,sr);

    if(debug_time_request){
        // Set the output to fixed-point notation
        std::cout << std::fixed << std::setprecision(3);
        // Align and print the values
        std::cout << std::setw(3) << "t:" << std::setw(8) << t
                  << std::setw(3) << " sr:" << std::setw(8) << sr
                  << std::setw(3) << " vr:" << std::setw(8) << vr
                  << std::setw(3) << " ar:" << std::setw(8) << ar
                  << std::endl;
    }
}

//! Sum of "so" displacement begin and "s" displacment curve.
double linear_motion::get_sr_given_so_s(double so_now, double s_now){

    // so 10 s 100
    if(so_now>=0 && s_now>=0){
        return so_now+s_now;
    }

    // so -10 + s -100
    if(so_now<=0 && s_now<=0){
        return -abs(abs(so_now)+abs(s_now));
    }

    // so -10 + s 100
    if(so_now<=0 && s_now>=0){
        return so_now+s_now;
    }
    // so 100 + s -10
    if(so_now>=0 && s_now<=0){
        return so_now-abs(s_now);
    }

    if(so_now==s_now){
        return 0;
    }
    return 0;

}


//! Calculate the netto displacement of 2 values, and get the
//! right sign of direction.
double linear_motion::get_s_given_so_se(double so, double se){

    // so 10 se 100
    if(so>=0 && se>=0 && so<se){
        return se-so;
    }
    // so 100 se 10
    if(so>=0 && se>=0 && so>se){
        return -abs(so-se);
    }

    // so -100 + se -10
    if(so<=0 && se<=0 && so<se){
        return abs(so)-abs(se);
    }
    // so -10 + se -100
    if(so<=0 && se<=0 && so>se){
        return -abs(so-se);
    }

    // so -10 + se 100
    if(so<=0 && se>=0 && so<se){
        return abs(so)+se;
    }
    // so 100 + se -10
    if(so>=0 && se<=0 && so>se){
        return -abs(so+abs(se));
    }

    if(so==se){
        return 0;
    }
    return 0;
}

//! Calculate "a" acceleration given "s" displacement ant "t" time.
double linear_motion::a_given_s_vo_t(double s, double vo, double t) {
    return 2 * (s - vo * t) / (t * t);
}

//! Calculate "s" displacement given "vo" velocity begin and "t" time.
double linear_motion::s_given_a_vo_t(double a, double vo, double t) {
    return vo * t + 0.5 * a * t * t;
}

//! Construct a curve for positive feed.
//! Calculate curve periods t1,t2,t3 and their state : acc,steday,dcc,none.
//! Calculate curve time and displacement for each period t1,t2,t3.
void linear_motion::curve_flow_positive(){

    // Input validation
    if(a==0){
        std::cout<<"Error: curve construction error"<<std::endl;
        t1=0, t2=0, t3=0;
        s1=0, s2=0, s3=0;
        res=std::make_tuple(none,none,none);
        return;
    }
    if(s==0){
        std::cout<<"Error: curve construction error"<<std::endl;
        t1=0, t2=0, t3=0;
        s1=0, s2=0, s3=0;
        res=std::make_tuple(none,none,none);
    }

    s1=0;
    s2=0;
    s3=0;
    t1=0;
    t2=0;
    t3=0;

    double a1=a;
    double a2=a;
    if(vo<vm){
        a1=abs(a1);
    }
    if(vo>vm){
        a1=-abs(a1);
    }
    if(vm<ve){
        a2=abs(a2);
    }
    if(vm>ve){
        a2=-abs(a2);
    }

    s1=s_acc_dcc(vo,vm,a1);
    s3=s_acc_dcc(vm,ve,a2);
    s2=s-s1-s3;

    t1=t_acc_dcc(vo,vm,a1);
    t2=t_steady(s2,vm);
    t3=t_acc_dcc(vm,ve,a2);

    if(s2<0 && s_acc_dcc(vo,ve, a_sign(vo,ve,a) )>=s && vo<ve){
        s1=s;
        s2=0;
        s3=0;
        ve=ve_acc_given_s_vo_a(a,vo,s1);
        t1=t_acc_dcc(vo,ve,a);
        t2=0;
        t3=0;
        res=std::make_tuple(acc,none,none);
        return;
    }

    if(s2<0 && s_acc_dcc(vo,ve, a_sign(vo,ve,a) )>=s && vo>ve){
        s1=s;
        s2=0;
        s3=0;
        ve=ve_dcc_given_s_vo_a(a,vo,s1);
        t1=t_acc_dcc(vo,ve,a);
        t2=0;
        t3=0;
        res=std::make_tuple(dcc,none,none);
        return;
    }

    if(s2>=0 && vo==vm && ve==vm){
        s1=s;
        s2=0;
        s3=0;
        t1=t_steady(s1,vm);
        t2=0;
        t3=0;
        res=std::make_tuple(steady,none,none);
        return;
    }

    if(s2>=0 && vo<vm && ve<vm){
        res=std::make_tuple(acc,steady,dcc);
        return;
    }

    if(s2>=0 && vo>vm && ve<vm){
        res=std::make_tuple(dcc,steady,dcc);
        return;
    }

    if(s2>=0 && vo<vm && ve>vm){
        res=std::make_tuple(acc,steady,acc);
        return;
    }

    if(s2>=0 && vo>vm && ve>vm){
        res=std::make_tuple(dcc,steady,acc);
        return;
    }

    if(s2>=0 && vo>vm && ve==vm){
        res=std::make_tuple(dcc,steady,none);
        return;
    }

    if(s2>=0 && vo<vm && ve==vm){
        res=std::make_tuple(acc,steady,none);
        return;
    }

    if(s2>=0 && vo==vm && ve<vm){
        res=std::make_tuple(none,steady,dcc);
        return;
    }

    if(s2>=0 && vo==vm && ve>vm){
        res=std::make_tuple(none,steady,acc);
        return;
    }

    if(s2<0 && vm>vo && vm>ve){
        for(float i=vm; i>std::max(vo,ve); i-=0.1){ //! Curve vm sampled down to fit s.

            double a1=a;
            double a2=a;
            if(vo<i){
                a1=abs(a1);
            }
            if(vo>i){
                a1=-abs(a1);
            }
            if(i<ve){
                a2=abs(a2);
            }
            if(i>ve){
                a2=-abs(a2);
            }

            s1=s_acc_dcc(vo,i,a1);
            s3=s_acc_dcc(i,ve,a2);
            s2=s-s1-s3;

            if(s2>0){
                vm=i;
                t1=t_acc_dcc(vo,vm,a1);
                t2=t_steady(s2,vm);
                t3=t_acc_dcc(vm,ve,a2);
                res=std::make_tuple(acc,steady,dcc);
                return;
            }
        }
    }

    if(s2<0 && vm<vo && vm<ve){
        for(float i=vm; i<std::min(vo,ve); i+=0.1){ //! Curve vm sampled up to fit s.

            double a1=a;
            double a2=a;
            if(vo<i){
                a1=abs(a1);
            }
            if(vo>i){
                a1=-abs(a1);
            }
            if(i<ve){
                a2=abs(a2);
            }
            if(i>ve){
                a2=-abs(a2);
            }

            s1=s_acc_dcc(vo,i,a1);
            s3=s_acc_dcc(i,ve,a2);
            s2=s-s1-s3;

            if(s2>0){
                vm=i;
                t1=t_acc_dcc(vo,vm,a1);
                t2=t_steady(s2,vm);
                t3=t_acc_dcc(vm,ve,a2);
                res=std::make_tuple(dcc,steady,acc);
                return;
            }
        }
    }

    std::cout<<""<<std::endl;
    std::cout<<"Error: curve construction error"<<std::endl;
}

//! Construct a curve for negative feed.
//! Calculate curve periods t1,t2,t3 and their state : acc,steday,dcc,none.
//! Calculate curve time and displacement for each period t1,t2,t3.
void linear_motion::curve_flow_negative(){

    // Input validation
    if(a==0){
        std::cout<<"Error: curve construction error"<<std::endl;
        t1=0, t2=0, t3=0;
        s1=0, s2=0, s3=0;
        res=std::make_tuple(none,none,none);
        return;
    }
    if(s==0){
        std::cout<<"Error: curve construction error"<<std::endl;
        t1=0, t2=0, t3=0;
        s1=0, s2=0, s3=0;
        res=std::make_tuple(none,none,none);
    }

    s1=0;
    s2=0;
    s3=0;
    t1=0;
    t2=0;
    t3=0;

    // Negative "s" displacment, then a negative "vm" value is valid.
    vm=-abs(vm);

    double a1=a;
    double a2=a;
    if(vo>vm){
        a1=-abs(a1);
    }
    if(vo<vm){
        a1=abs(a1);
    }
    if(vm<ve){
        a2=abs(a2);
    }
    if(vm>ve){
        a2=-abs(a2);
    }

    s1=s_acc_dcc(vo,vm,a1);
    s3=s_acc_dcc(vm,ve,a2);
    s2=s-s1-s3;

    t1=t_acc_dcc(vo,vm,a1);
    t2=t_steady(s2,vm);
    t3=t_acc_dcc(vm,ve,a2);

    if(t2<0 && abs(s_acc_dcc(vo,ve, a_sign(vo,ve,a) ))>= abs(s) && vo<ve){
        s1=s;
        s2=0;
        s3=0;
        ve=ve_acc_given_s_vo_a(a,vo,s1);
        t1=t_acc_dcc(vo,ve,a);
        t2=0;
        t3=0;
        res=std::make_tuple(acc,none,none);
        return;
    }

    if(t2<0 && abs(s_acc_dcc(vo,ve, a_sign(vo,ve,a) ))>= abs(s) && vo>ve){
        a1=-abs(a1);
        s1=s;
        s2=0;
        s3=0;
        ve=ve_dcc_given_s_vo_a(a1,vo,s1);

        t1=t_acc_dcc(vo,ve,a1);
        t2=0;
        t3=0;
        res=std::make_tuple(dcc,none,none);
        return;
    }

    if(t2>=0 && vo==vm && ve==vm){
        s1=s;
        s2=0;
        s3=0;
        t1=t_steady(s1,vm);
        t2=0;
        t3=0;
        res=std::make_tuple(steady,none,none);
        return;
    }

    if(t2>=0 && vo<vm && ve<vm){
        res=std::make_tuple(acc,steady,dcc);
        return;
    }

    if(t2>=0 && vo>vm && ve<vm){
        res=std::make_tuple(dcc,steady,dcc);
        return;
    }

    if(t2>=0 && vo<vm && ve>vm){
        res=std::make_tuple(acc,steady,acc);
        return;
    }

    if(t2>=0 && vo>vm && ve>vm){
        res=std::make_tuple(dcc,steady,acc);
        return;
    }

    if(t2>=0 && vo>vm && ve==vm){
        res=std::make_tuple(dcc,steady,none);
        return;
    }

    if(t2>=0 && vo<vm && ve==vm){
        res=std::make_tuple(acc,steady,none);
        return;
    }

    if(t2>=0 && vo==vm && ve<vm){
        res=std::make_tuple(none,steady,dcc);
        return;
    }

    if(t2>=0 && vo==vm && ve>vm){
        res=std::make_tuple(none,steady,acc);
        return;
    }

    if(t2<0 && vm>vo && vm>ve){
        for(float i=vm; i>std::max(vo,ve); i-=0.1){ //! Curve vm sampled down to fit s.

            double a1=a;
            double a2=a;
            if(vo>i){
                a1=-abs(a1);
            }
            if(vo<i){
                a1=abs(a1);
            }
            if(i<ve){
                a2=abs(a2);
            }
            if(i>ve){
                a2=-abs(a2);
            }

            s1=s_acc_dcc(vo,i,a1);
            s3=s_acc_dcc(i,ve,a2);
            s2=s-s1-s3;

            if(s2<0){
                vm=i;
                t1=t_acc_dcc(vo,vm,a1);
                t2=t_steady(s2,vm);
                t3=t_acc_dcc(vm,ve,a2);
                res=std::make_tuple(acc,steady,dcc);
                return;
            }
        }
    }

    if(t2<0 && vm<vo && vm<ve){
        for(float i=vm; i<std::min(vo,ve); i+=0.1){ //! Curve vm sampled up to fit s.

            double a1=a;
            double a2=a;
            if(vo>i){
                a1=-abs(a1);
            }
            if(vo<i){
                a1=abs(a1);
            }
            if(i<ve){
                a2=abs(a2);
            }
            if(i>ve){
                a2=-abs(a2);
            }

            s1=s_acc_dcc(vo,i,a1);
            s3=s_acc_dcc(i,ve,a2);
            s2=s-s1-s3;

            if(s2<0){
                vm=i;
                t1=t_acc_dcc(vo,vm,a1);
                t2=t_steady(s2,vm);
                t3=t_acc_dcc(vm,ve,a2);
                res=std::make_tuple(dcc,steady,acc);
                return;
            }
        }
    }

    std::cout<<""<<std::endl;
    std::cout<<"Error: curve construction error"<<std::endl;
}

//! Print debug information.
void linear_motion::print_curve_info(){

    if(debug){
        std::cout << std::fixed << std::setprecision(3);

        std::cout<<""<<std::endl;
        std::cout<<"- Curve input -"<<std::endl;
        std::cout<<"vo:"<<vo<<" vm:"<<vm<<" ve:"<<ve<<" so:"<<so<<" se:"<<se<<" s:"<<s<<" a:"<<a<<std::endl;
        std::cout<<""<<std::endl;

        std::cout<<"- Curve period t1 :";
        if(std::get<0>(res)==0){
            std::cout << "acc" << std::endl;
        }
        if(std::get<0>(res)==1){
            std::cout << "steady" << std::endl;
        }
        if(std::get<0>(res)==2){
            std::cout << "dcc" << std::endl;
        }
        if(std::get<0>(res)==3){
            std::cout << "none" << std::endl;
        }

        std::cout<<"- Curve period t2 :";
        if(std::get<1>(res)==0){
            std::cout << "acc" << std::endl;
        }
        if(std::get<1>(res)==1){
            std::cout << "steady" << std::endl;
        }
        if(std::get<1>(res)==2){
            std::cout << "dcc" << std::endl;
        }
        if(std::get<1>(res)==3){
            std::cout << "none" << std::endl;
        }

        std::cout<<"- Curve period t3 :";
        if(std::get<2>(res)==0){
            std::cout << "acc" << std::endl;
        }
        if(std::get<2>(res)==1){
            std::cout << "steady" << std::endl;
        }
        if(std::get<2>(res)==2){
            std::cout << "dcc" << std::endl;
        }
        if(std::get<2>(res)==3){
            std::cout << "none" << std::endl;
        }
        std::cout<<""<<std::endl;

        std::cout<<"- Curve times :"<<std::endl;
        std::cout<<"t1:"<<t1<<std::endl;
        std::cout<<"t2:"<<t2<<std::endl;
        std::cout<<"t3:"<<t3<<std::endl;
        ttot=t1+t2+t3;
        std::cout<<"ttot:"<<t1+t2+t3<<std::endl;
        std::cout<<""<<std::endl;

        std::cout<<"- Curve displacement :"<<std::endl;
        std::cout<<"s1:"<<s1<<std::endl;
        std::cout<<"s2:"<<s2<<std::endl;
        std::cout<<"s3:"<<s3<<std::endl;
        stot=s1+s2+s3;

        std::cout<<"stot:"<<s1+s2+s3<<std::endl;
        std::cout<<""<<std::endl;
    }
}

// Here, "s" represents the position, "ve" represents the velocity, "vo" is the initial velocity, and "t" is the time.
// "a" represents acceleration, "d" represents deceleration.

// Linear acceleration:
// s = vo * t + 0.5 * a * t*t
// ve = vo + a * t

// Linear deceleration:
// s = u * t − 0.5 * ​d * t*t
// ve = vo − d * t

// Steady:
// s = vo * t
// ve = vo

//! Calculate stop distance, "v"=velocity, "a"=max_acceleration
//! If velocity>0, stop lenght result is positive..
//! If velocity<0, stop lenght result is negative.
double linear_motion::s_stop(double v, double a){
    // Stopping distance during deceleration: s = (ve^2 - vo^2) / (2 * |a|)
    double stoplenght=(v * v) / (2 * std::abs(a));
    if(v<0){ // Moving in negative direction, is negative stoplenght for s.
        stoplenght=-abs(stoplenght);
    }
    return stoplenght;
}

//! Calculate "t" time for acceleration or deceleration periods, given
//! "vo" velocity begin, "ve" velocity end, "a" acceleration.
double linear_motion::t_acc_dcc(double vo, double ve, double a){
    // Formula: time = (ve - vo) / a
    // Avoid division by zero
    // Time output must always be positive, therefore abs(a) is a<0
    if (a == 0.0) {
        return 0.0;  // Acceleration is zero, time is not well-defined
    }
    // Calculate acceleration time
    return abs(ve - vo) / abs(a);
}

//! Calculate "s" displacement for acceleration or deceleration periods, given
//! "vo" velocity begin, "ve" velocity end, "a" acceleration.
double linear_motion::s_acc_dcc(double vo, double ve, double a){

    // std::cout<<"vo:"<<vo<<std::endl;
    // std::cout<<"ve:"<<ve<<std::endl;
    //std::cout<<"a:"<<a<<std::endl;

    // Acceleration length: s = (ve^2 - vo^2) / (2 * a)
    double sr=(ve * ve - vo * vo) / (2 * a);

    //std::cout<<"result s:"<<sr<<std::endl;

    return sr;
}

//! Calculate "ve" velocity end for acceleration or deceleration periods, given
//! "vo" velocity begin, "a" acceleration, "t" time.
double linear_motion::ve_acc_dcc_at_time(double vo, double a, double t) {
    // Calculate final velocity given time t.
    return vo + a * t;
}

//! Calculate "ve" velocity end for acceleration period, given
//! "s" displacement, "vo" velocity begin, "a" acceleration,
double linear_motion::ve_acc_given_s_vo_a(double s, double vo, double a) {
    // Kinematic equation: ve = sqrt(vo^2 + 2 * a * s)
    return sqrt((vo * vo) + 2 * a * s);
}

//! Calculate "ve" velocity end for deceleration period, given
//! "s" displacement, "vo" velocity begin, "a" acceleration,
double linear_motion::ve_dcc_given_s_vo_a(double s, double vo, double a) {
    // Kinematic equation: ve = sqrt(vo^2 + 2 * a * s)
    // return sqrt((vo * vo) - 2 * a * s);

    // Kinematic equation: ve = sqrt(vo^2 + 2 * a * s)
    double ve_result = vo * vo + 2 * a * s;
    if (ve_result < 0) {
        // Handle cases where the result is negative
        return -sqrt(-ve_result);
    } else {
        return -sqrt(ve_result);
    }
}

//! Calculate "s" displacement for a non acceleration, linear or steady period,
//! given "v" velocity and "t" time.
double linear_motion::s_steady(double v, double t){
    // Linear motion distance: s = v * t
    return v * t;
}

//! Calculate "t" time for a non acceleration, linear or steady period,
//! given "s" displacement and "v" velocity.
double linear_motion::t_steady(double s, double v){
    // Formula: time = distance / velocity
    // Avoid division by zero
    if (v == 0.0) {
        return 0;  // Velocity is zero, time is not well-defined
    }
    // Calculate linear motion time
    return s / v;
}

//! Get the sign of "a" acceleration given "vo" velocity begin
//! and "ve" velocity end. The sign returns "-a" or "a".
double linear_motion::a_sign(double vo, double ve, double a){
    if(vo<ve){
        a=abs(a);
    }
    if(vo>ve){
        a=-abs(a);
    }
    return a;
}

//! Function to control a machine by velocity commands. Used for jogging axis.
//! If machine_limit_max or min is zero, limits are set to INFINITY.
void linear_motion::run_velocity_mode(double velocity_begin,
                                      double velocity_max,
                                      double displacement_begin,
                                      double acceleration_max,
                                      double machine_limit_max,
                                      double machine_limit_min,
                                      bool direction_reverse,
                                      bool stop,
                                      double time_interval,
                                      double &displacement_end,
                                      double &velocity_end,
                                      double &acceleration_end
                                      ){

    vo=velocity_begin;
    vm=velocity_max;
    so=displacement_begin;
    a=acceleration_max;
    dir_rev=direction_reverse;

    double smax=machine_limit_max;
    double smin=machine_limit_min;

    if(smax==0){
        smax=INFINITY;
    }
    if(smin==0){
        smin=-INFINITY;
    }

    if(!stop){ // Run forward, backward.

        if(!direction_reverse){ // Run forward.
            set_curve_values(vo,ve,vm,a,so,smax);
        } else { // Run backward.
            set_curve_values(vo,ve,vm,a,so,smin);
        }
    } else { // Stop sequence.
        s=get_sr_given_so_s(so,s_stop(vo,a));
        set_curve_values(vo,0,vm,a,so,s);
    }

    get_curve_at_time(time_interval,
                      displacement_end,
                      velocity_end,
                      acceleration_end);

    se=displacement_end;

    if(debug){
        std::cout<<"ttot:"<<ttot<<" sr:"<<displacement_end<<" ve:"<<velocity_end<<" ar:"<<acceleration_end<<std::endl;
    }
}

//! Test several functions for correctness.
void linear_motion::unit_test_displacement(){

    std::cout<<"Test forward stop:"<<std::endl;
    std::cout<<"Stoplenght:"<<s_stop(10,2)<<std::endl;

    std::cout<<"Test backward stop:"<<std::endl;
    std::cout<<"Stoplenght:"<<s_stop(-10,2)<<std::endl;

    std::cout<<"Test displacement of 2 values:"<<std::endl;

    double so=10;
    double se=20;
    double st=get_s_given_so_se(so,se);
    std::cout<<"so:"<<so<<" se:"<<se<<" sr:"<<st<<std::endl;

    so=20;
    se=10;
    st=get_s_given_so_se(so,se);
    std::cout<<"so:"<<so<<" se:"<<se<<" sr:"<<st<<std::endl;

    so=-10;
    se=-20;
    st=get_s_given_so_se(so,se);
    std::cout<<"so:"<<so<<" se:"<<se<<" sr:"<<st<<std::endl;

    so=-20;
    se=-10;
    st=get_s_given_so_se(so,se);
    std::cout<<"so:"<<so<<" se:"<<se<<" sr:"<<st<<std::endl;

    so=10;
    se=-20;
    st=get_s_given_so_se(so,se);
    std::cout<<"so:"<<so<<" se:"<<se<<" sr:"<<st<<std::endl;

    so=-10;
    se=20;
    st=get_s_given_so_se(so,se);
    std::cout<<"so:"<<so<<" se:"<<se<<" sr:"<<st<<std::endl;
}

void linear_motion::unit_test_position_mode(bool gnu_plot){
    std::cout<<"- Unit test standard motion. -"<<std::endl;
    double vo=0;            // Velocity begin.
    double ve=0;            // Velocity end.
    double vm=10;           // Velocity max.
    double a=2;             // Acceleration max.
    double so=0;            // Displacement begin.
    double se=100;          // Displacement end;
    double interval=0.2;    // Interval time.
    bool debug=1;           // Set debug output.
    bool debug_time=1;      // Set debug output.

    linear_motion *lm = new linear_motion();
    lm->set_debug(debug, debug_time);
    lm->set_curve_values(vo,ve,vm,a,so,se);

    lm->get_curve_ve();     // Is "ve" velocity end changed to fit "s" displacement?

    double sr=0,vr=0,ar=0;  // Results for "s" displacement, "v" velocity , "a" acceleration.

    for(float t=0; t<lm->get_curve_total_time(); t+=interval){
        lm->get_curve_at_time(t,sr,vr,ar);
    }

    if(gnu_plot){
        lm->plot_gnu_graph(0.1,"Position mode.");
    }
    std::cout<<""<<std::endl;
    delete lm;
}

void linear_motion::unit_test_velocity_mode_run_forward(bool gnu_plot){
    std::cout<<"- Unit test velocity mode. -"<<std::endl;
    double vo=0;            // Velocity begin.
    double vm=10;           // Velocity max.
    double so=0;            // Displacement begin.
    double a=2;             // Acceleration max.
    double limit_max=100;   // Machine upper limit.
    double limit_min=-100;  // Machine lower limit.
    bool motion_reverse=0;  // Positive or negative motion.
    bool motion_stop=0;     // Motion stop.
    double interval=0.2;    // Interval time.

    bool debug=1;           // Set debug output.
    bool debug_time=0;      // Set debug output.

    linear_motion *lm = new linear_motion();
    lm->set_debug(debug, debug_time);

    double sr=0,vr=0,ar=0;  // Results for "s" displacement, "v" velocity , "a" acceleration.

    lm->run_velocity_mode(vo,vm,so,a,limit_max,limit_min,motion_reverse,motion_stop,interval,sr,vr,ar);

    if(gnu_plot){
        lm->plot_gnu_graph(0.1,"Velocity mode run forward.");
    }
    std::cout<<""<<std::endl;
    delete lm;
}

void linear_motion::unit_test_velocity_mode_run_reverse(bool gnu_plot){
    std::cout<<"- Unit test velocity mode. -"<<std::endl;
    double vo=0;            // Velocity begin.
    double vm=10;           // Velocity max.
    double so=0;            // Displacement begin.
    double a=2;             // Acceleration max.
    double limit_max=100;   // Machine upper limit.
    double limit_min=-100;  // Machine lower limit.
    bool motion_reverse=1;  // Positive or negative motion.
    bool motion_stop=0;     // Motion stop.
    double interval=0.2;    // Interval time.

    bool debug=1;           // Set debug output.
    bool debug_time=0;      // Set debug output.

    linear_motion *lm = new linear_motion();
    lm->set_debug(debug, debug_time);

    double sr=0,vr=0,ar=0;  // Results for "s" displacement, "v" velocity , "a" acceleration.

    lm->run_velocity_mode(vo,vm,so,a,limit_max,limit_min,motion_reverse,motion_stop,interval,sr,vr,ar);

    if(gnu_plot){
        lm->plot_gnu_graph(0.1,"Velocity mode run reverse.");
    }
    std::cout<<""<<std::endl;
    delete lm;
}

void linear_motion::unit_test_velocity_mode_run_stop(bool gnu_plot){
    std::cout<<"- Unit test velocity mode. -"<<std::endl;
    double vo=-9;            // Velocity begin.
    double vm=10;           // Velocity max.
    double so=5;            // Displacement begin.
    double a=2;             // Acceleration max.
    double limit_max=100;   // Machine upper limit.
    double limit_min=-100;  // Machine lower limit.
    bool motion_reverse=0;  // Positive or negative motion.
    bool motion_stop=1;     // Motion stop.
    double interval=0.2;    // Interval time.

    bool debug=1;           // Set debug output.
    bool debug_time=0;      // Set debug output.

    linear_motion *lm = new linear_motion();
    lm->set_debug(debug, debug_time);

    double sr=0,vr=0,ar=0;  // Results for "s" displacement, "v" velocity , "a" acceleration.

    lm->run_velocity_mode(vo,vm,so,a,limit_max,limit_min,motion_reverse,motion_stop,interval,sr,vr,ar);

    if(gnu_plot){
        lm->plot_gnu_graph(0.1,"Velocity mode run stop.");
    }
    std::cout<<""<<std::endl;
    delete lm;
}

void linear_motion::plot_gnu_graph(double interval, std::string graph_header_text){

    double sr,vr,ar;
    double ttot=get_curve_total_time();

    std::vector<double> time;
    std::vector<double> velocity;
    std::vector<double> displacement;
    std::vector<double> acceleration;

    for(double i=0; i<ttot; i+=interval){
        get_curve_at_time(i,sr,vr,ar);
        time.push_back(i);
        displacement.push_back(sr);
        velocity.push_back(vr);
        acceleration.push_back(ar);
    }

    // Open a pipe to GNU Plot
    FILE *gnuplotPipe = popen("gnuplot -persist", "w");

    if (!gnuplotPipe) {
        std::cerr << "Error opening pipe to GNU Plot" << std::endl;
        return;
    }

    // Send commands to GNU Plot
    std::string text="set title '";
    text.append(graph_header_text);
    text.append("'\n");

    fprintf(gnuplotPipe, "%s", text.c_str());
    fprintf(gnuplotPipe, "set xlabel 'Time'\n");
    fprintf(gnuplotPipe, "set ylabel 'Values'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'Velocity', '-' with lines title 'Displacement', '-' with lines title 'Acceleration'\n");

    // Send data to GNU Plot
    for (size_t i = 0; i < time.size(); ++i) {
        fprintf(gnuplotPipe, "%lf %lf\n", time[i], velocity[i]);
    }
    fprintf(gnuplotPipe, "e\n");

    for (size_t i = 0; i < time.size(); ++i) {
        fprintf(gnuplotPipe, "%lf %lf\n", time[i], displacement[i]);
    }
    fprintf(gnuplotPipe, "e\n");

    for (size_t i = 0; i < time.size(); ++i) {
        fprintf(gnuplotPipe, "%lf %lf\n", time[i], acceleration[i]);
    }
    fprintf(gnuplotPipe, "e\n");

    // Close the pipe
    pclose(gnuplotPipe);
}

