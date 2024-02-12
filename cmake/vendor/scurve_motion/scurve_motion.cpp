#include "scurve_motion.h"
#include <cassert>

scurve_motion::scurve_motion()
{

}

/*
 * Example how to use this c++ code in c :
 *
    // Add a copy of the exter "C" functions :
    struct scurve_motion *linear_ptr;
    extern scurve_motion* lm_init_ptr();
    extern void lm_set_values(scurve_motion *ptr,
                           double velocity_begin,
                           double velocity_end,
                           double velocity_max,
                           double acceleration_max,
                           double displacment,
                           bool debug);
    extern double lm_get_curve_ve(scurve_motion *ptr);
    extern double lm_get_curve_total_time(scurve_motion *ptr);
    extern const double* lm_get_curve_at_time(scurve_motion *ptr, double t);

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

extern "C" scurve_motion* lm_init_ptr(){
    return new scurve_motion();
}

extern "C" void lm_set_values(scurve_motion *ptr,
                              double velocity_begin,
                              double velocity_end,
                              double velocity_max,
                              double acceleration_max,
                              double displacment,
                              bool debug){
    ptr->set_debug(debug,debug);
    ptr->set_curve_values(velocity_begin, velocity_end, velocity_max, acceleration_max, displacment);
}

extern "C" double lm_get_curve_ve(scurve_motion *ptr){
    return ptr->get_curve_ve();
}

extern "C"  double lm_get_curve_total_time(scurve_motion *ptr){
    return ptr->get_curve_total_time();
}

extern "C" const double* lm_get_curve_at_time(scurve_motion *ptr, double t){

    static double result[3];  // Static array to hold the result
    ptr->get_curve_at_time(t, result[0], result[1], result[2]);
    return result;
}

//! Set debug state.
void scurve_motion::set_debug(bool state, bool state_at_time){
    debug=state;
    debug_time_request=state_at_time;
}

//! Set curve values and calculate curve.
void scurve_motion::set_curve_values(double velocity_begin,
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

void scurve_motion::set_curve_values(double velocity_begin,
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
double scurve_motion::get_curve_total_time(){
    return t1+t2+t3;
}

double scurve_motion::get_curve_ve(){
    if(debug){
        std::cout<<"curve ve:"<<ve<<std::endl;
    }
    return ve;
}

//! Interpolate curve at a certain time.
void scurve_motion::get_curve_at_time(double t, double &sr, double &vr, double &ar){

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
double scurve_motion::get_sr_given_so_s(double so_now, double s_now){

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
double scurve_motion::get_s_given_so_se(double so, double se){

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

double scurve_motion::a_given_s_vo_t(double s, double vo, double t) {
    return 2 * (s - vo * t) / (t * t);
}

double scurve_motion::s_given_a_vo_t(double a, double vo, double t) {
    return vo * t + 0.5 * a * t * t;
}

//! Calculate curve periods t1,t2,t3 and their state : acc,steday,dcc,none.
//! Calculate curve time and displacement for each period t1,t2,t3.
void scurve_motion::curve_flow_positive(){

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
        ve=ve_acc_given_s(a,vo,s1);
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
        ve=ve_dcc_given_s(a,vo,s1);
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

void scurve_motion::curve_flow_negative(){

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
        ve=ve_acc_given_s(a,vo,s1);
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
        ve=ve_dcc_given_s(a1,vo,s1);

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

void scurve_motion::print_curve_info(){

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

// Calculate stopping distance during deceleration until velocity becomes zero
double scurve_motion::s_stop(double v, double a){
    // Stopping distance during deceleration: s = (ve^2 - vo^2) / (2 * |a|)
    return (v * v) / (2 * std::abs(a));
}

double scurve_motion::t_acc_dcc(double vo, double ve, double a){
    // Formula: time = (ve - vo) / a
    // Avoid division by zero
    // Time output must always be positive, therefore abs(a) is a<0
    if (a == 0.0) {
        return 0.0;  // Acceleration is zero, time is not well-defined
    }
    // Calculate acceleration time
    return abs(ve - vo) / abs(a);
}

double scurve_motion::s_acc_dcc(double vo, double ve, double a){

    // std::cout<<"vo:"<<vo<<std::endl;
    // std::cout<<"ve:"<<ve<<std::endl;
    //std::cout<<"a:"<<a<<std::endl;

    // Acceleration length: s = (ve^2 - vo^2) / (2 * a)
    double sr=(ve * ve - vo * vo) / (2 * a);

    //std::cout<<"result s:"<<sr<<std::endl;

    return sr;
}

double scurve_motion::ve_acc_dcc_at_time(double vo, double a, double t) {
    // Calculate final velocity given time t.
    return vo + a * t;
}

double scurve_motion::ve_acc_given_s(double a, double vo, double s) {
    // Kinematic equation: ve = sqrt(vo^2 + 2 * a * s)
    return sqrt((vo * vo) + 2 * a * s);
}

double scurve_motion::ve_dcc_given_s(double a, double vo, double s) {
    // Kinematic equation: ve = sqrt(vo^2 + 2 * a * s)
    //return sqrt((vo * vo) - 2 * a * s);

    // Kinematic equation: ve = sqrt(vo^2 + 2 * a * s)
    double ve_result = vo * vo + 2 * a * s;
    if (ve_result < 0) {
        // Handle cases where the result is negative
        return -sqrt(-ve_result);
    } else {
        return -sqrt(ve_result);
    }
}

double scurve_motion::s_steady(double v, double t){
    // Linear motion distance: s = v * t
    return v * t;
}

double scurve_motion::t_steady(double s, double v){
    // Formula: time = distance / velocity
    // Avoid division by zero
    if (v == 0.0) {
        return 0;  // Velocity is zero, time is not well-defined
    }
    // Calculate linear motion time
    return s / v;
}

double scurve_motion::a_sign(double vo, double ve, double a){
    if(vo<ve){
        a=abs(a);
    }
    if(vo>ve){
        a=-abs(a);
    }
    return a;
}

void scurve_motion::perform_unit_test(){

    std::cout<<"Test displacement of 2 values:"<<std::endl;

    double so=10;
    double se=20;
    double st=get_s_given_so_se(so,se);

    so=20;
    se=10;
    st=get_s_given_so_se(so,se);

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

    double vo=0;            // Velocity begin.
    double ve=0;            // Velocity end.
    double vm=10;           // Velocity max.
    double a=2;             // Acceleration max.
    so=-10;                 // Displacement begin.
    se=-100;                   // Displacement end;
    double interval=0.2;    // Interval time.
    bool debug=1;           // Set debug output.
    bool debug_time=1;      // Set debug output.

    scurve_motion *lm = new scurve_motion();
    lm->set_debug(debug, debug_time);
    lm->set_curve_values(vo,ve,vm,a,so,se);

    lm->get_curve_ve();     // Is "ve" velocity end changed to fit "s" displacement?

    double sr=0,vr=0,ar=0;  // Results for "s" displacement, "v" velocity , "a" acceleration.

    std::cout << std::fixed << std::setprecision(1);

    for(float t=0; t<lm->get_curve_total_time(); t+=interval){
        lm->get_curve_at_time(t,sr,vr,ar);
    }

    delete lm;
}
