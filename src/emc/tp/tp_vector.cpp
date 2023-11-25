#include "tp_vector.h"
#include "tp_scurve.h"

tp_vector::tp_vector()
{

}

extern "C" tp_vector* vector_init_ptr(){
    return new tp_vector();
}

extern "C" void vector_add_segment(tp_vector *ptr, struct tp_segment b){
    ptr->pvec.push_back(b);
}

extern "C" void vector_set_vm(tp_vector *ptr, int index, double value){
    ptr->pvec.at(index).vm=value;
}

extern "C" void vector_remove_last_segment(tp_vector *ptr){
    ptr->pvec.pop_back();
}

extern "C" int vector_size_c(tp_vector *ptr){
    return ptr->pvec.size();
}

extern "C" void vector_clear(tp_vector *ptr){
    return ptr->pvec.clear();
}

extern "C" int vector_at_id(tp_vector *ptr, int n){
    return ptr->pvec.at(n).primitive_id;
}

extern "C" struct tp_segment vector_at(tp_vector *ptr, int index){
    return ptr->pvec.at(index);
}

extern "C" void vector_set_end_angle(tp_vector *ptr, int index, double angle_deg){
    ptr->pvec.at(index).angle_end=angle_deg;
}

//! Interpolates traject progress 0-1.
void tp_vector::interpolate_traject(double traject_progress,
                                    double traject_lenght,
                                    double &curve_progress, int &curve_nr){

    double ltot=traject_lenght;
    double l=0;
    for(uint i=0; i<pvec.size(); i++){
        double blocklenght=pvec[i].path_lenght;
        if(traject_progress>=l/ltot && traject_progress<(l+blocklenght)/ltot){

            double low_pct=l/ltot;                                   //10%
            double high_pct=(l+blocklenght)/ltot;                    //25%
            double range=high_pct-low_pct;                           //25-10=15%
            double offset_low=traject_progress-low_pct;              //12-10=2%
            curve_progress=offset_low/range;
            curve_nr=i;
            return;
        }
        l+=blocklenght;
    }
}

extern "C" void vector_interpolate_traject_c(tp_vector *ptr,
                                             double traject_progress,
                                             double traject_lenght,
                                             double *curve_progress,
                                             int *curve_nr){

    double curve_progress_=0;
    int curve_nr_=0;
    ptr->interpolate_traject(traject_progress, traject_lenght, curve_progress_, curve_nr_);

    *curve_progress=curve_progress_;
    *curve_nr=curve_nr_;
}

