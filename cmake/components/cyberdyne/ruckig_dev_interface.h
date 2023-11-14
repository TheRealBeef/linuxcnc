#ifndef RUCKIG_DEV_INTERFACE_H
#define RUCKIG_DEV_INTERFACE_H

#ifdef __cplusplus

#include "ruckig_dev_format.h"
#include <ruckig/ruckig.hpp>

class ruckig_dev_interface
{
public:
    ruckig_dev_interface();

    ruckig::InputParameter<1> ruckig_c_data_to_cpp(ruckig_c_data input);
    ruckig_c_data ruckig_cpp_data_to_c(ruckig::InputParameter<1> input);

    int ruckig_calculate(ruckig_c_data in, ruckig_c_data &out);

    std::vector<ruckig_c_waypoint> pointvec;
};


//! Here it tells if this code is used in c, convert the class to a struct. This is handy!
#else
typedef struct ruckig_dev_interface ruckig_dev_interface;
#endif //! cplusplus

#endif // RUCKIG_DEV_INTERFACE_H


