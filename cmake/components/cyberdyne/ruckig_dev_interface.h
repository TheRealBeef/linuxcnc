#ifndef RUCKIG_DEV_INTERFACE_H
#define RUCKIG_DEV_INTERFACE_H

#include "ruckig_dev_format.h"
#include <ruckig/ruckig.hpp>

class ruckig_dev_interface
{
public:
    ruckig_dev_interface();

    ruckig::InputParameter<1> ruckig_c_data_to_cpp(ruckig_c_data input);
    ruckig_c_data ruckig_cpp_data_to_c(ruckig::InputParameter<1> input);

    ruckig_c_data ruckig_calculate_offline(ruckig_c_data in);
    ruckig_c_data ruckig_calculate_online(ruckig_c_data in);

};




#endif // RUCKIG_DEV_INTERFACE_H


