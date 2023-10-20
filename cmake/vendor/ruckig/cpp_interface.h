#ifndef CPP_INTERFACE_H
#define CPP_INTERFACE_H

#include <iostream>
#include <vector>
#include <fstream>

//! https://github.com/pantor/ruckig
#include "ruckig.hpp"
#include "ruckig_format.h"

/*
int i=1;
ruckig::Ruckig<ruckig::DynamicDOFs> otg {static_cast<size_t>(i), 0.001};
ruckig::InputParameter<ruckig::DynamicDOFs> in {static_cast<size_t>(i)};
ruckig::OutputParameter<ruckig::DynamicDOFs> out {static_cast<size_t>(i)};
std::array<double, 3> pos, vel, acc;
*/

class Cpp_interface
{
public:

public:
    Cpp_interface();
    result dofs(result input);
private:

};

#endif
