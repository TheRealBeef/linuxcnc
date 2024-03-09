### scurve_motion Library

![S-curve Motion](screen.jpg)

### Overview

The `scurve_construct` library is a C++ library designed for jogging and position control applications with S-curve motion profiles. It enables smooth acceleration and deceleration for precise and controlled motion.

### Features

- S-curve motion profiling for smooth jogging.
- Jerk limited.
- Support for both forward and reverse jogging.
- Linear acceleration stage between concave & convex curve depending on jerk value.
- Gui project to preview curve outputs in a qt-realtime-plot.
- Position control.

### Language

- ~/scurve_construct_c  C  
- ~/scurve_construct    C++ 

### Example for c

```
#include "scurve_construct.h"
#include <stdio.h>
#include <stdbool.h>

int main(int argc, char *argv[]) {

    double jermax = 10;
    double accmax = 10;
    double maxvel = 10;
    double curvel = 0;
    double curacc = 0;
    double cyctim = 0.001;
    struct scurve_data d;

    set_init_values(jermax,
                    accmax,
                    curvel,
                    curacc,
                    maxvel,
                    cyctim,
                    &d);

    // Run up to the fwd tarpos.
    jog_forward_pressed(&d);

    double vr, sr, ar;
    d.fwd_tarpos= 10;
    d.rev_tarpos = -10;

    while(1){

        // Update curve cycle.
        int state=update(&d, &vr, &ar, &sr);

        printf("state: %i ", state);
        printf("vr: %f ", vr);
        printf("ar: %f ", ar);
        printf("sr: %f \n", sr);

        if(state){
            break;
        }

    }
    return 0;
}
```

### Prerequisites

- CMake (version 3.5 or higher)
- C++ compiler

### Building

```bash
mkdir build
cd build
cmake ..
make
