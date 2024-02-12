#include "linear_motion.h"

int main(int argc, char *argv[])
{
    linear_motion().unit_test_displacement();
    linear_motion().unit_test_position_mode(true);
    linear_motion().unit_test_velocity_mode_run_forward(true);
    linear_motion().unit_test_velocity_mode_run_reverse(true);
    linear_motion().unit_test_velocity_mode_run_stop(true);
    return 0;
}
