//Assignment 3
#include "trajectory_generation/time_scaling.h"

#include <cmath>

namespace ais4104::trajectory {

//TASK: T.4a
//REFERENCE: 
praxis::expected<praxis::trajectory::scaling_sample, praxis::refusal> cubic(double t, double duration)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: T.4b
//REFERENCE:
praxis::expected<praxis::trajectory::scaling_sample, praxis::refusal> quintic(double t, double duration)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

}
