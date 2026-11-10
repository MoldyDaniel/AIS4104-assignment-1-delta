//Assignment 3
#include "trajectory_generation/via_points.h"

namespace ais4104::trajectory {

//TASK: T.5a
//REFERENCE:
praxis::expected<cubic_segment, praxis::refusal> segment_coefficients(const praxis::trajectory::via_point &from, const praxis::trajectory::via_point &to)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: T.5b
//REFERENCE:
praxis::expected<praxis::trajectory::trajectory_sample, praxis::refusal> segment_sample(const cubic_segment &segment, double dt)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

}
