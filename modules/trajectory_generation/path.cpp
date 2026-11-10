//Assignment 3
#include "trajectory_generation/path.h"

#include <utility>

namespace ais4104::trajectory {

//TASK: T.3a
//REFERENCE: 
praxis::expected<Eigen::VectorXd, praxis::refusal> joint_straight_line(const Eigen::VectorXd &start, const Eigen::VectorXd &end, double s)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: T.3b
//REFERENCE: 
praxis::expected<Eigen::Matrix4d, praxis::refusal> screw(const praxis::rigid_motion::screw_ops &motion, const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &start,
                                                         const Eigen::Matrix4d &end, double s)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: T.3c
//REFERENCE:
praxis::expected<Eigen::Matrix4d, praxis::refusal> decoupled(const praxis::rigid_motion::screw_ops &motion, const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &start,
                                                             const Eigen::Matrix4d &end, double s)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

}
