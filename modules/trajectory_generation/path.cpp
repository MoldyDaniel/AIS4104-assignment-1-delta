//Assignment 3
#include "trajectory_generation/path.h"

#include <utility>

namespace ais4104::trajectory {

//TASK: T.3a
//REFERENCE: Equation (9.3) page 328, MR pre-print 2019
praxis::expected<Eigen::VectorXd, praxis::refusal> joint_straight_line(const Eigen::VectorXd &start, const Eigen::VectorXd &end, double s)
{
    if(start.size() != end.size())
        return praxis::unexpected(praxis::refusal::unsupported_input);
    Eigen::VectorXd joint_line = start -s*(start-end);
    return joint_line;
}

//TASK: T.3b
//REFERENCE:
praxis::expected<Eigen::Matrix4d, praxis::refusal> screw(const praxis::rigid_motion::screw_ops &motion, const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &start,
                                                         const Eigen::Matrix4d &end, double s)
{
    auto inv_start = frames.inverse(start);

    auto log = motion.matrix_logarithm_se3((inv_start*end));
    if (!log.has_value())
        return praxis::unexpected(praxis::refusal::unsupported_input);
    auto S = log.value().first*s;
    auto theta = log.value().second;
    Eigen::Vector3d v(3);
    v <<    S(0),
            S(1),
            S(2);

    Eigen::Vector3d w(3);
    w <<    S(3),
            S(4),
            S(5);
    auto exp = motion.matrix_exponential_se3(v,w,theta);
    auto joint_line = start*exp;
    return joint_line;
}

//TASK: T.3c
//REFERENCE:
praxis::expected<Eigen::Matrix4d, praxis::refusal> decoupled(const praxis::rigid_motion::screw_ops &motion, const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &start,
                                                             const Eigen::Matrix4d &end, double s)
{
    auto inv_start = frames.inverse(start);

    auto log = motion.matrix_logarithm_so3((inv_start*end));
    if (!log.has_value())
        return praxis::unexpected(praxis::refusal::unsupported_input);
    auto S = log.value().first*s;
    auto theta = log.value().second;
    Eigen::Vector3d v(3);
    v <<    S(0),
            S(1),
            S(2);

    Eigen::Vector3d w(3);
    w <<    S(3),
            S(4),
            S(5);
    auto exp = motion.matrix_exponential_se3(v,w,theta);
    auto joint_line = start*exp;
    return joint_line;
    return praxis::unexpected(praxis::refusal::not_implemented);
}

}
