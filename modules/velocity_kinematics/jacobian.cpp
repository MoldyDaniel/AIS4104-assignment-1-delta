//Assignment 2
#include "velocity_kinematics/jacobian.h"

#include <span>
#include <vector>
#include <cstddef>

namespace ais4104::manipulator {

//TASK: 4a
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 5.1.1, eq. (5.11).
praxis::expected<praxis::manipulator::jacobian, praxis::refusal> space_jacobian(const praxis::rigid_motion::screw_ops &screw, std::span<const Eigen::Vector6d> space_screws,
                                                                                const Eigen::VectorXd &theta)
{
    const std::size_t n = space_screws.size();
    praxis::manipulator::jacobian J_s(6, static_cast<Eigen::Index>(n));
    // Cause first column is S1
    J_s.col(0) = space_screws[0];
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // Since it ends at i-1 i is set to 1
    for (std::size_t i = 1; i < n; ++i)
    {
        const Eigen::Matrix4d exp_i =
            screw.matrix_exponential_screw(
                space_screws[i - 1],
                theta(static_cast<Eigen::Index>(i - 1)));
        T = T * exp_i;

        // Since the array only requires all of the exp matrices they can be
        // multiplied individually then made into a adjoint map.
        const auto J_si = screw.adjoint_map(space_screws[i], T);

        // Every column is saved since The Matrix is carried over and reused.
        // And this starts from the second column since the first is filled.
        J_s.col(static_cast<Eigen::Index>(i)) = J_si.value();
    }
    return J_s;


    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 4b
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 5.1.2, eq. (5.18).
praxis::expected<praxis::manipulator::jacobian, praxis::refusal> body_jacobian(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                                                               const praxis::manipulator::forward_kinematics_ops &forward, const Eigen::Matrix4d &m,
                                                                               std::span<const Eigen::Vector6d> space_screws, const Eigen::VectorXd &theta)
{
    const auto J_s_result = space_jacobian(screw, space_screws, theta);
    const praxis::manipulator::jacobian J_s = J_s_result.value();


    const std::size_t n = space_screws.size();
    // Cause first column is S1
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // Since it ends at i-1 i is set to 1
    for (std::size_t i = 1; i < n; ++i)
    {
        const Eigen::Matrix4d exp_i =
            screw.matrix_exponential_screw(
                -space_screws[i - 1],
                theta(static_cast<Eigen::Index>(i - 1)));
        T = T * exp_i;

        const auto J_si = screw.adjoint_map(space_screws[i], T);

    }
    return J_s;
    return praxis::unexpected(praxis::refusal::not_implemented);
}

}
