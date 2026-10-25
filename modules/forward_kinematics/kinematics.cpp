//Assignment 2
#include "forward_kinematics/kinematics.h"

#include <span>
#include <vector>

namespace ais4104::manipulator {

//TASK: 2a
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.1, eq. (4.14).
praxis::expected<Eigen::Matrix4d, praxis::refusal> forward_kinematics(const praxis::rigid_motion::screw_ops &screw, const Eigen::Matrix4d &m,
                                                                      std::span<const Eigen::Vector6d> space_screws, const Eigen::VectorXd &theta)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // Since its unknown how long it can be
    const std::size_t n = space_screws.size();
    for (std::size_t i = 0; i < n; ++i)
    {
        const Eigen::Matrix4d exp_i =
            screw.matrix_exponential_screw(
                space_screws[i],
                // Compiler told me to use static cast however i dont know what it does.
                theta(static_cast<Eigen::Index>(i)));

        T = T * exp_i;
    }

    T = T * m;
    return T;
}

//TASK: 2b
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.3, B_i = [Ad_{M^-1}] S_i below eq. (4.16), p. 147.
praxis::expected<std::vector<Eigen::Vector6d>, praxis::refusal> body_screws_from_space(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                                                                       const Eigen::Matrix4d &m, std::span<const Eigen::Vector6d> space_screws)
{
    const Eigen::Matrix4d m_inv = m.inverse();

    std::vector<Eigen::Vector6d> body_screws;
    body_screws.reserve(space_screws.size());

    for (std::size_t i = 0; i < space_screws.size(); ++i)
    {
        const auto B_i = screw.adjoint_map(space_screws[i], m_inv);
        body_screws.push_back(B_i.value());
    }

    return body_screws;
}

//TASK: 2c
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.3, eq. (4.16), p. 147.
praxis::expected<Eigen::Matrix4d, praxis::refusal> body_forward_kinematics(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                                                           const Eigen::Matrix4d &m, std::span<const Eigen::Vector6d> space_screws, const Eigen::VectorXd &theta)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

}
