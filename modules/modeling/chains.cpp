//Assignment 2
#include "modeling/chains.h"

#include <praxis/rigid_motion/angles.h>

namespace ais4104::modeling {

//TASK: 1a
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.1.
//
// Because of both of us were stuck on this problem me and Oliver worked together.
std::pair<Eigen::Matrix4d, std::vector<Eigen::Vector6d>> hardcoded_kr6r900sixx_chain(const praxis::rigid_motion::screw_ops &screw)
{
        Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
        M.block<3, 3>(0, 0) << 0, 0, 1,
                           0, 1, 0,
                          -1, 0, 0;

        M(0, 3) = 0.980;
        M(1, 3) = 0.000;
        M(2, 3) = 0.435;

        std::vector<Eigen::Vector3d> q = {
            {0.000, 0.000, 0.000},
            {0.025, 0.000, 0.400},
            {0.480, 0.000, 0.400},
            {0.515, 0.000, 0.435},
            {0.900, 0.000, 0.435},
            {0.980, 0.000, 0.435}
        };

        std::vector<Eigen::Vector3d> w = {
            { 0,  0, -1},
            { 0,  1,  0},
            { 0,  1,  0},
            {-1,  0,  0},
            { 0,  1,  0},
            {-1,  0,  0}
        };

        std::vector<Eigen::Vector6d> S;
        S.reserve(6);

        for (int i = 0; i < 6; ++i)
        {
            auto axis =
                screw.screw_axis_from_point_direction_pitch(
                    q[i], w[i], 0.0);

            if (!axis)
            {
                return {Eigen::Matrix4d::Zero(), {}};
            }

            S.push_back(axis.value());
        }

        return {M, S};
}

//TASK: 1b
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.1.
std::pair<Eigen::Matrix4d, std::vector<Eigen::Vector6d>> hardcoded_lbr_med14_r820_chain(const praxis::rigid_motion::screw_ops &screw)
{
    return {Eigen::Matrix4d::Zero(), {}};
}

//TASK: 3a
Eigen::Matrix4d hardcoded_kr6r900sixx_curved_rod_tool_offset(const praxis::rigid_motion::frame_ops &frames)
{
    return Eigen::Matrix4d::Zero();
}

//TASK: 3b
Eigen::Matrix4d hardcoded_kr6r900sixx_humerus_tool_offset(const praxis::rigid_motion::frame_ops &frames)
{
    return Eigen::Matrix4d::Zero();
}

}
