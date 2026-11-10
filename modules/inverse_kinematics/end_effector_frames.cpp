//Assignment 3
#include "inverse_kinematics/end_effector_frames.h"

namespace ais4104::manipulator {

//TASK: T.2a
praxis::expected<Eigen::Matrix4d, praxis::refusal> tool_pose_from_flange_pose(const Eigen::Matrix4d &flange_pose, const Eigen::Matrix4d &tool_offset)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: T.2b
praxis::expected<Eigen::Matrix4d, praxis::refusal> flange_pose_from_tool_pose(const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &tool_pose,
                                                                              const Eigen::Matrix4d &tool_offset)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: T.2c
praxis::expected<Eigen::Vector3d, praxis::refusal> position_from_pose(const Eigen::Matrix4d &pose)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: T.2c
praxis::expected<Eigen::Matrix3d, praxis::refusal> orientation_from_pose(const Eigen::Matrix4d &pose)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: T.2d
praxis::expected<Eigen::VectorXd, praxis::refusal> ik_solve_flange_pose(const praxis::manipulator::kinematics &solver, const Eigen::Matrix4d &flange_pose, const Eigen::VectorXd &j0)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: T.2e
praxis::expected<Eigen::VectorXd, praxis::refusal> ik_solve_pose(const praxis::manipulator::kinematics &solver, const praxis::rigid_motion::frame_ops &frames,
                                                                 const Eigen::Matrix4d &tool_pose, const Eigen::VectorXd &j0, const Eigen::Matrix4d &tool_offset)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

}
