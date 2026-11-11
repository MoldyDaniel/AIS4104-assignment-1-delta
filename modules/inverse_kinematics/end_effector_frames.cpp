//Assignment 3
#include "inverse_kinematics/end_effector_frames.h"

namespace ais4104::manipulator {

//TASK: T.2a
praxis::expected<Eigen::Matrix4d, praxis::refusal> tool_pose_from_flange_pose(const Eigen::Matrix4d &flange_pose, const Eigen::Matrix4d &tool_offset)
{

    return flange_pose*tool_offset;
}

//TASK: T.2b
praxis::expected<Eigen::Matrix4d, praxis::refusal> flange_pose_from_tool_pose(const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &tool_pose,
                                                                              const Eigen::Matrix4d &tool_offset)
{
    const Eigen::Matrix4d inv_frame = frames.inverse(tool_offset);
    return (tool_pose * inv_frame);
}

//TASK: T.2c
praxis::expected<Eigen::Vector3d, praxis::refusal> position_from_pose(const Eigen::Matrix4d &pose)
{
    Eigen::Vector3d v(3);
    v <<    pose(0,3),
            pose(1,3),
            pose(2,3);
    return v;
}

//TASK: T.2c
praxis::expected<Eigen::Matrix3d, praxis::refusal> orientation_from_pose(const Eigen::Matrix4d &pose)
{
    Eigen::Matrix3d m(3,3);
    m <<    pose(0,0),pose(0,1),pose(0,2),
            pose(1,0),pose(1,1),pose(1,2),
            pose(2,0),pose(2,1),pose(2,2);
    return m;
}

//TASK: T.2d
praxis::expected<Eigen::VectorXd, praxis::refusal> ik_solve_flange_pose(const praxis::manipulator::kinematics &solver, const Eigen::Matrix4d &flange_pose, const Eigen::VectorXd &j0)
{
    auto def = praxis::manipulator::solver_parameters();
    if (j0.size() == 0)
    {
        return praxis::unexpected(praxis::refusal::unsupported_input);
    }
    auto ik_solved = solver.ik_solve(flange_pose, j0, def);
    return ik_solved;
}

//TASK: T.2e
praxis::expected<Eigen::VectorXd, praxis::refusal> ik_solve_pose(const praxis::manipulator::kinematics &solver, const praxis::rigid_motion::frame_ops &frames,
                                                                 const Eigen::Matrix4d &tool_pose, const Eigen::VectorXd &j0, const Eigen::Matrix4d &tool_offset)
{
    auto flange_pose = flange_pose_from_tool_pose(frames,tool_pose,tool_offset);
    //auto ik_solve = ik_solve_flange_pose(solver, flange_pose.value(), j0);
    auto solve_pose = ik_solve_flange_pose(solver, flange_pose.value() ,j0);
    return solve_pose;
}

}
