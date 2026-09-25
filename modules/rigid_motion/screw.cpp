#include "rigid_motion/frame.h"
#include "rigid_motion/screw.h"

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <numbers>

namespace ais4104::rigid_motion {

//TASK: 1b
//ELSEWHERE:https://www.quantstart.com/articles/Eigen-Library-for-Matrix-Algebra-in-C/
//REFERENCE: Equation (3.30) page 75, MR pre-print 2019
Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d m(3,3);
    m << 0,-v(2),v(1),
            v(2),0,-v(0),
           -v(1),v(0),0;
    m = -m;
    m.transposeInPlace();

    return m;
}

//TASK: 1c
//ELSEWHERE:https://www.quantstart.com/articles/Eigen-Library-for-Matrix-Algebra-in-C/
//REFERENCE:REFERENCE: Equation (3.30) page 75, MR pre-print 2019
Eigen::Vector3d from_skew_symmetric(const Eigen::Matrix3d &m)
{
    Eigen::Vector3d v(-m(1,2),m(0,2),-m(0,1));
    return v;
}

//TASK: 3b
//REFERENCE: Figure (3.19) page 101, MR pre-print 2019
praxis::expected<Eigen::Vector6d, praxis::refusal> screw_axis_from_point_direction_pitch(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    Eigen::Vector3d v = -s.cross(q)+h*s;
    Eigen::Vector6d result;
    result << s(0), s(1),s(2), v(0),v(1),v(2);
    return result;
}

//TASK: 3c
//ELSEWHERE: https://stackoverflow.com/questions/17333/how-do-you-compare-float-and-double-while-accounting-for-precision-loss
//REFERENCE: Figure (3.19) Definition (3.24) page 101-102, MR pre-print 2019
Eigen::Vector6d screw_axis_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    const double tol = 1e-9;
    double norm_w = w.norm();
    double norm_v = v.norm();

    if (norm_w > tol)
    {
        Eigen::Vector6d result;
        result << w/norm_w, v/norm_w;
        return result;
    }
    else
    {
        Eigen::Vector6d result;
        result << 0,0,0, v/norm_v;
        return result;
    }
}

//TASK: 3d
//REFERENCE: Figure (3.19) page 101, MR pre-print 2019
Eigen::Vector6d twist_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    Eigen::Vector6d result;
    result << w(0), w(1),w(2), v(0),v(1),v(2);
    return result;
}

//TASK: 3e
//REFERENCE: Figure (3.19) page 101, MR pre-print 2019
praxis::expected<Eigen::Vector6d, praxis::refusal> twist_from_screw(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity)
{
    Eigen::Vector3d w = s*angular_velocity;
    Eigen::Vector3d v = -w.cross(q)+h*w;
    Eigen::Vector6d result;
    result << w(0), w(1),w(2), v(0),v(1),v(2);
    return result;
}

//TASK: 3f
//REFERENCE: Definition (3.20) page 98, MR pre-print 2019
praxis::expected<Eigen::Matrix6d, praxis::refusal> adjoint_matrix_from_rotation_position(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen::Matrix6d result;
    result <<   r, Eigen::Matrix3d::Zero(),
                skew_symmetric(p)*r,r;
    return result;
}

//TASK: 3g
//REFERENCE: Equation (3.62) page 87, MR pre-print 2019
praxis::expected<Eigen::Matrix6d, praxis::refusal> adjoint_matrix_from_transform(const Eigen::Matrix4d &tf)
{
    Eigen::Matrix3d r = rotation_matrix_from_transform(tf);
    Eigen::Vector3d p(tf(0,3),tf(1,3),tf(2,3));

    return adjoint_matrix_from_rotation_position(r,p);
}

//TASK: 3h
//REFERENCE: Equation (3.62) page 87, Definition (3.20) page 98, MR pre-print 2019
praxis::expected<Eigen::Vector6d, praxis::refusal> adjoint_map(const Eigen::Vector6d &t, const Eigen::Matrix4d &tf)
{
    Eigen::Matrix3d r = rotation_matrix_from_transform(tf);
    Eigen::Vector3d p(tf(0,3),tf(1,3),tf(2,3));
    Eigen::Matrix6d AdT = adjoint_matrix_from_rotation_position(r,p).value();
    Eigen::Vector6d result = AdT*t;
    return result;
}

//TASK: 3i
//REFERENCE:  Equation (3.71) page 96, Equation (3.74) page 97, MR pre-print 2019
Eigen::Matrix4d twist_matrix_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    Eigen::Matrix3d W = skew_symmetric(w);
    Eigen::Matrix4d result;
    result <<   W, v,
                0,0;
    return result;
}

//TASK: 3j
//REFERENCE: Figure (3.19) page 101, Equation (3.71) page 96, Equation (3.74) page 97, MR pre-print 2019
Eigen::Matrix4d twist_matrix_from_twist(const Eigen::Vector6d &t)
{
    Eigen::Vector3d w(t(0),t(1),t(2));
    Eigen::Vector3d v(t(3),t(4),t(5));
    Eigen::Matrix3d W = skew_symmetric(w);
    Eigen::Matrix4d result;
    result <<   W, v,
                0,0;
    return result;
}

//TASK: 3k
//REFERENCE: Definition (3.7) page 75, MR pre-print 2019
Eigen::Matrix3d matrix_exponential_so3(const Eigen::Vector3d &w, double theta_radians)
{
    return rotation_matrix_from_axis_angle(w,theta_radians);
}

//TASK: 3l
//REFERENCE: Proposition (3.15) page 88, Equation (3.71) page 96, MR pre-print 2019
Eigen::Matrix4d matrix_exponential_se3(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta_radians)
{
    Eigen::Matrix3d W = skew_symmetric(w);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d G_O = I*theta_radians + (1-std::cos(theta_radians))*W + (theta_radians-std::sin(theta_radians))*(W*W);

    const double tol = 1e-9;
    double norm_w = w.norm();
    double norm_v = v.norm();

    if (norm_w > 1-tol && norm_w < 1+tol)
    {
        Eigen::Matrix4d result;
        result <<   std::exp(W)*theta_radians, G_O*v,
                    0,1;
        return result;
    }
    else
    {
        Eigen::Matrix4d result;
        result <<   I, v*theta_radians,
                    0,1;
        return result;
    }
}

//TASK: 3m
//REFERENCE:
Eigen::Matrix4d matrix_exponential_screw(const Eigen::Vector6d &s, double theta_radians)
{
    return Eigen::Matrix4d::Zero();
}

//TASK: 3n
//REFERENCE:
praxis::expected<std::pair<Eigen::Vector3d, double>, praxis::refusal> matrix_logarithm_so3(const Eigen::Matrix3d &r)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 3o
//REFERENCE:
praxis::expected<std::pair<Eigen::Vector6d, double>, praxis::refusal> matrix_logarithm_se3_rp(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 3p
//REFERENCE:
praxis::expected<std::pair<Eigen::Vector6d, double>, praxis::refusal> matrix_logarithm_se3(const Eigen::Matrix4d &tf)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

}
