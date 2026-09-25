#include "rigid_motion/frame.h"
#include "rigid_motion/screw.h"

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace ais4104::rigid_motion {

//TASK: 1d
//ELSEWHERE: https://math.libretexts.org/Bookshelves/Applied_Mathematics/Mathematics_for_Game_Developers_(Burzynski)/04:_Matrices/4.06:_Rotation_Matrices_in_3-Dimensions
// I found it first on math.libretexts and tried that first before looking in the book for it.
//REFERENCE: Figure (3.8) page 72, MR pre-print 2019
Eigen::Matrix3d rotate_x(double radians)
{
    Eigen::Matrix3d x(3,3);
    x <<    1,0,0,
            0,std::cos(radians),-std::sin(radians),
            0,std::sin(radians),std::cos(radians);
    return x;
}

//TASK: 1e
//REFERENCE: Figure (3.8) page 72, MR pre-print 2019
Eigen::Matrix3d rotate_y(double radians)
{
    Eigen::Matrix3d y(3,3);
    y <<    std::cos(radians),0,std::sin(radians),
            0,1,0,
            -std::sin(radians),0,std::cos(radians);
    return y;
}

//TASK: 1f
//REFERENCE: Figure (3.8) page 72, MR pre-print 2019
Eigen::Matrix3d rotate_z(double radians)
{
    Eigen::Matrix3d z(3,3);
    z <<    std::cos(radians),-std::sin(radians),0,
            std::sin(radians),std::cos(radians),0,
            0,0,1;
    return z;
}

//TASK: 1g
//REFERENCE: Equation (3.16) page 65, MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z)
{

    Eigen::Matrix3d m(3,3);
    m <<    x(0),y(0),z(0),
            x(1),y(1),z(1),
            x(2),y(2),z(2);
    return m;
}

//TASK: 1h
//REFERENCE: Equation (3.51) page 82, MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians)
{
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d skew_matrix = skew_symmetric(axis);
    Eigen::Matrix3d result = I + std::sin(radians)*skew_matrix + (1-std::cos(radians))*(skew_matrix*skew_matrix);

    return result;
}

//TASK: 1i Implement the special cases for Euler ZYX and XYZ using rotate_x, rotate_y, and rotate_z
//REFERENCE:  Figure (B.1) page 577, MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_euler(const Eigen::Vector3d &e, praxis::axis_order order)
{
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    if(order == praxis::axis_order::zyx)
    {
        Eigen::Matrix3d X = rotate_x(e(0));
        Eigen::Matrix3d Y = rotate_y(e(1));
        Eigen::Matrix3d Z = rotate_z(e(2));
        return I*Z*Y*X;
    }
    else if(order == praxis::axis_order::xyz)
    {
        Eigen::Matrix3d X = rotate_x(e(0));
        Eigen::Matrix3d Y = rotate_y(e(1));
        Eigen::Matrix3d Z = rotate_z(e(2));
        return I*X*Y*Z;
    }

    const std::array<std::uint8_t, 3> axes = praxis::axis_indices(order);
    return rotation_matrix_from_axis_angle(Eigen::Vector3d::Unit(axes[0]), e[0]) * 
           rotation_matrix_from_axis_angle(Eigen::Vector3d::Unit(axes[1]), e[1]) *
           rotation_matrix_from_axis_angle(Eigen::Vector3d::Unit(axes[2]), e[2]);
}

//TASK: 1j -- Implement the special case for calculating the Euler ZYX.
//REFERENCE:  Figure (B.1) page 578, MR pre-print 2019
Eigen::Vector3d euler_from_rotation_matrix(const Eigen::Matrix3d &r, praxis::axis_order order)
{
    if(order == praxis::axis_order::zyx)
    {
        double X = std::atan2(r(1,0),r(0,0));
        double Y = std::atan2(-r(2,0),std::sqrt(r(0,0)*r(0,0)+r(1,0)*r(1,0)));
        double Z = std::atan2(r(2,1),r(2,2));

        Eigen::Vector3d v(X, Y, Z);

        return v;
    }

    const std::array<std::uint8_t, 3> axes = praxis::axis_indices(order);

    return r.eulerAngles(axes[0], axes[1], axes[2]);
}

//TASK: 2b
//REFERENCE: Equation (3.62) page 87, MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_transform(const Eigen::Matrix4d &tf)
{
    Eigen::Matrix3d m(3,3);
    m <<    tf(0,0),tf(0,1),tf(0,2),
            tf(1,0),tf(1,1),tf(1,2),
            tf(2,0),tf(2,1),tf(2,2);
    return m;
}

//TASK: 2c
//REFERENCE: Equation (3.62) page 87, MR pre-print 2019
Eigen::Matrix4d transformation_matrix_from_rotation_position(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen::Matrix4d m(4,4);
    m <<    r(0,0),r(0,1),r(0,2),p(0),
            r(1,0),r(1,1),r(1,2),p(1),
            r(2,0),r(2,1),r(2,2),p(2),
            0,0,0,1;
    return m;
}

//TASK: 2d
//REFERENCE: translation operator page 91, MR pre-print 2019
Eigen::Matrix4d transformation_matrix_from_position(const Eigen::Vector3d &p)
{
    Eigen::Matrix4d m(4,4);
    m <<    1,0,0,p(0),
            0,1,0,p(1),
            0,0,1,p(2),
            0,0,0,1;

    return m;
}

//TASK: 2e
//REFERENCE: ROT 4x4 matrix page 91, MR pre-print 2019
Eigen::Matrix4d transformation_matrix_from_rotation(const Eigen::Matrix3d &r)
{
    Eigen::Matrix4d m(4,4);
    m <<    r(0,0),r(0,1),r(0,2),0,
            r(1,0),r(1,1),r(1,2),0,
            r(2,0),r(2,1),r(2,2),0,
            0,0,0,1;
    return m;
}

//TASK: 2f
//REFERENCE: Equation (3.64) page 88, MR pre-print 2019
Eigen::Matrix4d inverse(const Eigen::Matrix4d &tf)
{
    return tf.inverse();
}

}
