/*
 * point3D header file
 * Implements data point transformation for Iterative closest point algorithm
 *
 * Creator: Joe Ferreira Scholtz
 * Date: 16/01/2022
 *
 */
#pragma once // include guard
#include "Point3D.hpp"
#include <iostream> //std::cout etc
#include <array>

namespace ICP {
    using Set3 = std::vector<Point3D>;
    using Set3Pointer = std::shared_ptr<Set3>;

    class Transform3D{
    public:
        Point3D _translation;
        std::array<Point3D,3> _rotation; //rotation matrix rows
    public:
        //Constructors
        Transform3D(const Point3D translation, const double roll, const double pitch, const double yaw) : _translation(translation){
            //populate rotation matrix
            _rotation[0] = Point3D(cos(yaw)*cos(pitch),
                                   cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll),
                                   cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll));
            _rotation[1] = Point3D(sin(yaw)*cos(pitch),
                                   sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll),
                                   sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll));
            _rotation[2] = Point3D(-sin(pitch),cos(pitch)*sin(roll),cos(pitch)*cos(roll));
        }
        //overloading operator *
        Point3D operator*(const Point3D& point){
            //return point transformed by this transform
            return Point3D(_rotation[0]*point + _translation._p[0],
                           _rotation[1]*point + _translation._p[1],
                           _rotation[2]*point + _translation._p[2]);
        }
        //return null transformation (transformation from a point to itself)
        static Transform3D nullTransform(){
            return Transform3D({0,0,0},0,0,0);;
        }
    };
    //overload << operator
    std::ostream& operator<<(std::ostream& os, const Transform3D& t)
    {
        os << "_translation: " << t._translation << "\n";
        os << "_rotation[0]: " << t._rotation[0] << "\n";
        os << "_rotation[1]: " << t._rotation[1] << "\n";
        os << "_rotation[2]: " << t._rotation[2];

        return os;
    }
}
