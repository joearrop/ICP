/*
 * point3D header file
 * Implements data point for Iterative closest point algorithm
 *
 * Creator: Joe Ferreira Scholtz
 * Date: 16/01/2022
 *
 */
#pragma once // include guard
#include <iostream> //std::cout etc
#include <math.h>
#include <array>
#include <vector>
#include <memory> //smart pointers

namespace ICP {

    class Point3D{
    //aliases
    using Set3 = std::vector<Point3D>;
    using Set3Pointer = std::shared_ptr<Set3>;

    public:
        std::array<double,3> _p;
    public:
        //Constructors
        Point3D(const double x,const double y,const double z){
            //populate array
            _p[0] = x; _p[1] = y; _p[2] = z;
        }
        Point3D(const int x,const int y,const int z){
            //populate array
            _p[0] = (double)x; _p[1] = (double)y; _p[2] = (double)z;
        }
        Point3D(){
            Point3D(0,0,0);
        }
        //overload + operator
        Point3D operator+(const Point3D& other){
            return Point3D(_p[0]+other._p[0],_p[1]+other._p[1],_p[2]+other._p[2]);
        }
        //overload - operator
        Point3D operator-(const Point3D& other){
            return Point3D(_p[0]-other._p[0],_p[1]-other._p[1],_p[2]-other._p[2]);
        }
        //overload * operator
        double operator*(const Point3D& other){
            return _p[0]*other._p[0]+_p[1]*other._p[1]+_p[2]*other._p[2];
        }
        //overload += operator
        Point3D& operator+=(const Point3D& other){
            _p[0] += other._p[0]; _p[1] += other._p[1];_p[2] += other._p[2];
            return *this;
        }
        //overload -= operator
        Point3D& operator-=(const Point3D& other){
            _p[0] -= other._p[0]; _p[1] -= other._p[1];_p[2] -= other._p[2];
            return *this;
        }
        //return point distance to other
        double distanceTo(const Point3D& other) const{
            return sqrt(pow(_p[0]-other._p[0],2)+pow(_p[1]-other._p[1],2)+pow(_p[2]-other._p[2],2));
        }
        //return point distance to zero
        double norm() const{
            return sqrt(pow(_p[0],2)+pow(_p[1],2)+pow(_p[2],2));
        }
        //return the closest point to other in searchSet
        static Point3D findClosestPoint(Set3Pointer searchSet, const Point3D &other){
            double minDist = -1;
            Point3D *minDistPoint;
            for(auto &aPoint: *searchSet){
                if(minDist<0){
                    minDist = other.distanceTo(aPoint);
                    minDistPoint = &aPoint;
                }
                else if(other.distanceTo(aPoint)<minDist){
                    minDist = other.distanceTo(aPoint);
                    minDistPoint = &aPoint;
                }
            }
            return *minDistPoint;
        }
        //print function
        void print() const{
            std::cout << "(" << _p[0] << ", " << _p[1] << ", " << _p[2] << ")\n";
        }

    };
    //overload << operator
    std::ostream& operator<<(std::ostream& os, const Point3D& p)
    {
        os << "(" << p._p[0] << ", " << p._p[1] << ", " << p._p[2] << ")";
        return os;
    }
}
