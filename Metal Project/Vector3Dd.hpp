//
//  Vector3Dd.hpp
//  Metal Project
//
//  Created by Oliver Homer on 10/11/2026.
//

#pragma once
#include <iostream>
#include <cmath>

class Vector3D;

    class Vector3Dd {
    public:

        double x_;
        double y_;
        double z_;

        Vector3Dd() :x_(0), y_(0), z_(0)
        {}
        Vector3Dd(double x, double y, double z) :x_(x), y_(y), z_(z)
        {}
        Vector3Dd(const Vector3D& v);


        static double dot(const Vector3Dd& u, const Vector3Dd& v)
        {
            return u.x_ * v.x_ + u.y_ * v.y_ + u.z_ * v.z_;
        }

        static Vector3Dd cross(const Vector3Dd& v1, const Vector3Dd& v2)
        {
            Vector3Dd temp;
            temp.x_ = v1.y_ * v2.z_ - v1.z_ * v2.y_;
            temp.y_ = v1.z_ * v2.x_ - v1.x_ * v2.z_;
            temp.z_ = v1.x_ * v2.y_ - v1.y_ * v2.x_;
            return temp;
        }

        Vector3Dd operator+(const Vector3Dd& vector)
        {
            Vector3Dd sum;
            sum.x_ = vector.x_ + x_;
            sum.y_ = vector.y_ + y_;
            sum.z_ = vector.z_ + z_;
            return sum;
        }

        Vector3Dd operator-(const Vector3Dd& vector)
        {
            Vector3Dd diff;
            diff.x_ = x_ - vector.x_;
            diff.y_ = y_ - vector.y_;
            diff.z_ = z_ - vector.z_;
            return diff;
        }

        Vector3Dd operator*(const double& f)
        {
            Vector3Dd mult;
            mult.x_ = x_ * f;
            mult.y_ = y_ * f;
            mult.z_ = z_ * f;
            return mult;
        }

        friend std::ostream& operator<< (std::ostream& os, const Vector3Dd& vector)
        {
            os << "(" << vector.x_ << "," << vector.y_ << "," << vector.z_ << ")";
            return os;
        }

        static Vector3Dd lerp(const Vector3Dd& start, const Vector3Dd& end, double delta)
        {
            Vector3Dd v;
            v.x_ = start.x_ * (1.0f - delta) + end.x_ * delta;
            v.y_ = start.y_ * (1.0f - delta) + end.y_ * delta;
            v.z_ = start.z_ * (1.0f - delta) + end.z_ * delta;

            return v;
        }

        double magnitude()
        {
            return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
        }

        void normalize()
        {
            double mag = magnitude();
            if(mag!=0.0f)
            {
                x_ /= mag;
                y_ /= mag;
                z_ /= mag;
            }
        }

        void print(std::string comment)
        {
            std::cout << comment << ": " << x_ << "," << y_ << "," << z_ << std::endl;
        }
    };


