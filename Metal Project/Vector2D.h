//
//  Vector2D.h
//  Metal Project
//
//  Created by Oliver Homer on 04/11/2026.



#pragma once
#pragma once
#include <math.h>
#include <iostream>

class Vector2D {
public:

    float x_;
    float y_;
    
    Vector2D() :x_(0), y_(0)
    {}
    Vector2D(float x, float y) :x_(x), y_(y)
    {}
    Vector2D(const Vector2D& vector) :x_(vector.x_), y_(vector.y_)
    {}

    Vector2D operator+(const Vector2D& vector)
    {
        Vector2D sum;
        sum.x_ = vector.x_ + x_;
        sum.y_ = vector.y_ + y_;
        return sum;
    }

    Vector2D operator-(const Vector2D& vector)
    {
        Vector2D diff;
        diff.x_ = x_ - vector.x_;
        diff.y_ = y_ - vector.y_;
        return diff;
    }

    Vector2D operator*(const float& f)
    {
        Vector2D mult;
        mult.x_ = x_ * f;
        mult.y_ = y_ * f;
        return mult;
    }

    friend std::ostream& operator<< (std::ostream& os, const Vector2D& vector)
    {
        os << "(" << vector.x_ << "," << vector.y_ << ")";
        return os;
    }

    static Vector2D lerp(const Vector2D& start, const Vector2D& end, float delta)
    {
        Vector2D v;
        v.x_ = start.x_ * (1.0f - delta) + end.x_ * delta;
        v.y_ = start.y_ * (1.0f - delta) + end.y_ * delta;
        
        return v;
    }

    float magnitude()
    {
        return sqrtf(x_ * x_ + y_ * y_ );
    }

    void normalize()
    {
        float mag = magnitude();
        x_ /= mag;
        y_ /= mag;
    }

    void cross(Vector2D v1, Vector2D v2)
    {
        this->x_ = v1.x_ * v2.y_ - v1.y_ * v2.x_;
        this->y_ = v1.y_ * v2.x_ - v1.x_ * v2.y_;
        
    }

    static float dot(Vector2D v1, Vector2D v2)
    {
        float d = v1.x_ * v2.x_ + v1.y_ * v2.y_;
        return d;
    }
};
