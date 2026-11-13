#pragma once
#include "Vector3D.h"
#include "Vector3Dd.hpp"
#include "Vector4D.h"
#include "Matrix4x4.h"
#include "Matrix3x3d.hpp"

constexpr auto DEG_RAD = 0.0174532925f;

class Quaternion {
private:
	double w_; //real part
	Vector3Dd v_; //vector part
public:
	Quaternion() :w_(1.0f), v_() {}
	Quaternion(double w, Vector3Dd v) :w_(w), v_(v) {}
	Quaternion(double w, double x, double y, double z) :w_(w), v_(Vector3Dd(x,y,z)) {}

	Vector3Dd vectorPart(){return v_;}
	double realPart(){return w_;}

	Quaternion operator*(const Quaternion& q)
	{
		Quaternion t;
		
		t.w_ = w_ * q.w_ - Vector3Dd::dot(v_, q.v_);
		t.v_ = (Vector3Dd(q.v_) * w_) + (Vector3Dd(v_) * q.w_) + Vector3Dd::cross(v_, q.v_);

		return t;
	}

	Quaternion operator+(const Quaternion& q)
	{
		Quaternion t;
		t.w_ = w_ + q.w_;
		t.v_ = v_ + q.v_;
		return t;
	}


	void setIdentity()
	{
		w_ = 1.0f;
		v_ = Vector3Dd(0, 0, 0);
	}

	double magnitude()
	{
		double vmag = v_.magnitude();
		vmag *= vmag;
		vmag += w_ * w_;
		return sqrtf(vmag);
	}


	void setRotationAroundAxis(double theta, const Vector3D& axis) //RADIANS
	{
		w_ = cos(0.5 * theta);
		v_.x_ = sin(0.5 * theta) * axis.x_;
		v_.y_ = sin(0.5 * theta) * axis.y_;
		v_.z_ = sin(0.5 * theta) * axis.z_;
	}
    
    void setRotationAroundAxis(double theta, const Vector3Dd& axis) //RADIANS
    {
        w_ = cos(0.5 * theta);
        v_.x_ = sin(0.5 * theta) * axis.x_;
        v_.y_ = sin(0.5 * theta) * axis.y_;
        v_.z_ = sin(0.5 * theta) * axis.z_;
    }

	static void rotatePointAroundVector(Vector3Dd& point, double theta, const Vector3D& axis) //RADIANS
	{
		Quaternion q,qbar;

		q.w_ = cos(0.5 * theta);
		q.v_.x_ = sin(0.5 * theta) * axis.x_;
		q.v_.y_ = sin(0.5 * theta) * axis.y_;
		q.v_.z_ = sin(0.5 * theta) * axis.z_;

		qbar = q.getCongugate();

		Quaternion v(0, point);

		v = q * v * qbar;

		point = v.vectorPart();
	}

	Quaternion getCongugate()
	{
		Quaternion q;
		q.w_ = w_;
		q.v_ = v_*-1.0f;
		return q;
	}

	void normalise()
	{
		double mag;
		mag = magnitude();
		w_ *= (1/mag);
		v_ = v_*(1/mag);
	}

	Matrix4x4 toMatrix()
	{
		Matrix4x4 m;
		double qw = w_;
		double qx = v_.x_;
		double qy = v_.y_;
		double qz = v_.z_;

		m.mat_[0][0] = 1 - 2 * qy * qy - 2 * qz * qz;
		m.mat_[0][1] = 2 * qx * qy + 2 * qz * qw;
		m.mat_[0][2] = 2 * qx * qz - 2 * qy * qw;
		
		m.mat_[1][0] = 2 * qx * qy - 2 * qz * qw;
		m.mat_[1][1] = 1 - 2 * qx * qx - 2 * qz * qz;
		m.mat_[1][2] = 2 * qy * qz + 2 * qx * qw;

		m.mat_[2][0] = 2 * qx * qz + 2 * qy * qw;
		m.mat_[2][1] = 2 * qy * qz - 2 * qx * qw;
		m.mat_[2][2] = 1 - 2 * qx * qx - 2 * qy * qy;
		
		return m;
	}
    
    Matrix3x3d toMatrix3x3d()
    {
        Matrix3x3d m;
        double qw = w_;
        double qx = v_.x_;
        double qy = v_.y_;
        double qz = v_.z_;

        m.mat_[0][0] = 1 - 2 * qy * qy - 2 * qz * qz;
        m.mat_[0][1] = 2 * qx * qy + 2 * qz * qw;
        m.mat_[0][2] = 2 * qx * qz - 2 * qy * qw;
        
        m.mat_[1][0] = 2 * qx * qy - 2 * qz * qw;
        m.mat_[1][1] = 1 - 2 * qx * qx - 2 * qz * qz;
        m.mat_[1][2] = 2 * qy * qz + 2 * qx * qw;

        m.mat_[2][0] = 2 * qx * qz + 2 * qy * qw;
        m.mat_[2][1] = 2 * qy * qz - 2 * qx * qw;
        m.mat_[2][2] = 1 - 2 * qx * qx - 2 * qy * qy;
        
        return m;
    }

	Quaternion(Matrix4x4 m)
	{
		double t;
	
		if (m.mat_[2][2] < 0)
		{
			if (m.mat_[0][0] > m.mat_[1][1])
			{
				t = 1 + m.mat_[0][0] - m.mat_[1][1] - m.mat_[2][2];
				w_ = m.mat_[1][2] - m.mat_[2][1];
				v_ = Vector3D(t, m.mat_[0][1] + m.mat_[1][0], m.mat_[2][0] + m.mat_[0][2]);
			}
			else
			{
				t = 1 - m.mat_[0][0] + m.mat_[1][1] - m.mat_[2][2];
				w_ = m.mat_[2][0] - m.mat_[0][2];
				v_ = Vector3D(m.mat_[0][1] + m.mat_[1][0], t,m.mat_[1][2] + m.mat_[2][1]);
			}
		}
		else
		{
			if (m.mat_[0][0] < -m.mat_[1][1])
			{
				t = 1 - m.mat_[0][0] - m.mat_[1][1] + m.mat_[2][2];
				w_ = m.mat_[0][1] - m.mat_[1][0];
				v_ = Vector3D(m.mat_[2][0] + m.mat_[0][2], m.mat_[1][2] + m.mat_[2][1],t);
			}
			else
			{
				t = 1 + m.mat_[0][0] + m.mat_[1][1] + m.mat_[2][2];
				w_ = t;
				v_ = Vector3D(m.mat_[1][2] - m.mat_[2][1], m.mat_[2][0] - m.mat_[0][2], m.mat_[0][1] - m.mat_[1][0]);
			}
		}
		w_ *= (0.5f / sqrtf(t));
		v_ = v_ * (0.5f / sqrtf(t));
	}
};
