#pragma once
#include "Vector3D.h"
#include "Vector4D.h"
#include "Matrix4x4.h"

constexpr auto DEG_RAD = 0.0174532925f;

class Quaternion {
private:
	float w_; //real part
	Vector3D v_; //vector part
public:
	Quaternion() :w_(0.0f), v_() {}
	Quaternion(float w, Vector3D v) :w_(w), v_(v) {}
	Quaternion(float w, float x, float y, float z) :w_(w), v_(Vector3D(x,y,z)) {}

	Vector3D vectorPart(){return v_;}
	float realPart(){return w_;}

	Quaternion operator*(const Quaternion& q)
	{
		Quaternion t;
		
		t.w_ = w_ * q.w_ - Vector3D::dot(v_, q.v_);
		t.v_ = (Vector3D(q.v_) * w_) + (Vector3D(v_) * q.w_) + Vector3D::cross(v_, q.v_);

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
		v_ = Vector3D(0, 0, 0);
	}

	float magnitude()
	{
		float vmag = v_.magnitude();
		vmag *= vmag;
		vmag += w_ * w_;
		return sqrtf(vmag);
	}


	void setRotationAroundAxis(float theta, const Vector3D& axis) //RADIANS
	{
		w_ = cosf(0.5f * theta);
		v_.x_ = sinf(0.5f * theta) * axis.x_;
		v_.y_ = sinf(0.5f * theta) * axis.y_;
		v_.z_ = sinf(0.5f * theta) * axis.z_;
	}

	static void rotatePointAroundVector(Vector3D& point, float theta, const Vector3D& axis) //RADIANS
	{
		Quaternion q,qbar;

		q.w_ = cosf(0.5f * theta);
		q.v_.x_ = sinf(0.5f * theta) * axis.x_;
		q.v_.y_ = sinf(0.5f * theta) * axis.y_;
		q.v_.z_ = sinf(0.5f * theta) * axis.z_;

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
		float mag;
		mag = magnitude();
		w_ *= (1/mag);
		v_ = v_*(1/mag);
	}

	Matrix4x4 toMatrix()
	{
		Matrix4x4 m;
		float qw = w_;
		float qx = v_.x_;
		float qy = v_.y_;
		float qz = v_.z_;

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
		float t;
	
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
