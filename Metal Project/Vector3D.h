#pragma once
#include <iostream>

	class Vector3D {
	public:

		float x_;
		float y_;
		float z_;

		Vector3D() :x_(0), y_(0), z_(0)
		{}
		Vector3D(float x, float y, float z) :x_(x), y_(y), z_(z)
		{}


		static float dot(const Vector3D& u, const Vector3D& v)
		{
			return u.x_ * v.x_ + u.y_ * v.y_ + u.z_ * v.z_;
		}

		static Vector3D cross(const Vector3D& v1, const Vector3D& v2)
		{
			Vector3D temp;
			temp.x_ = v1.y_ * v2.z_ - v1.z_ * v2.y_;
			temp.y_ = v1.z_ * v2.x_ - v1.x_ * v2.z_;
			temp.z_ = v1.x_ * v2.y_ - v1.y_ * v2.x_;
			return temp;
		}

		Vector3D operator+(const Vector3D& vector)
		{
			Vector3D sum;
			sum.x_ = vector.x_ + x_;
			sum.y_ = vector.y_ + y_;
			sum.z_ = vector.z_ + z_;
			return sum;
		}

		Vector3D operator-(const Vector3D& vector)
		{
			Vector3D diff;
			diff.x_ = x_ - vector.x_;
			diff.y_ = y_ - vector.y_;
			diff.z_ = z_ - vector.z_;
			return diff;
		}

		Vector3D operator*(const float& f)
		{
			Vector3D mult;
			mult.x_ = x_ * f;
			mult.y_ = y_ * f;
			mult.z_ = z_ * f;
			return mult;
		}

		friend std::ostream& operator<< (std::ostream& os, const Vector3D& vector)
		{
			os << "(" << vector.x_ << "," << vector.y_ << "," << vector.z_ << ")";
			return os;
		}

		static Vector3D lerp(const Vector3D& start, const Vector3D& end, float delta)
		{
			Vector3D v;
			v.x_ = start.x_ * (1.0f - delta) + end.x_ * delta;
			v.y_ = start.y_ * (1.0f - delta) + end.y_ * delta;
			v.z_ = start.z_ * (1.0f - delta) + end.z_ * delta;

			return v;
		}

		float magnitude()
		{
			return sqrtf(x_ * x_ + y_ * y_ + z_ * z_);
		}

		void normalize()
		{
			float mag = magnitude();
			x_ /= mag;
			y_ /= mag;
			z_ /= mag;
		}

	};


