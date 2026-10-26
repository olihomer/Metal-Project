#pragma once

class Vector4D {
public:
	float x_;
	float y_;
	float z_;
	float w_;

	Vector4D() :x_(0), y_(0), z_(0), w_(0)
	{}
	Vector4D(float x, float y, float z,float w) :x_(x), y_(y), z_(z),w_(w)
	{}
	Vector4D(const Vector4D& vector) :x_(vector.x_), y_(vector.y_), z_(vector.z_), w_(vector.w_)
	{}
	Vector4D(const Vector3D& vector) :x_(vector.x_), y_(vector.y_), z_(vector.z_), w_(1.0f)
	{}

	static Vector4D lerp(const Vector4D& start, const Vector4D& end, float delta)
	{
		Vector4D v;
		v.x_ = start.x_ * (1.0f - delta) + end.x_ * delta;
		v.y_ = start.y_ * (1.0f - delta) + end.y_ * delta;
		v.z_ = start.z_ * (1.0f - delta) + end.z_ * delta;
		v.w_ = start.w_ * (1.0f - delta) + end.w_ * delta;

		return v;
	}

	void cross(Vector4D& v1, Vector4D& v2, Vector4D& v3)
	{
		this->x_ = v1.y_ * (v2.z_ * v3.w_ - v3.z_ * v2.w_) - v1.z_ * (v2.y_ * v3.w_ - v3.y_ * v2.w_) + v1.w_ * (v2.y_ * v3.z_ - v2.z_ * v3.y_);
		this->y_ = -(v1.x_ * (v2.z_ * v3.w_ - v3.z_ * v2.w_) - v1.z_ * (v2.x_ * v3.w_ - v3.x_ * v2.w_) + v1.w_ * (v2.x_ * v3.z_ - v3.x_ * v2.z_));
		this->z_ = v1.x_ * (v2.y_ * v3.w_ - v3.y_ * v2.w_) - v1.y_ * (v2.x_ * v3.w_ - v3.x_ * v2.w_) + v1.w_ * (v2.x_ * v3.y_ - v3.x_ * v2.y_);
		this->w_ = -(v1.x_ * (v2.y_ * v3.z_ - v3.y_ * v2.z_) - v1.y_ * (v2.x_ * v3.z_ - v3.x_ * v2.z_) + v1.z_ * (v2.x_ * v3.y_ - v3.x_ * v2.y_));
	}



};

