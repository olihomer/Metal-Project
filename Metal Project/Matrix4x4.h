#pragma once
#include <memory>
#include "Vector3D.h"
#include "Vector4D.h"


constexpr float TWO_PI = 6.28318530718f;

// LEFT HANDED COORDINATE SYSTEM: POSITIVE X = RIGHT, POSITIVE Y = UP, POSITIVE Z = AWAY
// ROTATIONS = ANTICLOCKWISE AS LOOKING ALONG POSITIVE AXIS


class Matrix4x4{
public:
	float mat_[4][4] = {};

	void setIdentity()
	{
		::memset(mat_, 0, sizeof(float) * 16);
		mat_[0][0] = 1;
		mat_[1][1] = 1;
		mat_[2][2] = 1;
		mat_[3][3] = 1;
	}

	void setTranslation(const Vector3D& translation)
	{
		mat_[3][0] = translation.x_;
		mat_[3][1] = translation.y_;
		mat_[3][2] = translation.z_;
	}

	void setScale(const Vector3D& scale)
	{
		mat_[0][0] = scale.x_;
		mat_[1][1] = scale.y_;
		mat_[2][2] = scale.z_;
	}

	void setRotX(float psi)
	{
		mat_[1][1] = cosf(psi);
		mat_[1][2] = sinf(psi);
		mat_[2][1] = -sinf(psi);
		mat_[2][2] = cosf(psi);
	}

	void setRotY(float theta)
	{
		mat_[0][0] = cosf(theta);
		mat_[0][2] = -sinf(theta);
		mat_[2][0] = sinf(theta);
		mat_[2][2] = cosf(theta);
	}

	void setRotZ(float phi)
	{
		mat_[0][0] = cosf(phi);
		mat_[0][1] = sinf(phi);
		mat_[1][0] = -sinf(phi);
		mat_[1][1] = cosf(phi);
	}

	void setMatrix(Matrix4x4 matrix)
	{
		::memcpy(&mat_, &matrix, sizeof(float) * 16);
	}


	void operator *=(const Matrix4x4& matrix)
	{
		Matrix4x4 out;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
			{
				out.mat_[i][j] =
					mat_[i][0] * matrix.mat_[0][j] + mat_[i][1] * matrix.mat_[1][j] +
					mat_[i][2] * matrix.mat_[2][j] + mat_[i][3] * matrix.mat_[3][j];
			}		
		::memcpy(mat_, out.mat_, sizeof(float) * 16);
	}

	void setOrthoLH(float width, float height, float near_plane, float far_plane)
	{
		setIdentity();
		mat_[0][0] = 2.0f / width;
		mat_[1][1] = 2.0f / height;;
		mat_[2][2] = 1.0f / (far_plane - near_plane);
		mat_[3][2] = -(near_plane / (far_plane - near_plane));
	}
	

	void setPerspLH(int fov, float aspect, float near_plane,float far_plane)
	{
		float theta = (float)fov / 2.0f * 0.0174532f;
		
		setIdentity();
		mat_[0][0] = 1 / tanf(theta);
		mat_[1][1] = aspect / tanf(theta);
		mat_[2][2] = far_plane / (far_plane - near_plane);
		mat_[3][2] = -(near_plane * far_plane) / (far_plane - near_plane);
		mat_[2][3] = 1.0f;
		mat_[3][3] = 0;
	}

	Vector3D getUp()
	{
		return Vector3D(mat_[1][0], mat_[1][1], mat_[1][2]);
	}

	Vector3D getForward()
	{
		return Vector3D(mat_[2][0], mat_[2][1], mat_[2][2]);
	}

	Vector3D getRight()
	{
		return Vector3D(mat_[0][0], mat_[0][1], mat_[0][2]);
	}

	Matrix4x4()
	{
		setIdentity();
	}
	
	float getDeterminant()
	{
		Vector4D minor, v1, v2, v3;
		float det;

		v1 = Vector4D(this->mat_[0][0], this->mat_[1][0], this->mat_[2][0], this->mat_[3][0]);
		v2 = Vector4D(this->mat_[0][1], this->mat_[1][1], this->mat_[2][1], this->mat_[3][1]);
		v3 = Vector4D(this->mat_[0][2], this->mat_[1][2], this->mat_[2][2], this->mat_[3][2]);

		minor.cross(v1, v2, v3);
		det = -(this->mat_[0][3] * minor.x_ + this->mat_[1][3] * minor.y_ + this->mat_[2][3] * minor.z_ +
			this->mat_[3][3] * minor.w_);
		return det;
	}

	void invert()
	{
		int a, i, j;
		Matrix4x4 out;
		Vector4D v, vec[3];
		float det = 0.0f;

		det = this->getDeterminant();
		if (!det) return;
		for (i = 0; i < 4; i++)
		{
			for (j = 0; j < 4; j++)
			{
				if (j != i)
				{
					a = j;
					if (j > i) a = a - 1;
					vec[a].x_ = (this->mat_[j][0]);
					vec[a].y_ = (this->mat_[j][1]);
					vec[a].z_ = (this->mat_[j][2]);
					vec[a].w_ = (this->mat_[j][3]);
				}
			}
			v.cross(vec[0], vec[1], vec[2]);

			out.mat_[0][i] = (powf(-1.0f, (float)i) * v.x_ / det);
			out.mat_[1][i] = (powf(-1.0f, (float)i) * v.y_ / det);
			out.mat_[2][i] = (powf(-1.0f, (float)i) * v.z_ / det);
			out.mat_[3][i] = (powf(-1.0f, (float)i) * v.w_ / det);
		}

		this->setMatrix(out);
	}

	Vector3D getEulerAngles()
	{
		float yaw, pitch, roll;

		Vector3D axis_x = getRight();
		Vector3D axis_y = getUp();
		Vector3D axis_z = getForward();

		if (axis_z.y_ != 1 && axis_z.y_ != -1)
		{
			pitch = -asinf(axis_z.y_);
			yaw = atan2f(axis_z.x_ / cosf(pitch), axis_z.z_ / cosf(pitch));
			roll = atan2f(axis_x.y_ / cosf(pitch), axis_y.y_ / cosf(pitch));
		}
		else
		{
			roll = 0;
			if (axis_z.y_ == -1)
			{
				pitch = TWO_PI / 4;
				yaw = roll + atan2(axis_y.x_, axis_x.x_);
			}
			else
			{
				pitch = -TWO_PI / 4;
				yaw = -roll - atan2(axis_y.x_, axis_x.x_);
			}

		}

		if (pitch != pitch)pitch = 0;
		if (yaw != yaw)yaw = 0;
		if (roll != roll)roll = 0;

		return Vector3D(pitch, yaw, roll);

	}

};
