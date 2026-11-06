#include "Camera.h"
#include <iostream>

#define wUP Vector3D(0,1,0)
#define wX Vector3D(1,0,0)
#define wY Vector3D(0,1,0)
#define wZ Vector3D(0,0,1)

void Camera::setPosition(Vector3D pos)
{
	position_ = pos;
}

void Camera::setAspect(float aspect)
{
	aspect_ = aspect;
}

void Camera::lookAt(Vector3D target, Vector3D up)
{
	cameraLocalMatrix_.setIdentity();

	//SET AXES
		
	Vector3D camera_z = (target - position_);
	camera_z.normalize();
	Vector3D camera_y = up;
	camera_y.normalize();
	Vector3D camera_x;
	camera_x=Vector3D::cross(camera_y, camera_z);
	camera_x.normalize();
	camera_y=Vector3D::cross(camera_z, camera_x);
	camera_y.normalize();

	//COPY TO STATE MATRIX

	cameraLocalMatrix_.mat_[0][0] = camera_x.x_;
	cameraLocalMatrix_.mat_[0][1] = camera_x.y_;
	cameraLocalMatrix_.mat_[0][2] = camera_x.z_;

	cameraLocalMatrix_.mat_[1][0] = camera_y.x_;
	cameraLocalMatrix_.mat_[1][1] = camera_y.y_;
	cameraLocalMatrix_.mat_[1][2] = camera_y.z_;

	cameraLocalMatrix_.mat_[2][0] = camera_z.x_;
	cameraLocalMatrix_.mat_[2][1] = camera_z.y_;
	cameraLocalMatrix_.mat_[2][2] = camera_z.z_;

	//COPY TO QUATERNION

	cameraQuaternion_ = Quaternion(cameraLocalMatrix_);

}


void Camera::Translate(float delta, Vector3D direction)
{
	position_.x_ += delta * direction.x_;
	position_.y_ += delta * direction.y_;
	position_.z_ += delta * direction.z_;

}

void Camera::Pitch(float theta)
{
	Quaternion q;
	q.setRotationAroundAxis(theta, getRight());
	cameraQuaternion_ = q * cameraQuaternion_;
	cameraLocalMatrix_ = cameraQuaternion_.toMatrix();
}

void Camera::Yaw(float theta)
{
	Quaternion q;
	q.setRotationAroundAxis(theta, wUP);
	cameraQuaternion_ = q * cameraQuaternion_;
	cameraLocalMatrix_ = cameraQuaternion_.toMatrix();
}

void Camera::Roll(float theta)
{
	Quaternion q;
	q.setRotationAroundAxis(theta, getForward());
	cameraQuaternion_ = q * cameraQuaternion_;
	cameraLocalMatrix_ = cameraQuaternion_.toMatrix();

}

void Camera::debugPrintState()
{
    ;
}

Matrix4x4 Camera::getPerspectiveTransform()
{
	Matrix4x4 persp;
	float theta = (float)fov_ / 2.0f * 0.0174532f;

	persp.setIdentity();
	persp.mat_[0][0] = 1 / tanf(theta);
	persp.mat_[1][1] = aspect_ / tanf(theta);
	persp.mat_[2][2] = farPlane_ / (farPlane_ - nearPlane_);
    persp.mat_[3][2] = -(nearPlane_ * farPlane_) / (farPlane_ - nearPlane_);
	persp.mat_[2][3] = 1.0f;
	persp.mat_[3][3] = 0;
	return persp;
}

Matrix4x4 Camera::getCameraTransform()
{
	Matrix4x4 cam;
	
	cam = cameraLocalMatrix_;
	cam.setTranslation(position_);
	cam.invert();
		
	return cam;
}

Vector3D Camera::getPosition()
{
	return position_;
}

Vector3D Camera::getUp()
{
	return cameraLocalMatrix_.getUp();
}

Vector3D Camera::getRight()
{
	return cameraLocalMatrix_.getRight();
}

Vector3D Camera::getForward()
{
	return cameraLocalMatrix_.getForward();
}

Camera::Camera(Vector3D position, Vector3D lookat, Vector3D up, float aspect_ratio, float near_plane, float far_plane, int fov)
{
	Matrix4x4 temp;

	position_ = position;

	this->lookAt(lookat, up);
		
	aspect_ = aspect_ratio;
	nearPlane_ = near_plane;
	farPlane_ = far_plane;
	fov_ = fov;

	this->debugPrintState();
}

