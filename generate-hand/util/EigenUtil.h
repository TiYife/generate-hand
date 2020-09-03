#pragma once
#include "Eigen/Dense"
#include "leapmotion/leapconnection.h"

using Vector = Eigen::Vector4d;
using Vector3 = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;
using Matrix3 = Eigen::Matrix3d;
using Matrix = Eigen::Matrix4d;

class EigenUtil
{
public:
	static Vector toVec(LEAP_VECTOR v);
	static Quaternion toQuat(LEAP_QUATERNION q);
	
};

