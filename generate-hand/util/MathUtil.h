#pragma once
#include "EigenUtil.h"

class MathUtil
{
public:
	enum eAxis
	{
		eAxisX,
		eAxisY,
		eAxisZ,
		eAxisMax
	};

	static int Clamp(int val, int min, int max);
	static void Clamp(const Eigen::VectorXd& min, const Eigen::VectorXd& max, Eigen::VectorXd& out_vec);
	static double Clamp(double val, double min, double max);
	static double Saturate(double val);
	static double Lerp(double t, double val0, double val1);

	static double NormalizeAngle(double theta);

	// matrices
	static Matrix TranslateMat(const Vector& trans);
	static Matrix ScaleMat(double scale);
	static Matrix ScaleMat(const Vector& scale);
	static Matrix RotateMat(const Vector& euler); // euler angles order rot(Z) * rot(Y) * rot(X)
	static Matrix RotateMat(const Vector& axis, double theta);
	static Matrix RotateMat(const Quaternion& q);
	static Matrix CrossMat(const Vector& a);
	// inverts a transformation consisting only of rotations and translations
	static Matrix InvRigidMat(const Matrix& mat);
	static Vector GetRigidTrans(const Matrix& mat);
	static Vector InvEuler(const Vector& euler);
	static void RotMatToAxisAngle(const Matrix& mat, Vector& out_axis, double& out_theta);
	static Vector RotMatToEuler(const Matrix& mat);
	static Quaternion RotMatToQuaternion(const Matrix& mat);
	static void EulerToAxisAngle(const Vector& euler, Vector& out_axis, double& out_theta);
	static Vector AxisAngleToEuler(const Vector& axis, double theta);
	static Matrix DirToRotMat(const Vector& dir, const Vector& up);

	static void DeltaRot(const Vector& axis0, double theta0, const Vector& axis1, double theta1,
		Vector& out_axis, double& out_theta);
	static Matrix DeltaRot(const Matrix& R0, const Matrix& R1);

	static Quaternion EulerToQuaternion(const Vector& euler);
	static Vector QuaternionToEuler(const Quaternion& q);
	static Quaternion AxisAngleToQuaternion(const Vector& axis, double theta);
	static void QuaternionToAxisAngle(const Quaternion& q, Vector& out_axis, double& out_theta);
	static Matrix BuildQuaternionDiffMat(const Quaternion& q);
	static Vector CalcQuaternionVel(const Quaternion& q0, const Quaternion& q1, double dt);
	static Vector CalcQuaternionVelRel(const Quaternion& q0, const Quaternion& q1, double dt);
	static Quaternion VecToQuat(const Vector& v);
	static Vector QuatToVec(const Quaternion& q);
	static Quaternion QuatDiff(const Quaternion& q0, const Quaternion& q1);
	static double QuatDiffTheta(const Quaternion& q0, const Quaternion& q1);
	static double QuatTheta(const Quaternion& dq);
	static Quaternion VecDiffQuat(const Vector& v0, const Vector& v1);
	static Vector QuatRotVec(const Quaternion& q, const Vector& dir);
	static Quaternion MirrorQuaternion(const Quaternion& q, eAxis axis);

};

