#include "MathUtil.h"

int MathUtil::Clamp(int val, int min, int max)
{
	return std::max(min, std::min(val, max));
}

void MathUtil::Clamp(const Eigen::VectorXd& min, const Eigen::VectorXd& max, Eigen::VectorXd& out_vec)
{
	out_vec = out_vec.cwiseMin(max).cwiseMax(min);
}

double MathUtil::Clamp(double val, double min, double max)
{
	return std::max(min, std::min(val, max));
}

double MathUtil::Saturate(double val)
{
	return Clamp(val, 0.0, 1.0);
}

double MathUtil::Lerp(double t, double val0, double val1)
{
	return (1 - t) * val0 + t * val1;
}

double MathUtil::NormalizeAngle(double theta)
{
	// normalizes theta to be between [-pi, pi]
	double norm_theta = fmod(theta, 2 * M_PI);
	if (norm_theta > M_PI)
	{
		norm_theta = -2 * M_PI + norm_theta;
	}
	else if (norm_theta < -M_PI)
	{
		norm_theta = 2 * M_PI + norm_theta;
	}
	return norm_theta;
}

Matrix MathUtil::TranslateMat(const Vector& trans)
{
	Matrix mat = Matrix::Identity();
	mat(0, 3) = trans[0];
	mat(1, 3) = trans[1];
	mat(2, 3) = trans[2];
	return mat;
}

Matrix MathUtil::ScaleMat(double scale)
{
	return ScaleMat(Vector::Ones() * scale);
}

Matrix MathUtil::ScaleMat(const Vector& scale)
{
	Matrix mat = Matrix::Identity();
	mat(0, 0) = scale[0];
	mat(1, 1) = scale[1];
	mat(2, 2) = scale[2];
	return mat;
}

Matrix MathUtil::RotateMat(const Vector& euler)
{
	double x = euler[0];
	double y = euler[1];
	double z = euler[2];

	double x_s = std::sin(x);
	double x_c = std::cos(x);
	double y_s = std::sin(y);
	double y_c = std::cos(y);
	double z_s = std::sin(z);
	double z_c = std::cos(z);

	Matrix mat = Matrix::Identity();
	mat(0, 0) = y_c * z_c;
	mat(1, 0) = y_c * z_s;
	mat(2, 0) = -y_s;

	mat(0, 1) = x_s * y_s * z_c - x_c * z_s;
	mat(1, 1) = x_s * y_s * z_s + x_c * z_c;
	mat(2, 1) = x_s * y_c;

	mat(0, 2) = x_c * y_s * z_c + x_s * z_s;
	mat(1, 2) = x_c * y_s * z_s - x_s * z_c;
	mat(2, 2) = x_c * y_c;

	return mat;
}

Matrix MathUtil::RotateMat(const Vector& axis, double theta)
{
	assert(std::abs(axis.squaredNorm() - 1) < 0.0001);

	double c = std::cos(theta);
	double s = std::sin(theta);
	double x = axis[0];
	double y = axis[1];
	double z = axis[2];

	Matrix mat;
	mat << c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s, 0,
		y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s, 0,
		z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c), 0,
		0, 0, 0, 1;

	return mat;
}

Matrix MathUtil::RotateMat(const Quaternion& q)
{
	Matrix mat = Matrix::Identity();

	double sqw = q.w() * q.w();
	double sqx = q.x()*  q.x();
	double sqy = q.y() * q.y();
	double sqz = q.z() * q.z();
	double invs = 1 / (sqx + sqy + sqz + sqw);

	mat(0, 0) = (sqx - sqy - sqz + sqw) * invs;
	mat(1, 1) = (-sqx + sqy - sqz + sqw) * invs;
	mat(2, 2) = (-sqx - sqy + sqz + sqw) * invs;

	double tmp1 = q.x()*q.y();
	double tmp2 = q.z()*q.w();
	mat(1, 0) = 2.0 * (tmp1 + tmp2) * invs;
	mat(0, 1) = 2.0 * (tmp1 - tmp2) * invs;

	tmp1 = q.x()*q.z();
	tmp2 = q.y()*q.w();
	mat(2, 0) = 2.0 * (tmp1 - tmp2) * invs;
	mat(0, 2) = 2.0 * (tmp1 + tmp2) * invs;

	tmp1 = q.y()*q.z();
	tmp2 = q.x()*q.w();
	mat(2, 1) = 2.0 * (tmp1 + tmp2) * invs;
	mat(1, 2) = 2.0 * (tmp1 - tmp2) * invs;

	return mat;
}

Matrix MathUtil::CrossMat(const Vector& a)
{
	Matrix m;
	m << 0, -a[2], a[1], 0,
		a[2], 0, -a[0], 0,
		-a[1], a[0], 0, 0,
		0, 0, 0, 1;
	return m;
}

Matrix MathUtil::InvRigidMat(const Matrix& mat)
{
	Matrix inv_mat = Matrix::Zero();
	inv_mat.block(0, 0, 3, 3) = mat.block(0, 0, 3, 3).transpose();
	inv_mat.col(3) = -inv_mat * mat.col(3);
	inv_mat(3, 3) = 1;
	return inv_mat;
}

Vector MathUtil::GetRigidTrans(const Matrix& mat)
{
	return Vector(mat(0, 3), mat(1, 3), mat(2, 3), 0);
}

Vector MathUtil::InvEuler(const Vector& euler)
{
	Matrix inv_mat = MathUtil::RotateMat(Vector(1, 0, 0, 0), -euler[0])
		* MathUtil::RotateMat(Vector(0, 1, 0, 0), -euler[1])
		* MathUtil::RotateMat(Vector(0, 0, 1, 0), -euler[2]);
	Vector inv_euler = MathUtil::RotMatToEuler(inv_mat);
	return inv_euler;
}

void MathUtil::RotMatToAxisAngle(const Matrix& mat, Vector& out_axis, double& out_theta)
{
	double c = (mat(0, 0) + mat(1, 1) + mat(2, 2) - 1) * 0.5;
	c = MathUtil::Clamp(c, -1.0, 1.0);

	out_theta = std::acos(c);
	if (std::abs(out_theta) < 0.00001)
	{
		out_axis = Vector(0, 0, 1, 0);
	}
	else
	{
		double m21 = mat(2, 1) - mat(1, 2);
		double m02 = mat(0, 2) - mat(2, 0);
		double m10 = mat(1, 0) - mat(0, 1);
		double denom = std::sqrt(m21 * m21 + m02 * m02 + m10 * m10);
		out_axis[0] = m21 / denom;
		out_axis[1] = m02 / denom;
		out_axis[2] = m10 / denom;
		out_axis[3] = 0;
	}
}

Vector MathUtil::RotMatToEuler(const Matrix& mat)
{
	Vector euler;
	euler[0] = std::atan2(mat(2, 1), mat(2, 2));
	euler[1] = std::atan2(-mat(2, 0), std::sqrt(mat(2, 1) * mat(2, 1) + mat(2, 2) * mat(2, 2)));
	euler[2] = std::atan2(mat(1, 0), mat(0, 0));
	euler[3] = 0;
	return euler;
}

Quaternion MathUtil::RotMatToQuaternion(const Matrix& mat)
{
	double tr = mat(0, 0) + mat(1, 1) + mat(2, 2);
	Quaternion q;

	if (tr > 0) {
		double S = sqrt(tr + 1.0) * 2; // S=4*qw 
		q.w() = 0.25 * S;
		q.x() = (mat(2, 1) - mat(1, 2)) / S;
		q.y() = (mat(0, 2) - mat(2, 0)) / S;
		q.z() = (mat(1, 0) - mat(0, 1)) / S;
	}
	else if ((mat(0, 0) > mat(1, 1) && (mat(0, 0) > mat(2, 2)))) {
		double S = sqrt(1.0 + mat(0, 0) - mat(1, 1) - mat(2, 2)) * 2; // S=4*qx 
		q.w() = (mat(2, 1) - mat(1, 2)) / S;
		q.x() = 0.25 * S;
		q.y() = (mat(0, 1) + mat(1, 0)) / S;
		q.z() = (mat(0, 2) + mat(2, 0)) / S;
	}
	else if (mat(1, 1) > mat(2, 2)) {
		double S = sqrt(1.0 + mat(1, 1) - mat(0, 0) - mat(2, 2)) * 2; // S=4*qy
		q.w() = (mat(0, 2) - mat(2, 0)) / S;
		q.x() = (mat(0, 1) + mat(1, 0)) / S;
		q.y() = 0.25 * S;
		q.z() = (mat(1, 2) + mat(2, 1)) / S;
	}
	else {
		double S = sqrt(1.0 + mat(2, 2) - mat(0, 0) - mat(1, 1)) * 2; // S=4*qz
		q.w() = (mat(1, 0) - mat(0, 1)) / S;
		q.x() = (mat(0, 2) + mat(2, 0)) / S;
		q.y() = (mat(1, 2) + mat(2, 1)) / S;
		q.z() = 0.25 * S;
	}

	return q;
}

void MathUtil::EulerToAxisAngle(const Vector& euler, Vector& out_axis, double& out_theta)
{
	double x = euler[0];
	double y = euler[1];
	double z = euler[2];

	double x_s = std::sin(x);
	double x_c = std::cos(x);
	double y_s = std::sin(y);
	double y_c = std::cos(y);
	double z_s = std::sin(z);
	double z_c = std::cos(z);

	double c = (y_c * z_c + x_s * y_s * z_s + x_c * z_c + x_c * y_c - 1) * 0.5;
	c = Clamp(c, -1.0, 1.0);

	out_theta = std::acos(c);
	if (std::abs(out_theta) < 0.00001)
	{
		out_axis = Vector(0, 0, 1, 0);
	}
	else
	{
		double m21 = x_s * y_c - x_c * y_s * z_s + x_s * z_c;
		double m02 = x_c * y_s * z_c + x_s * z_s + y_s;
		double m10 = y_c * z_s - x_s * y_s * z_c + x_c * z_s;
		double denom = std::sqrt(m21 * m21 + m02 * m02 + m10 * m10);
		out_axis[0] = m21 / denom;
		out_axis[1] = m02 / denom;
		out_axis[2] = m10 / denom;
		out_axis[3] = 0;
	}
}

Vector MathUtil::AxisAngleToEuler(const Vector& axis, double theta)
{
	Quaternion q = AxisAngleToQuaternion(axis, theta);
	return QuaternionToEuler(q);
}

Matrix MathUtil::DirToRotMat(const Vector& dir, const Vector& up)
{
	Vector x = up.cross3(dir);
	double x_norm = x.norm();
	if (x_norm == 0)
	{
		x_norm = 1;
		x = (dir.dot(up) >= 0) ? Vector(1, 0, 0, 0) : Vector(-1, 0, 0, 0);
	}
	x /= x_norm;

	Vector y = dir.cross3(x).normalized();
	Vector z = dir;

	Matrix mat = Matrix::Identity();
	mat.block(0, 0, 3, 1) = x.segment(0, 3);
	mat.block(0, 1, 3, 1) = y.segment(0, 3);
	mat.block(0, 2, 3, 1) = z.segment(0, 3);

	return mat;
}

void MathUtil::DeltaRot(const Vector& axis0, double theta0, const Vector& axis1, double theta1,
	Vector& out_axis, double& out_theta)
{
	Matrix R0 = RotateMat(axis0, theta0);
	Matrix R1 = RotateMat(axis1, theta1);
	Matrix M = DeltaRot(R0, R1);
	RotMatToAxisAngle(M, out_axis, out_theta);
}

Matrix MathUtil::DeltaRot(const Matrix& R0, const Matrix& R1)
{
	return R1 * R0.transpose();
}

Quaternion MathUtil::EulerToQuaternion(const Vector& euler)
{
	Vector axis;
	double theta;
	EulerToAxisAngle(euler, axis, theta);
	return AxisAngleToQuaternion(axis, theta);
}

Vector MathUtil::QuaternionToEuler(const Quaternion& q)
{
	double sinr = 2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	double x = std::atan2(sinr, cosr);

	double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
	double y = 0;
	if (fabs(sinp) >= 1)
	{
		y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	}
	else
	{
		y = asin(sinp);
	}

	double siny = 2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
	double z = std::atan2(siny, cosy);

	return Vector(x, y, z, 0);
}

Quaternion MathUtil::AxisAngleToQuaternion(const Vector& axis, double theta)
{
	// axis must be normalized
	double c = std::cos(theta / 2);
	double s = std::sin(theta / 2);
	Quaternion q;
	q.w() = c;
	q.x() = s * axis[0];
	q.y() = s * axis[1];
	q.z() = s * axis[2];
	return q;
}

void MathUtil::QuaternionToAxisAngle(const Quaternion& q, Vector& out_axis, double& out_theta)
{
	out_theta = 0;
	out_axis = Vector(0, 0, 1, 0);

	Quaternion q1 = q;
	if (q1.w() > 1)
	{
		q1.normalize();
	}

	double sin_theta = std::sqrt(1 - q1.w() * q1.w());
	if (sin_theta > 0.000001)
	{
		out_theta = 2 * std::acos(q1.w());
		out_theta = MathUtil::NormalizeAngle(out_theta);
		out_axis = Vector(q1.x(), q1.y(), q1.z(), 0) / sin_theta;
	}
}

Matrix MathUtil::BuildQuaternionDiffMat(const Quaternion& q)
{
	Matrix mat;
	mat << -0.5 * q.x(), -0.5 * q.y(), -0.5 * q.z(), 0,
		0.5 * q.w(), -0.5 * q.z(), 0.5 * q.y(), 0,
		0.5 * q.z(), 0.5 * q.w(), -0.5 * q.x(), 0,
		-0.5 * q.y(), 0.5 * q.x(), 0.5 * q.w(), 0;
	return mat;
}

Vector MathUtil::CalcQuaternionVel(const Quaternion& q0, const Quaternion& q1, double dt)
{
	Quaternion q_diff = MathUtil::QuatDiff(q0, q1);
	Vector axis;
	double theta;
	QuaternionToAxisAngle(q_diff, axis, theta);
	return (theta / dt) * axis;
}

Vector MathUtil::CalcQuaternionVelRel(const Quaternion& q0, const Quaternion& q1, double dt)
{
	// calculate relative rotational velocity in the coordinate frame of q0
	Quaternion q_diff = q0.conjugate() * q1;
	Vector axis;
	double theta;
	QuaternionToAxisAngle(q_diff, axis, theta);
	return (theta / dt) * axis;
}

Quaternion MathUtil::VecToQuat(const Vector& v)
{
	return Quaternion(v[0], v[1], v[2], v[3]);
}

Vector MathUtil::QuatToVec(const Quaternion& q)
{
	return Vector(q.w(), q.x(), q.y(), q.z());
}

Quaternion MathUtil::QuatDiff(const Quaternion& q0, const Quaternion& q1)
{
	return q1 * q0.conjugate();
}

double MathUtil::QuatDiffTheta(const Quaternion& q0, const Quaternion& q1)
{
	Quaternion dq = QuatDiff(q0, q1);
	return QuatTheta(dq);
}

double MathUtil::QuatTheta(const Quaternion& dq)
{
	double theta = 0;
	Quaternion q1 = dq;
	if (q1.w() > 1)
	{
		q1.normalize();
	}

	double sin_theta = std::sqrt(1 - q1.w() * q1.w());
	if (sin_theta > 0.0001)
	{
		theta = 2 * std::acos(q1.w());
		theta = MathUtil::NormalizeAngle(theta);
	}
	return theta;
}

Quaternion MathUtil::VecDiffQuat(const Vector& v0, const Vector& v1)
{
	return Quaternion::FromTwoVectors(v0.segment(0, 3), v1.segment(0, 3));
}

Vector MathUtil::QuatRotVec(const Quaternion& q, const Vector& dir)
{
	Vector rot_dir = Vector::Zero();
	rot_dir.segment(0, 3) = q * dir.segment(0, 3);
	return rot_dir;
}

Quaternion MathUtil::MirrorQuaternion(const Quaternion& q, eAxis axis)
{
	Quaternion mirror_q;
	mirror_q.w() = q.w();
	mirror_q.x() = (axis == eAxisX) ? q.x() : -q.x();
	mirror_q.y() = (axis == eAxisY) ? q.y() : -q.y();
	mirror_q.z() = (axis == eAxisZ) ? q.z() : -q.z();
	return mirror_q;
}