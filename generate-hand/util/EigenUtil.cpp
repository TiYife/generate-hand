#include "EigenUtil.h"

Vector EigenUtil::toVec(LEAP_VECTOR v)
{
	return Vector(v.x, v.y, v.z, 0);
}

Quaternion EigenUtil::toQuat(LEAP_QUATERNION q)
{
	return Quaternion(q.w, q.x, q.y, q.z);
}
