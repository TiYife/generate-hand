#include "HandModel.h"
#include <iostream>
#include "util/MathUtil.h"

HandModel::HandModel()	
{
	init();
}


HandModel::~HandModel()
{
}


void HandModel::init()
{
	m_joints_attach.clear();	
	m_bodies_attach.clear();
	m_bodies_attach_theta.clear();
	m_bodies_length.clear();
	m_joint_dims.clear();
	m_base_rotation.clear();
	m_base_relative_rotation.clear();

	m_joints_attach.resize(kJointsNum + kFingersNum);
	m_bodies_attach.resize(kJointsNum);
	m_bodies_attach_theta.resize(kJointsNum);
	m_bodies_length.resize(kJointsNum);
	m_joint_dims.resize(kJointsNum);
	m_base_rotation.resize(kJointsNum);
	m_base_relative_rotation.resize(kJointsNum);

	loadTempModel();
}


void HandModel::update(const vector<Vector>& joints)
{
	int parent[kJointsNum + kFingersNum] = {
		0, 0 ,1, 2,
		3, 4, 5, 6,
		3, 8, 9, 10,
		3, 12, 13, 14,
		3, 16, 17, 18,
		3, 20, 21, 22,
		7, 11, 15, 19, 23	//tips' parent joint
	};

	int child[kJointsNum + kFingersNum] = {
		1, 2, 3, 0,		// 0 means zero attach dis
		5, 6, 7, 24,
		9, 10, 11, 25,
		13, 14, 15, 26,
		17, 18, 19, 27,
		21, 22, 23, 28,
		24, 25, 26, 27, 28	//tips' have no child
	};

	for (int i = 0; i < kJointsNum + kFingersNum; i++) {
		m_joints_attach[i] = joints[i] - joints[parent[i]];
	}

	for (int i = 0; i < kJointsNum; i++) {
		m_bodies_attach[i] = m_joints_attach[child[i]] / 2;
		m_bodies_length[i] = m_joints_attach[child[i]].norm() - kTolerance;

		Quaternion quat = MathUtil::VecDiffQuat(Vector(0, 1, 0, 0), m_joints_attach[child[i]]);
		m_bodies_attach_theta[i] = MathUtil::QuaternionToEuler(quat);

		Quaternion quat_parent = MathUtil::VecDiffQuat(Vector(0, 1, 0, 0), m_joints_attach[i]);
		Quaternion quat_relative = MathUtil::QuatDiff(quat_parent, quat).normalized();
		Quaternion quat_relative2 = MathUtil::VecDiffQuat(m_joints_attach[i], m_joints_attach[child[i]]).normalized();
		m_base_rotation[i] = quat;
		m_base_relative_rotation[i] = quat_relative;
	}
}

void HandModel::update(const vector<Vector>& joints, const vector<Quaternion> rotations)
{
	int parent[kJointsNum + kFingersNum] = {
		0, 0 ,1, 2,
		3, 4, 5, 6,
		3, 8, 9, 10,
		3, 12, 13, 14,
		3, 16, 17, 18,
		3, 20, 21, 22,
		7, 11, 15, 19, 23	//tips' parent joint
	};

	int child[kJointsNum + kFingersNum] = {
		1, 2, 3, 0,		// 0 means zero attach dis
		5, 6, 7, 24,
		9, 10, 11, 25,
		13, 14, 15, 26,
		17, 18, 19, 27,
		21, 22, 23, 28,
		24, 25, 26, 27, 28	//tips' have no child
	};

	for (int i = 0; i < kJointsNum + kFingersNum; i++) {
		m_joints_attach[i] = joints[i] - joints[parent[i]];
	}

	for (int i = 0; i < kJointsNum; i++) {
		m_bodies_attach[i] = m_joints_attach[child[i]] / 2;
		m_bodies_length[i] = m_joints_attach[child[i]].norm() - kTolerance;

		Quaternion quat = MathUtil::VecDiffQuat(Vector(0, 1, 0, 0), m_joints_attach[child[i]]).normalized();
		Quaternion base_quat = MathUtil::VecDiffQuat(Vector(0, 1, 0, 0), Vector(0,0,-1,0));
		Quaternion new_quat =(rotations[i] * base_quat).normalized();
		Vector v = MathUtil::QuatRotVec(rotations[i].inverse(), m_joints_attach[child[i]]).normalized();

		m_bodies_attach_theta[i] = MathUtil::QuaternionToEuler(quat);
		m_bodies_attach_theta[i] = MathUtil::QuaternionToEuler(new_quat);

		std::cout << MathUtil::QuatToVec(quat).transpose() << "\t" << MathUtil::QuatToVec(rotations[i]).transpose() << "\n";
		Quaternion quat_parent = MathUtil::VecDiffQuat(Vector(0, 1, 0, 0), m_joints_attach[i]);
		Quaternion quat_relative = MathUtil::QuatDiff(quat_parent, quat).normalized();
		Quaternion quat_relative2 = MathUtil::VecDiffQuat(m_joints_attach[i], m_joints_attach[child[i]]).normalized();
		m_base_rotation[i] = quat;
		m_base_relative_rotation[i] = quat_relative;
	}
}

bool HandModel::loadTempModel()
{
	bool succ = true;
	ifstream f_stream(kTempModelFile);
	Json::Reader reader;

	succ = reader.parse(f_stream, m_json_root);

	parseSkeleton(m_json_root["Skeleton"]);

	f_stream.close();
	return succ;
}

void HandModel::writeNewFile()
{
	ofstream f_stream(kOutputModelFile);
	Json::StyledWriter writer;

	f_stream << writer.write(m_json_root);
	f_stream.close();
}

void HandModel::writeBack()
{
	if (!m_json_root["Skeleton"].isNull()) {
		writeSkeleton(m_json_root["Skeleton"]);
	}
	if (!m_json_root["BodyDefs"].isNull()) {
		writeBody(m_json_root["BodyDefs"]);
	}

	if (!m_json_root["DrawShapeDefs"].isNull()) {
		writeDrawShape(m_json_root["DrawShapeDefs"]);
	}
	
	writeNewFile();
}

bool HandModel::parseSkeleton(Json::Value & root)
{
	Json::Value& joints = root["Joints"];
	for (int i = 0; i < kJointsNum; i++) {
		string joint_type = joints[i]["Type"].asString();
		int dim = 0;

		if (i == 0) {
			dim = 7;
		}
		else if (joint_type == "fixed") {
			dim = 0;
		}
		else if (joint_type == "spherical") {
			dim = 4;
		}
		else if (joint_type == "revolute") {
			dim = 1;
		}
		else {
			return false;
		}

		m_joint_dims[i] = dim;
		m_motion_dim += dim;

	}

	return true;
}

void HandModel::writeSkeleton(Json::Value& root)
{
	Json::Value& joints = root["Joints"];
	for (int i = 0; i < kJointsNum; i++) {
		Json::Value& joint_json = joints[i];
		joint_json["AttachX"] = m_joints_attach[i].x();
		joint_json["AttachY"] = m_joints_attach[i].y();
		joint_json["AttachZ"] = m_joints_attach[i].z();
		//cout << joint_json << endl;
		cout << m_joints_attach[i].transpose() << std::endl;
	}
	//cout << joints << endl;

}

void HandModel::writeBody(Json::Value& root)
{
	for (int i = 0; i < kJointsNum; i++) {
		Json::Value& joint_json = root[i];
		joint_json["AttachX"] = m_bodies_attach[i].x();
		joint_json["AttachY"] = m_bodies_attach[i].y();
		joint_json["AttachZ"] = m_bodies_attach[i].z();

		joint_json["AttachThetaX"] = m_bodies_attach_theta[i].x();
		joint_json["AttachThetaY"] = m_bodies_attach_theta[i].y();
		joint_json["AttachThetaZ"] = m_bodies_attach_theta[i].z();

		joint_json["Param1"] = m_bodies_length[i];
	}
}

void HandModel::writeDrawShape(Json::Value& root)
{
	for (int i = 0; i < kJointsNum; i++) {
		Json::Value& joint_json = root[i];
		joint_json["AttachX"] = m_bodies_attach[i].x();
		joint_json["AttachY"] = m_bodies_attach[i].y();
		joint_json["AttachZ"] = m_bodies_attach[i].z();

		joint_json["AttachThetaX"] = m_bodies_attach_theta[i].x();
		joint_json["AttachThetaY"] = m_bodies_attach_theta[i].y();
		joint_json["AttachThetaZ"] = m_bodies_attach_theta[i].z();

		joint_json["Param1"] = m_bodies_length[i];

	}
}

