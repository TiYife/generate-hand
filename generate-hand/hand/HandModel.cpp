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

	m_joints_attach.resize(kJointsNum + kFingersNum);
	m_bodies_attach.resize(kJointsNum);
	m_bodies_attach_theta.resize(kJointsNum);
	m_bodies_length.resize(kJointsNum);

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
		quat.normalize();
		m_bodies_attach_theta[i] = MathUtil::QuaternionToEuler(quat);

	}
}

bool HandModel::loadTempModel()
{
	bool succ = true;
	ifstream f_stream(kTempModelFile);
	Json::Reader reader;

	succ = reader.parse(f_stream, m_json_root);
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

