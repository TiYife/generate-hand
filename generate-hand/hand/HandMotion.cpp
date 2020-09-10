#include "HandMotion.h"
#include "util/MathUtil.h"
#include <iostream>


HandMotion::HandMotion(HandModel * model):m_rest_hand(model)
{
	init();
}


HandMotion::~HandMotion()
{
}

bool HandMotion::init()
{
	return true;
}

bool HandMotion::addMotion(vector<Vector> joints)
{
	vector<Vector> bone_direction = vector<Vector>(kJointsNum + kFingersNum);

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
		bone_direction[i] = joints[i] - joints[parent[i]];
	}

	vector<double> motion = vector<double>();
	for (int i = 0; i < kJointsNum; i++) {
		Quaternion dir = MathUtil::VecDiffQuat(Vector(0, 1, 0, 0), bone_direction[child[i]]);
		Quaternion dir_parent = MathUtil::VecDiffQuat(Vector(0, 1, 0, 0), bone_direction[i]);
		Quaternion dir_parent_base = m_rest_hand->m_base_rotation[parent[i]];
		Quaternion rotation_parent = MathUtil::QuatDiff(dir_parent_base, dir_parent);
		Quaternion dir_virtual = rotation_parent * m_rest_hand->m_base_rotation[i];


		Quaternion quat_relative = MathUtil::QuatDiff(dir_parent, dir);
		Quaternion quat_relative_base = m_rest_hand->m_base_relative_rotation[i];
		Quaternion rotation = quat_relative;
		//Quaternion rotation = MathUtil::QuatDiff(quat_relative, quat_relative_base);
		//Quaternion rotation = MathUtil::QuatDiff(dir_virtual, dir);
		//Quaternion rotation = MathUtil::VecDiffQuat(bone_direction[i], bone_direction[child[i]]);
		rotation.normalize();

		if (m_rest_hand->m_joint_dims[i] == 7) {
			motion.push_back(0);
			motion.push_back(1);
			motion.push_back(0);

			Vector q = MathUtil::QuatToVec(rotation);
			motion.push_back(q.w());
			motion.push_back(q.x());
			motion.push_back(q.y());
			motion.push_back(q.z());
		}
		else if (m_rest_hand->m_joint_dims[i] == 4) {
			Vector q = MathUtil::QuatToVec(rotation);
			motion.push_back(q.w());
			motion.push_back(q.x());
			motion.push_back(q.y());
			motion.push_back(q.z());
		}
		else if (m_rest_hand->m_joint_dims[i] == 1) {
			motion.push_back(MathUtil::QuaternionToEuler(rotation).z());
		}
		
		//m_base_rotation[i] = quat;
		//m_bodies_attach_theta[i] = MathUtil::QuaternionToEuler(quat);
	}

	if (motion.size() != m_rest_hand->m_motion_dim) {
		return false;
	}

	for (auto d : motion) {
		cout << d <<'\n';
	}
	cout << "\n\n";
	m_motion_list.push_back(motion);
	return true;
}

bool HandMotion::writeMotion()
{
	Json::Value root;
	root["Loop"] = "wrap";

	Json::Value frames;
	for (auto & motion : m_motion_list) {
		Json::Value json_frame;
		json_frame.append(0.03);//todo 
		for (auto d : motion) {
			json_frame.append(d);
		}
		frames.append(json_frame);
	}
	root["Frames"] = frames;

	ofstream f_stream(kOutputMotionFile);
	Json::StyledWriter writer;

	f_stream << writer.write(root);
	f_stream.close();

	return false;
}
