#pragma once
#include <fstream>
#include <vector>
#include "util/EigenUtil.h"
#include "util/JsonUtil.h"
#include "Constant.h"


using namespace std;
class HandModel
{
public:
	vector<int> m_joint_dims;
	int m_motion_dim;
	vector<Quaternion> m_base_rotation;
	vector<Quaternion> m_base_relative_rotation;




	HandModel();
	~HandModel();

	void init();
	void update(const vector<Vector>& joints);
	void writeBack();

private:
	vector<Vector> m_joints_attach;
	vector<Vector> m_bodies_attach;
	vector<Vector> m_bodies_attach_theta;
	vector<double> m_bodies_length;

	Json::Value m_json_root;



	bool loadTempModel();
	void writeNewFile();

	bool parseSkeleton(Json::Value& root);
	void writeSkeleton(Json::Value& root);
	void writeBody(Json::Value& root);
	void writeDrawShape(Json::Value& root);

};

