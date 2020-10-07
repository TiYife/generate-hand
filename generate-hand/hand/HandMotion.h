#pragma once
#include "HandModel.h"

class HandMotion
{
public:
	HandMotion(HandModel * model);
	~HandMotion();

	bool init();
	bool addMotion(vector<Vector> joints);
	bool addMotion(Vector root_pos, vector<Quaternion> rotations);
	bool writeMotion();

private:
	HandModel * m_rest_hand;
	vector<vector<double>> m_motion_list;


};

