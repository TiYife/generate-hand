#pragma once
#include <vector>
#include "util/EigenUtil.h"
#include "leapmotion/leapconnection.h"

using namespace std;
class FileUtil
{
public:
	static void writePoints(string file_path, const vector<Vector>& points);
	
};

