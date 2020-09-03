#pragma once

#include <string>
#include "json/json.h"
#include "EigenUtil.h"

class cJsonUtil
{
public:
	static std::string BuildVectorJson(const Vector& vec);
	static bool ReadVectorJson(const Json::Value& root, Vector& out_vec);
	static std::string BuildVectorJson(const Eigen::VectorXd& vec);
	static std::string BuildVectorString(const Eigen::VectorXd& vec);
	static bool ReadVectorJson(const Json::Value& root, Eigen::VectorXd& out_vec);

private:
	
};
