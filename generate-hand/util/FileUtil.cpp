#include <iostream>
#include <fstream>
#include <string>
#include "FileUtil.h"

void FileUtil::writePoints(string file_path, const std::vector<Vector>& points)
{
	ofstream out_file;
	out_file.open(file_path);

	for (auto & p : points) {
		out_file << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
	}

	out_file.close();
	return;
}
