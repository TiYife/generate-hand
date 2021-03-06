#include <iostream>
#include "leapmotion/leapconnection.h"
#include "util/FileUtil.h"
#include "hand/HandModel.h"


using namespace std;

using FRAME = LEAP_TRACKING_EVENT;
using HAND = LEAP_HAND;
using BONE = LEAP_BONE;
using FINGER = LEAP_DIGIT;

void generate_hand_model(const vector<Vector>& joints) {

}


void parseHand(HAND * hand, vector<Vector>& joints) {
	BONE arm = hand->arm;
	FINGER fingers[5] = { hand->index, hand->middle,
		hand->pinky, hand->ring, hand->thumb };

	joints.push_back(Vector(0, 0, 0, 0));
	joints.push_back(Vector(0, 0, 0, 0));
	joints.push_back(EigenUtil::toVec(arm.prev_joint));
	joints.push_back(EigenUtil::toVec(arm.next_joint));

	for (auto & f : fingers) {
		for (auto & b : f.bones) {
			joints.push_back(EigenUtil::toVec(b.prev_joint));
		}
	}

	for (auto & f : fingers) {
		joints.push_back(EigenUtil::toVec(f.bones[3].next_joint));
	}

	for (auto & j : joints) {
		j = j / 1000;
	}

	FileUtil::writePoints("D:/Documents/Visual Studio 2015/Projects/generate-hand/generate-hand/outputs/joints.obj", joints);

}


int main() {
	OpenConnection();
	while (!IsConnected) {
		millisleep(100);
		std::cout << "unConnected!/n";
	}

	HandModel model;

	FRAME *frame = GetFrame();
	int no = 0;
	while (frame) {
		no++;
		int frame_no = frame->tracking_frame_id;
		std::cout << frame->tracking_frame_id << "/t";
		LEAP_HAND* hand = &frame->pHands[0];
		if (hand) {
			std::cout << hand->palm.position.x << "/t" << hand->palm.position.y << "/t" << hand->palm.position.z << "/n	";
			if (no == 30)
			{
				vector<Vector> ori_joints = vector<Vector>();
				parseHand(hand, ori_joints);
				model.update(ori_joints);
				model.writeBack();
				break;
			}
		}
	}
	return 0;
}

