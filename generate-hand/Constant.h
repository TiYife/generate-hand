#pragma once
#include <string>
using namespace std;

const string kTempModelFile = "D:/Documents/Visual Studio 2015/Projects/generate-hand/generate-hand/outputs/temp.txt";
const string kTempMotionFile = "";

//const string kOutputDir = "D:/Documents/Visual Studio 2015/Projects/generate-hand/generate-hand/outputs/";
const string kOutputDir = "D:/Documents/Visual Studio 2015/Projects/DeepMimic-dev/DeepMimicCore/data/generated/hand3d_test/";
const string kOutputModelFile = kOutputDir + "character.txt";
const string kOutputMotionFile = kOutputDir + "motion.txt";


const int kJointsNum = 24;
const int kFingersNum = 5;
const int kTolerance = 0.001 * 2;