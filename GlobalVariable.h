#pragma once
#include <string>

using namespace std;

//定义全局变量
extern string inputFilePath;
extern string outputFilePath;
extern string inputGcodeName;
extern string outputLinePath;

extern string inputTextureField;

extern float scanLineGap;


class GlobalVariable
{
public:
	GlobalVariable();
	~GlobalVariable();

};

