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
extern float unitE;
extern float currE;//将E作为一个全局的常量，记录在没有加入当前线段前当前的E值

class GlobalVariable
{
public:
	GlobalVariable();
	~GlobalVariable();

};

