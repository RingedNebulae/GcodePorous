#pragma once
#include <string>

using namespace std;

//����ȫ�ֱ���
extern string inputFilePath;
extern string outputFilePath;
extern string inputGcodeName;
extern string outputLinePath;

extern string inputTextureField;

extern float scanLineGap;
extern float unitE;
extern float currE;//��E��Ϊһ��ȫ�ֵĳ�������¼��û�м��뵱ǰ�߶�ǰ��ǰ��Eֵ

class GlobalVariable
{
public:
	GlobalVariable();
	~GlobalVariable();

};

