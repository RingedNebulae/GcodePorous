#ifndef READINCONFIG_H
#define READINCONFIG_H

/*******************************************
��ͷ�ļ��������������
�޸ļ�¼��
2018.12.06 ��Ƶ��ϵ����FREQUENCYɾ��
����ΪƵ�����벨����صķǶ��������������ظ�������
********************************************/

#include <iostream>
#include <fstream>
#include <string>
#include "GlobalVariable.h"

using namespace std;


int readInConfigure()
{
	//�����������
	char buffer[256];//��ȡ�ļ�����
	string inputbuffer;//Ϊ�˶����ļ��ı��ڸ�ʽת��

	//�����飬ȷ���ļ�����ȷ
	ifstream in("Configure");
	if (!in.is_open())
	{
		cout << "Error opening file!";
		return 1;
	}
	while (!in.eof())
	{
		in.getline(buffer, 200);
		inputbuffer = buffer;

		//��б�ܿ�ͷ�����ļ��е�ע�ͣ����Բ�������һ��
		if (inputbuffer[0] == '/')
			continue;
		//INPUT_FILE_PATH �������ļ�·���ı�ʶ��
		else if (inputbuffer == "INPUT_FILE_PATH")
		{
			in.getline(buffer, 100);
			inputbuffer = buffer;
			inputFilePath = inputbuffer;
			cout << "Input file path is: " << inputFilePath << endl;
		}
		//OUTPUT_FILE_PATH ������ļ�·����ʶ��
		else if (inputbuffer == "OUTPUT_FILE_PATH")
		{
			in.getline(buffer, 100);
			inputbuffer = buffer;
			outputFilePath = inputbuffer;
			cout << "Output file path is: " << outputFilePath << endl;
		}
		else if (inputbuffer == "OUTPUT_LINE_FOR_RENDERING")
		{
			in.getline(buffer, 100);
			inputbuffer = buffer;
			outputLinePath = inputbuffer;
			cout << "Output line for rendering path is: " << outputLinePath << endl;
		}
		//INPUT_GCODE_NAME �Ƕ�ȡ���ļ���
		else if (inputbuffer == "INPUT_GCODE_NAME")
		{
			in.getline(buffer, 100);
			inputbuffer = buffer;
			inputGcodeName = inputbuffer;
			cout << "Input Gcode name is: " << inputGcodeName << endl;
		}
		//INPUT_TEXTURE_FIELD �Ƕ�ȡtexture field���ļ���
		else if (inputbuffer == "INPUT_TEXTURE_FIELD")
		{
			in.getline(buffer, 100);
			inputbuffer = buffer;
			inputTextureField = inputbuffer;
			cout << "Input Texture name is: " << inputTextureField << endl;
		}
		//UNITE�ǵ�λ���ȶ�Ӧ��E
		else if (inputbuffer == "UNITE")
		{
			in.getline(buffer, 100);
			inputbuffer = buffer;
			unitE = stof(inputbuffer);
		}
		//SCANLINE_GAP��ɨ���߼��
		else if (inputbuffer == "SCANLINE_GAP")
		{
			in.getline(buffer, 100);
			inputbuffer = buffer;
			scanLineGap = stof(inputbuffer);
		}
	}
}

#endif // !READINCONFIG_H