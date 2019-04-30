#ifndef READINCONFIG_H
#define READINCONFIG_H

/*******************************************
该头文件负责处理输入参数
修改记录：
2018.12.06 将频率系数项FREQUENCY删除
（因为频率是与波长相关的非独立变量，属于重复参数）
********************************************/

#include <iostream>
#include <fstream>
#include <string>
#include "GlobalVariable.h"

using namespace std;


int readInConfigure()
{
	//定义输入变量
	char buffer[256];//读取文件缓存
	string inputbuffer;//为了读入文件的便于格式转化

	//错误检查，确保文件打开正确
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

		//以斜杠开头的是文件中的注释，忽略并读入下一行
		if (inputbuffer[0] == '/')
			continue;
		//INPUT_FILE_PATH 是输入文件路径的标识符
		else if (inputbuffer == "INPUT_FILE_PATH")
		{
			in.getline(buffer, 100);
			inputbuffer = buffer;
			inputFilePath = inputbuffer;
			cout << "Input file path is: " << inputFilePath << endl;
		}
		//OUTPUT_FILE_PATH 是输出文件路径标识符
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
		//INPUT_GCODE_NAME 是读取的文件名
		else if (inputbuffer == "INPUT_GCODE_NAME")
		{
			in.getline(buffer, 100);
			inputbuffer = buffer;
			inputGcodeName = inputbuffer;
			cout << "Input Gcode name is: " << inputGcodeName << endl;
		}
		//INPUT_TEXTURE_FIELD 是读取texture field的文件名
		else if (inputbuffer == "INPUT_TEXTURE_FIELD")
		{
			in.getline(buffer, 100);
			inputbuffer = buffer;
			inputTextureField = inputbuffer;
			cout << "Input Texture name is: " << inputTextureField << endl;
		}
		//UNITE是单位长度对应的E
		else if (inputbuffer == "UNITE")
		{
			in.getline(buffer, 100);
			inputbuffer = buffer;
			unitE = stof(inputbuffer);
		}
		//SCANLINE_GAP是扫描线间距
		else if (inputbuffer == "SCANLINE_GAP")
		{
			in.getline(buffer, 100);
			inputbuffer = buffer;
			scanLineGap = stof(inputbuffer);
		}
	}
}

#endif // !READINCONFIG_H