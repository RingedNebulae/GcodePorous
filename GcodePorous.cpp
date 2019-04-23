//GcodePorous.cpp 程序入口点

#include "Gcode.h"
#include "readInConfig.h"
#include "GlobalVariable.h"
#include "SolidTexture.h"

int main()
{
	//读取配置文件
	readInConfigure();

	//读取Gcode文件
	Gcode currGcode;
	currGcode.readInCuraGcode(inputFilePath + inputGcodeName);

	//读取solidtexture field
	SolidTexture solidTexture;
	cout << "begin solid field texture" << endl;
	solidTexture.readinTextureField2(inputTextureField);

	//根据solidtexture field生成新的gcode
	solidTexture.generateNewGcode(currGcode);
	//输出gcode
	cout << "start output gcode" << endl;
	currGcode.outputGcode(outputFilePath + "_solidField_" + inputGcodeName);

	return 0;
}