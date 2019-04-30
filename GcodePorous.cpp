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
	Gcode currGcode,newGcode;
	currGcode.readInCuraGcode(inputFilePath + inputGcodeName);
	
	//读取solidtexture field
	SolidTexture solidTexture;
	cout << "begin solid field texture" << endl;
	solidTexture.readinTextureField2(inputTextureField);
	//solidTexture.generateTPMSfield();

	//根据solidtexture field生成新的gcode
	solidTexture.generateNewGcode(currGcode,newGcode);
	//solidTexture.generateNewGcodeTPMS(currGcode, newGcode);
	
	newGcode.generateZvalue(newGcode);

	//输出gcode
	cout << "start output gcode" << endl;
	newGcode.outputGcode(outputFilePath + "_solidField_" + inputGcodeName);

	return 0;
}