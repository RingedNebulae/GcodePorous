//GcodePorous.cpp ������ڵ�

#include "Gcode.h"
#include "readInConfig.h"
#include "GlobalVariable.h"
#include "SolidTexture.h"

int main()
{
	//��ȡ�����ļ�
	readInConfigure();

	//��ȡGcode�ļ�
	Gcode currGcode,newGcode;
	currGcode.readInCuraGcode(inputFilePath + inputGcodeName);
	
	//��ȡsolidtexture field
	SolidTexture solidTexture;
	cout << "begin solid field texture" << endl;
	solidTexture.readinTextureField2(inputTextureField);
	//solidTexture.generateTPMSfield();

	//����solidtexture field�����µ�gcode
	solidTexture.generateNewGcode(currGcode,newGcode);
	//solidTexture.generateNewGcodeTPMS(currGcode, newGcode);
	
	newGcode.generateZvalue(newGcode);

	//���gcode
	cout << "start output gcode" << endl;
	newGcode.outputGcode(outputFilePath + "_solidField_" + inputGcodeName);

	return 0;
}