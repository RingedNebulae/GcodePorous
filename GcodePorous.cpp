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
	Gcode currGcode;
	currGcode.readInCuraGcode(inputFilePath + inputGcodeName);

	//��ȡsolidtexture field
	SolidTexture solidTexture;
	cout << "begin solid field texture" << endl;
	solidTexture.readinTextureField2(inputTextureField);

	//����solidtexture field�����µ�gcode
	solidTexture.generateNewGcode(currGcode);
	//���gcode
	cout << "start output gcode" << endl;
	currGcode.outputGcode(outputFilePath + "_solidField_" + inputGcodeName);

	return 0;
}