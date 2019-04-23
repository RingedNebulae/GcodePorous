#pragma once
#ifndef _GCODE_
#define _GCODE_

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <math.h>
#include <algorithm>
#include <iomanip>
#include <regex>

/**************************************************
������Ҫʵ�ֵĹ�����Gcode���ݽṹ�����룬���
**************************************************/
#define PI 3.14159265

using namespace std;

struct eachValue {
	bool isValidValue=false;
	double value;
};

//the basic structure of command line
//Gxx Fxx Xxx Yxx Exx
struct GCommand {
	string Gcommand="";//G
	int feedRate=-1;//F
	eachValue X;//X
	eachValue Y;//Y
	eachValue Z;//Z
	eachValue E;//E
};

enum codeType
{
	SKIRT,
	FILL,
	WALL_INNER,
	WALL_OUTER,
	SKIN,
	SUPPORT,
	UNDEFINED,
};

struct eachLine {
	bool isGcommand;
	codeType codetype;
	string otherCommand;//�����G����ָ��
	GCommand GcommandLine;//����G����ָ��
};

//���ʱ����contour index��˳��
//����contour�ṹ����Ϊ�˱�����һ���ڰ���������
struct contour {
	int contourIndex;
	vector<eachLine> wallOuterContainer;
	vector<eachLine> wallInnerContainer;
	vector<eachLine> skinContainer;
	vector<eachLine> fillContainer;
	vector<eachLine> supportContainer;
	vector<codeType> typeSequence;//һ�������ڲ�ͬ���͵����˳��
};

struct LayerInfo {
	int layer;
	vector<eachLine> lineContainer;//�����������������Ϣ
	vector<eachLine> skirtContainer;
	vector<contour> contourContainer;
	
};

//��point2D�����ԭGcode���ó��ĵ�
struct point2D
{
	float x;
	float y;
	double e;
	int index;//��ԭgcode�е�λ��
};

struct point3D
{
	float x;
	float y;
	float z;
};

//��samplePoint2D��������sample point
struct samplePoint2D {
	float x=0;
	float y=0;
	double e=0;
	int indexStart=0;
	int indexEnd=0;
	int contourIndex;//���ڼ�¼������λ���ĸ�contour��
	int parentContourIndex;//���ڼ�¼�ò������������ĸ�������
	int parentPointIndex;//���ڼ�¼�ò������Ӧ�ڸ������ϵ��ĸ���
	float ContourWaveLength;//��¼�ò������Ӧ��wavelength

	//���ڸ���e�Ĵ�С�Բ������������
	bool operator< (const samplePoint2D& other)
	{
		return e < other.e;
	}
};

struct startPointPosition
{
	point2D startPoint;
	int position;
};

class Gcode
{
public:
	Gcode();
	~Gcode();

	bool readInCuraGcode(string fileName);

	bool isNewContour(const vector<string> &inputGcode, int index, string inputbuffer);
	void parseCommandLine(string inputbuffer,eachLine &tmpLine);
	void clearContourContainer(contour &tmpContour);

	bool outputGcode(string fileName);
	bool outputGcodeShell(string fileName);
	void outputLine(eachLine tmpline,ofstream &out);

	void updateGcode();

	//�漰�����������㣬��Ҫ֪������XYEֵ
	point2D findActuralStartPoint(Gcode currGcode, int layerBeforeIdx, int currLayerIdx, int contourIndex);
	//ֻ��Ҫ����һ���������ȵ�����µ��ã����Բ���E
	point2D findActuralStartXY(LayerInfo currLayer, int contourIndex);

	point2D findActuralEndPoint(LayerInfo tmpLayer, int contourIndex);

	//the structure of Gcode file 
	// start-Gcode
	// layer by layer command
	// end-Gcode
	vector<string> startGcode;
	vector<LayerInfo> mainBody;
	vector<string> endGcode;
	vector<float> zValue;

	//Assistant information
	//vector<vector<samplePoint2D>> samplePoints;
	void prepareOuterlines(Gcode currGcode, vector<point3D> &outputPoints, vector<int> &outContourIdx);
	void prepareOuterLines(Gcode currGcode, vector<vector<point3D>> &outputPoints);
	
	vector<vector<vector<samplePoint2D>>> samplePoints;
	
	//�Ե����ƽ��
	void smoothGcodeWallouter(vector<vector<point3D>> &outputPoints);
	point3D smoothPoints(point3D currPoint, point3D prePoint, point3D nextPoint);

	void outputLineSegmentsForRendering(Gcode currGcode, string fileName);
	void outputLineObj(Gcode currGcode, string fileName);
	void outputLineObj2(Gcode currGcode, string fileName);
	void outputTextureFieldContours(string fileName);

};

#endif // !_GCODE_
