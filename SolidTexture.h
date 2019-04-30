#pragma once

#include "Gcode.h"
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include "MarchingSquares.h"
#include "Operation.h"
#include "GlobalVariable.h"

using namespace std;

struct Line
{
	int p1;
	int p2;
};

struct ActiveEdgeTable
{
	float x;//ɨ�������Ľ�������
	float dletaX;//�ӵ�ǰɨ���ߵ���һ��ɨ���߼�x��������б�ʵĵ���
	float yMax;//�ñߵ����yֵ
	float yMin;//�ñߵ���Сyֵ
	ActiveEdgeTable *next;//ָ����һ���ߵ�ָ��
};

struct NewEdgeTable
{
	float yMax;//�ñߵ����yֵ
	float xMin;//�ñ߽ϵ͵��x����ֵ
	float yMin;//�ñ߽ϵ͵��y����ֵ
	float invSlope;//б�ʵĵ���
	NewEdgeTable *next;//ָ����һ���ߵ�ָ��
};

class SolidTexture
{
public:
	SolidTexture();
	~SolidTexture();

	//����texture field����Gcode
	void generateNewGcode(Gcode & currGcode, Gcode &newGcode);
	void generateNewGcodeTPMS(Gcode & currGcode, Gcode &newGcode);

	//�ȼ����ܲ�����
	void densifyGcode(Gcode &currGcode);
	void insertPoint(float length, point2D v1, point2D v2, point2D &addPoint);

	//������������ĺ���
	float getLength(point2D v1, point2D v2) { return sqrt((v2.x - v1.x)*(v2.x - v1.x) + (v2.y - v1.y)*(v2.y - v1.y)); };


	//��ȡtexture field
	//void readinTextureField(string filepath);
	void readinTextureField2(string filepath);


	//����solidtexture������
	//double ***TextureField;
	vector<vector<double>> textureField;
	vector<vector<double>> tpmsField;

	//��marching squares������߶��еõ�������ͨ������,

	//�жϸ����ĵ��Ƿ���dict�У�����ڸ���index�����ڷ���-1
	int isInDict(vector<point2D> &outPointDict, point2D queryPoint);
	//������texturePoints�к�������ĵ㣬����������+����ķ�ʽѹ����ʾ
	void getPointsDictionary(vector<point2D> inTexturePoints, vector<point2D> &outPointDict, vector<int> &outPointIndex);
	//����ÿ���߶�Ӧ�Ķ��������
	void getPointIdxEachLine(vector<int> inPointIndex, vector<Line> &outLine);
	//�ҵ�ÿ���������ڵĶ���
	void getlineByPoint(vector<Line> inLine, map<int, vector<int>> &pointNeighbor);
	
	//������ͨ��֧
	//���ConnectedComponent����¼�˸�����ͨ��֧
	void constructConnectedComponent(map<int, vector<int>> pointNeighbor, vector<vector<int>> &ConnectedComponent);
	
	//�ж϶�����Ƿ�Ϊ˳ʱ��
	bool IsPolyClockwise(std::vector<point2D>& vPts);

	void getRealConnectComponent(vector<vector<int>> index, vector<point2D> PointDict, vector<vector<point2D>> &coord);

	//������ת��Ϊ��ʱ��
	void adjustContourCCW(vector<vector<point2D>> &coord);

	//����NewEdgeTable
	void ConstructNewEdgeTable(float yMin, float yMax, vector<vector<NewEdgeTable>> &net, vector<point2D> modelPolygon, vector<vector<point2D>> texturePolygon);
	
	bool isEdgeInAET(vector<ActiveEdgeTable> aet, NewEdgeTable newEdge);

	void fillPoly(vector<point2D> modelPolygon, vector<vector<point2D>> texturePolygon, vector<vector<point2D>> &intersectPoint);
	
	void generateTPMSfield();
};

