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
	float x;//扫描线与表的交点坐标
	float dletaX;//从当前扫描线到下一条扫描线间x的增量，斜率的倒数
	float yMax;//该边的最大y值
	float yMin;//该边的最小y值
	ActiveEdgeTable *next;//指向下一条边的指针
};

struct NewEdgeTable
{
	float yMax;//该边的最大y值
	float xMin;//该边较低点的x坐标值
	float yMin;//该边较低点的y坐标值
	float invSlope;//斜率的倒数
	NewEdgeTable *next;//指向下一条边的指针
};

class SolidTexture
{
public:
	SolidTexture();
	~SolidTexture();

	//根据texture field更新Gcode
	void generateNewGcode(Gcode & currGcode, Gcode &newGcode);
	void generateNewGcodeTPMS(Gcode & currGcode, Gcode &newGcode);

	//等间距加密采样点
	void densifyGcode(Gcode &currGcode);
	void insertPoint(float length, point2D v1, point2D v2, point2D &addPoint);

	//计算两点间距离的函数
	float getLength(point2D v1, point2D v2) { return sqrt((v2.x - v1.x)*(v2.x - v1.x) + (v2.y - v1.y)*(v2.y - v1.y)); };


	//读取texture field
	//void readinTextureField(string filepath);
	void readinTextureField2(string filepath);


	//保存solidtexture的数组
	//double ***TextureField;
	vector<vector<double>> textureField;
	vector<vector<double>> tpmsField;

	//从marching squares输出的线段中得到各个连通的轮廓,

	//判断给定的点是否在dict中，如果在给出index，不在返回-1
	int isInDict(vector<point2D> &outPointDict, point2D queryPoint);
	//给定的texturePoints中含有冗余的点，将它用索引+坐标的方式压缩表示
	void getPointsDictionary(vector<point2D> inTexturePoints, vector<point2D> &outPointDict, vector<int> &outPointIndex);
	//建立每条边对应的顶点的数组
	void getPointIdxEachLine(vector<int> inPointIndex, vector<Line> &outLine);
	//找到每个顶点相邻的顶点
	void getlineByPoint(vector<Line> inLine, map<int, vector<int>> &pointNeighbor);
	
	//构建连通分支
	//输出ConnectedComponent，记录了各个连通分支
	void constructConnectedComponent(map<int, vector<int>> pointNeighbor, vector<vector<int>> &ConnectedComponent);
	
	//判断多边形是否为顺时针
	bool IsPolyClockwise(std::vector<point2D>& vPts);

	void getRealConnectComponent(vector<vector<int>> index, vector<point2D> PointDict, vector<vector<point2D>> &coord);

	//将轮廓转化为逆时针
	void adjustContourCCW(vector<vector<point2D>> &coord);

	//构建NewEdgeTable
	void ConstructNewEdgeTable(float yMin, float yMax, vector<vector<NewEdgeTable>> &net, vector<point2D> modelPolygon, vector<vector<point2D>> texturePolygon);
	
	bool isEdgeInAET(vector<ActiveEdgeTable> aet, NewEdgeTable newEdge);

	void fillPoly(vector<point2D> modelPolygon, vector<vector<point2D>> texturePolygon, vector<vector<point2D>> &intersectPoint);
	
	void generateTPMSfield();
};

