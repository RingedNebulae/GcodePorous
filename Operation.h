#pragma once

#include "Gcode.h"
#include <cmath>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <list>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef K::Point_2   Point;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;


class Operation
{
public:
	Operation();
	~Operation();

	//���߶�ת��ΪGcode��Ҫ����G1��G0���Լ�E������
	//ȫ��E���޸�
	//��E��Ϊһ��ȫ�ֵĳ�������¼��û�м��뵱ǰ�߶�ǰ��ǰ��Eֵ
	float currE;
	float unitE;//��λ���ȶ�Ӧ��E

	//�����֮��ľ���
	float getLength(point2D v1, point2D v2) { return sqrt((v2.x - v1.x)*(v2.x - v1.x) + (v2.y - v1.y)*(v2.y - v1.y)); };

	//���㵱ǰ�߶ζ�Ӧ��E����
	float getDeltaE(point2D p1, point2D p2) { return unitE*getLength(p1, p2);}

	//��һ���߶�ת��ΪG0
	void convertLine2G0(point2D targetPos, eachLine &currLine);

	//��һ���߶�ת��ΪG1
	void convertLine2G1(point2D sourcePos, point2D targetPos, eachLine &currLine);

	//����׵�������ת��Ϊgcode���
	void convertContour2Gcode(vector<vector<point2D>> texturePolygon, vector<eachLine> &contourGcode);

	//���ڲ������ת��Ϊgcode���
	void convertInfillLine2Gcode(vector<vector<point2D>> intersectPoint, vector<eachLine> &inFillGcode);

	//��Point����ת��ΪPoint_2
	void convertPoint_2ToPoint(Point_2 pIn, Point &pOut);
	void convertPoint_2ToPoint(vector<Point_2> pIn, vector<Point> &pOut);

	//��������ݽṹ��ת��Ϊcgal�����ݽṹ
	void convert2CGALpoint(vector<point2D> in, vector<Point_2> &out);

	//��������ݽṹ��ת��Ϊcgal�����ݽṹ
	void convert2CGALpoint(vector<vector<point2D>> in, vector<vector<Point_2>> &out);
		
	bool checkPointInside(Point pt, vector<Point> polygon);

	int check2PolygonRelation(vector<Point_2> modelPolygon, vector<Point_2> TexturePolygon);

	void classifyPolygons(vector<point2D> modelPolygon, vector<vector<point2D>> texturePolygons);

	void doDiffOperate(vector<Point_2> pA, vector<Point_2> pB);
};

