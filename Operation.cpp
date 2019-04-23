#include "Operation.h"



Operation::Operation()
{
}


Operation::~Operation()
{
}

void Operation::convertLine2G0(point2D targetPos, eachLine & currLine)
{
	currLine.isGcommand = true;
	currLine.GcommandLine.Gcommand = "G0";
	currLine.GcommandLine.X.value = targetPos.x;
	currLine.GcommandLine.X.isValidValue = true;
	currLine.GcommandLine.Y.value = targetPos.y;
	currLine.GcommandLine.Y.isValidValue = true;
	currLine.GcommandLine.E.isValidValue = false;
}

void Operation::convertLine2G1(point2D sourcePos, point2D targetPos, eachLine & currLine)
{
	currLine.isGcommand = true;
	currLine.GcommandLine.Gcommand = "G1";
	currLine.GcommandLine.X.value = targetPos.x;
	currLine.GcommandLine.X.isValidValue = true;
	currLine.GcommandLine.Y.value = targetPos.y;
	currLine.GcommandLine.Y.isValidValue = true;
	//更新currE
	currE += getDeltaE(sourcePos, targetPos);
	currLine.GcommandLine.E.value = currE;
	currLine.GcommandLine.E.isValidValue = true;

}

void Operation::convertContour2Gcode(vector<vector<point2D>> texturePolygon, vector<eachLine>& contourGcode)
{
	//初始化
	eachLine currLine;
	currLine.codetype = WALL_OUTER;
	point2D currPoint, prePoint;

	//用G1实现组成轮廓的各线段填充
	for (int i = 0; i < texturePolygon.size(); i++)
	{
		for (int j = 0; j < texturePolygon.at(i).size(); j++)
		{
			currPoint = texturePolygon.at(i).at(j);

			//对于轮廓的第一点做特殊处理
			if (j == 0)
			{
				//先用G0移动到该轮廓
				convertLine2G0(currPoint, currLine);
			}
			else
			{
				convertLine2G1(prePoint, currPoint, currLine);
			}

			prePoint = currPoint;
			contourGcode.push_back(currLine);
		}

		//当遍历完最后一个点之后，回到起点，使轮廓闭合
		currPoint = texturePolygon.at(i).at(0);
		convertLine2G1(prePoint, currPoint, currLine);
		contourGcode.push_back(currLine);
	}
}

void Operation::convertInfillLine2Gcode(vector<vector<point2D>> intersectPoint, vector<eachLine>& inFillGcode)
{
	//初始化
	eachLine currLine;
	currLine.codetype = FILL;
	point2D currPoint, prePoint;

	for (int i = 0; i < intersectPoint.size(); i++)
	{
		for (int j = 0; j < intersectPoint.at(i).size(); j++)
		{
			currPoint = intersectPoint.at(i).at(j);

			//如果是偶数点先G0移动到当前点
			if (j % 2 == 0)
			{
				convertLine2G0(currPoint, currLine);
			}
			else if (j % 2 == 1)
			{
				//如果是奇数点，用G1填充
				convertLine2G1(prePoint, currPoint, currLine);
			}

			prePoint = currPoint;
			inFillGcode.push_back(currLine);
		}
	}
}

void Operation::convertPoint_2ToPoint(Point_2 pIn, Point &pOut)
{
	pOut = Point(pIn.x(), pIn.y());
}

void Operation::convertPoint_2ToPoint(vector<Point_2> pIn, vector<Point> &pOut)
{
	for (int i = 0; i < pIn.size(); i++)
	{
		convertPoint_2ToPoint(pIn.at(i), pOut.at(i));
	}
}

void Operation::convert2CGALpoint(vector<point2D> in, vector<Point_2> &out)
{
	for (int i = 0; i < in.size(); i++)
	{
		out.push_back(Point_2(in.at(i).x, in.at(i).y));
	}
}

void Operation::convert2CGALpoint(vector<vector<point2D>> in, vector<vector<Point_2>> &out)
{
	vector<Point_2> tmpOut;
	for (int i = 0; i < in.size(); i++)
	{
		for (int j = 0; j < in.at(i).size(); j++)
		{
			tmpOut.push_back(Point_2(in.at(i).at(j).x, in.at(i).at(j).y));
		}
		out.push_back(tmpOut);
	}
}

bool Operation::checkPointInside(Point pt, vector<Point> polygon)
{
	switch (CGAL::bounded_side_2(polygon.at(0), polygon.at(polygon.size() - 1), pt, K())) \
	{
	case CGAL::ON_BOUNDED_SIDE:
		return true;//点在polygon内部
		break;
	case CGAL::ON_BOUNDARY:
		return true;//点在polygon边界
		break;
	case CGAL::ON_UNBOUNDED_SIDE:
		return false;//点在polygon外部
		break;
	}
}


int Operation::check2PolygonRelation(vector<Point_2> modelPolygon, vector<Point_2> TexturePolygon)
{
	vector<Point> copy_modelPolygon, copy_TexturePolygon;
	convertPoint_2ToPoint(modelPolygon, copy_modelPolygon);
	convertPoint_2ToPoint(TexturePolygon, copy_TexturePolygon);

	int hasPointInside = 0, hasPointOutside = 0;
	for (int i = 0; i < TexturePolygon.size(); i++)
	{
		bool isInside = checkPointInside(copy_TexturePolygon.at(i), copy_modelPolygon);

		if (isInside == true)
		{
			hasPointInside = 1;
		}
		else
		{
			hasPointOutside = 1;
		}

		if (hasPointInside == 1 && hasPointOutside == 1)
		{
			//如果texturePolygon中有点在modelPolygon内，同时有在外
			//那么texturePolygong与modelPolygon相交
			//return 1
			return 1;
		}

	}
	//如果texturePolygon中所有点都在modelPolygon中
	//那么texturePolygong在modelPolygon内部
	//return 2
	if (hasPointInside == 1 && hasPointOutside == 0)
	{
		return 2;
	}

	//如果texturePolygon中所有点都不在modelPolygon中
	//那么texturePolygong在modelPolygon外部
	//return 3
	else if (hasPointInside == 0 && hasPointOutside == 1)
	{
		return 3;
	}
	else
	{
		//上面的情况都不满足，return 0，报错
		cout << "check 2 Polygon Relation wrong !!" << endl;
		return 0;
	}
}


void Operation::classifyPolygons(vector<point2D> modelPolygon, vector<vector<point2D>> texturePolygons)
{
	vector<Point_2> modelpoly;
	vector<vector<Point_2>> texturePolys;

	vector<vector<Point_2>> intersectPolys;
	vector<vector<Point_2>> outsidePolys;
	vector<vector<Point_2>> insidePolys;


	//1.对输入数据转化为cgal的格式
	convert2CGALpoint(modelPolygon, modelpoly);
	convert2CGALpoint(texturePolygons, texturePolys);

	//2.判断位置关系
	for (int i = 0; i < texturePolygons.size(); i++)
	{
		switch (check2PolygonRelation(modelpoly, texturePolys.at(i)))
		{
		case 1:
			//相交
			intersectPolys.push_back(texturePolys.at(i));
			break;
		case 2:
			//texture在model内部
			insidePolys.push_back(texturePolys.at(i));
			break;
		case 3:
			//texture在model外部
			outsidePolys.push_back(texturePolys.at(i));
			break;
		default:
			break;
		} 
	}
}

//执行pA - pB操作,确保pA,pB都是逆时针存储的
void Operation::doDiffOperate(vector<Point_2> pA, vector<Point_2> pB)
{
	Polygon_2 PolygonA, polygonB;

	//构建多边形
	for (int i = 0; i < pA.size(); i++)
	{
		PolygonA.push_back(pA.at(i));
	}
	for (int i = 0; i < pB.size(); i++)
	{
		polygonB.push_back(pB.at(i));
	}

	Pwh_list_2 symmR;
	Pwh_list_2::const_iterator it;
	CGAL::difference(PolygonA, polygonB, std::back_inserter(symmR));
	it = symmR.begin();
	if (symmR.size()>1)
	{
		cout << "wrong boolean operation!" << endl;
	}
	for (it = symmR.begin(); it != symmR.end(); it++)
	{
		Polygon_2 outerPoly = it->outer_boundary();
	}
}
