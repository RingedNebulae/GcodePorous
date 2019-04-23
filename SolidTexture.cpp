#include "SolidTexture.h"



SolidTexture::SolidTexture()
{
}


SolidTexture::~SolidTexture()
{
}


//生成内部带孔的gcode的主要逻辑
//1.从每一层提取wall-outer,wall-inner,skin部分,wall-outer,skin部分保留,根据wall-inner部分计算infill
//2.mainbady部分的E值全部重新计算
//3.每一层先用marching squares算法对texture field得到等值线
//4.根据等值线建立轮廓C
//5.根据轮廓C与模型轮廓G(wall-inner)的关系将C分类,对于在C在G外部的,舍去
//6.对于C与G相交的,执行G-C
//7.对于C在G内部的,执行多边形扫描转换
void SolidTexture::generateNewGcode(Gcode & currGcode)
{
	MarchingSquares Ms;
	vector<point2D> texturePoints;
	vector<point2D> pointDict;
	vector<int> pointIndex;
	vector<Line> line;//每条边对应的顶点数组
	map<int, vector<int>> pointNeighbor;
	vector<vector<int>> ConnectedComponent;//点的索引构成的轮廓数组
	vector<vector<point2D>> coord;//点实际坐标构成的轮廓数组

	//对currGcode中wallouter部分进行处理
	for (int i = 0; i < currGcode.mainBody.size(); i++)
	{
		cout << "layer " << i << endl;

		//对于每一层先用marchingsquares算法从texture field中得到线段保存在texturePoints中
		//int textureIdx = i / 10;//使得每10层对应于field中的每一层
		int textureIdx = i;

		Ms.createMarchingSquares(textureField.at(textureIdx), texturePoints);

		//
		getPointsDictionary(texturePoints, pointDict, pointIndex);
		getPointIdxEachLine(pointIndex, line);
		getlineByPoint(line, pointNeighbor);

		//得到连通分支
		constructConnectedComponent(pointNeighbor, ConnectedComponent);
		getRealConnectComponent(ConnectedComponent, pointDict, coord);
		
		//保证轮廓是逆时针存储
		adjustContourCCW(coord);

		for (int j = 0; j < currGcode.mainBody.at(i).contourContainer.size(); j++)
		{
			for (int k = 0; k < currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer.size(); k++)
			{
				eachLine tmpline = currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k);

				if (tmpline.isGcommand
					&& tmpline.GcommandLine.Gcommand == "G1"
					&& tmpline.codetype == WALL_OUTER
					&& tmpline.GcommandLine.X.isValidValue
					&& tmpline.GcommandLine.Y.isValidValue
					&& tmpline.GcommandLine.E.isValidValue)
				{
					tmpPoint.x = tmpline.GcommandLine.X.value;
					tmpPoint.y = tmpline.GcommandLine.Y.value;

					//在TextureGcode中获取距离当前点距离最近的点
					newPoint = getClosestPoint(texturePoints, tmpPoint);

					float tmpDist = getLength(tmpPoint, newPoint);
					//cout << tmpDist << endl;

					if (tmpDist < offsetDist_upperBound)
					{
						//将当前点移动到新的位置
						currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k).GcommandLine.X.value = newPoint.x;
						currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k).GcommandLine.Y.value = newPoint.y;
					}
					else if (tmpDist >= offsetDist_upperBound)
					{
						//计算将点沿方向，移动到upper bound之后对应的点
						insertPoint(offsetDist_upperBound, tmpPoint, newPoint, addPoint);
						//cout << "move to upper bound" << endl;
						//将点沿方向，移动到upperbound
						currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k).GcommandLine.X.value = addPoint.x;
						currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k).GcommandLine.Y.value = addPoint.y;
					}
				}
			}
		}
		//清理变量，准备新的一层
		texturePoints.clear();
	}
}

void SolidTexture::densifyGcode(Gcode & currGcode)
{
	bool isFirstPoint;
	//const float pointGapLength = 0.2;//将gcode中点的间距离散为0.2mm
	const float pointGapLength = 0.15;

	point2D startPosition;//线段起点
	point2D endPosition;//线段终点
	point2D addPoint;

	vector<eachLine> tmpWallOuterContainer;
	eachLine addLine;

	for (int i = 0; i < currGcode.mainBody.size(); i++)
	{
		//将gcode中的线段等距变密
		for (int j = 0; j < currGcode.mainBody.at(i).contourContainer.size(); j++)
		{
			isFirstPoint = true;

			//得到实际起点
			point2D acturalStartPoint = currGcode.findActuralStartPoint(currGcode,i-1,i,j);

			for (int k = 0; k < currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer.size(); k++)
			{
				eachLine tmpline = currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k);

				addLine = tmpline;

				if (isFirstPoint)
				{
					startPosition = acturalStartPoint;
					isFirstPoint = false;
					tmpWallOuterContainer.push_back(tmpline);
				}

				if (tmpline.isGcommand
					&& tmpline.GcommandLine.Gcommand == "G1"
					&& tmpline.codetype == WALL_OUTER
					&& tmpline.GcommandLine.X.isValidValue
					&& tmpline.GcommandLine.Y.isValidValue
					&& tmpline.GcommandLine.E.isValidValue)
				{
					endPosition.x = tmpline.GcommandLine.X.value;
					endPosition.y = tmpline.GcommandLine.Y.value;
					endPosition.e = tmpline.GcommandLine.E.value;

					//计算当前线段长度
					float tmpLineLength = getLength(startPosition, endPosition);
					//计算需要添加多少个点
					int addPointNum = tmpLineLength / pointGapLength;
					//添加点
					for (int i = 1; i < addPointNum; i++)
					{
						insertPoint(pointGapLength*i, startPosition, endPosition, addPoint);
						addLine.GcommandLine.X.value = addPoint.x;
						addLine.GcommandLine.Y.value = addPoint.y;
						addLine.GcommandLine.E.value = addPoint.e;
						tmpWallOuterContainer.push_back(addLine);
					}
					//更新起点
					startPosition = endPosition;
					tmpWallOuterContainer.push_back(tmpline);
				}
				else if (tmpline.isGcommand
					&&tmpline.GcommandLine.Gcommand == "G0")
				{
					tmpWallOuterContainer.push_back(tmpline);
				}

			}
			currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer = tmpWallOuterContainer;
			//清理变量，准备新的轮廓
			tmpWallOuterContainer.clear();
		}
	}

}

void SolidTexture::insertPoint(float length, point2D v1, point2D v2, point2D & addPoint)
{
	float tmpLength = getLength(v1, v2);//v1到v2的距离

	addPoint.x = v1.x + length*(v2.x - v1.x) / tmpLength;
	addPoint.y = v1.y + length*(v2.y - v1.y) / tmpLength;
	addPoint.e = v1.e + length*(v2.e - v1.e) / tmpLength;
}

void SolidTexture::readinTextureField2(string filepath)
{
	//solidTexture分辨率
	int resolutionX = 200;
	int resolutionY = 200;
	int resolutionZ = 200;

	//定义输入变量
	double inputbuffer;//为了读入文件的便于格式转化
	vector<double> tmpVector;

	//读取文件
	ifstream openTextureField(filepath);
	if (!openTextureField.is_open())
	{
		cout << "Error opening file!";
		return;
	}
	
	for (int k = 0; k < resolutionZ; k++)
	{
		for (int i = 0; i < resolutionX*resolutionY; i++)
		{
			openTextureField >> inputbuffer;
			tmpVector.push_back(inputbuffer);
		}
		textureField.push_back(tmpVector);
		tmpVector.clear();
	}

}

int SolidTexture::isInDict(vector<point2D>& outPointDict, point2D queryPoint)
{
	for (int i = 0; i < outPointDict.size(); i++)
	{
		if (fabs(outPointDict.at(i).x - queryPoint.x) < 10e-5
			&&fabs(outPointDict.at(i).y - queryPoint.y) < 10e-5)
			return i;
	}

	return false - 1;
}

void SolidTexture::getPointsDictionary(vector<point2D> inTexturePoints, vector<point2D>& outPointDict, vector<int>& outPointIndex)
{
	point2D tmpPoint;
	int dictSize = 0;

	//texturePoints中保存的是ms重建得到的所有顶点
	for (int i = 0; i < inTexturePoints.size(); i++)
	{
		tmpPoint = inTexturePoints.at(i);
		int index = isInDict(outPointDict, tmpPoint);
		if (index >= 0)
		{
			//dict中已经有这个点了不做处理
			outPointIndex.push_back(index);
			continue;
		}
		else
		{
			//没有找到的话加进去,并记录该点的index
			outPointDict.push_back(tmpPoint);
			outPointIndex.push_back(dictSize);
			dictSize++;
		}
	}
}

//建立每条边对应的顶点的数组
void SolidTexture::getPointIdxEachLine(vector<int> inPointIndex, vector<Line>& outLine)
{
	Line tmpline;
	for (int i = 0; i < inPointIndex.size(); i + 2)
	{
		tmpline.p1 = inPointIndex.at(i);
		tmpline.p2 = inPointIndex.at(i + 1);
		outLine.push_back(tmpline);
	}
}

void SolidTexture::getlineByPoint(vector<Line> inLine, map<int, vector<int>> &pointNeighbor)
{
	int pointIndex1, pointIndex2;
	for (int i = 0; i < inLine.size(); i++)
	{
		pointIndex1 = inLine.at(i).p1;
		pointIndex2 = inLine.at(i).p2;
		pointNeighbor[pointIndex1].push_back(pointIndex2);
		pointNeighbor[pointIndex2].push_back(pointIndex1);
	}
}

void SolidTexture::constructConnectedComponent(map<int, vector<int>> pointNeighbor, vector<vector<int>>& ConnectedComponent)
{
	vector<bool> isVisited(pointNeighbor.size(), false);
	int unVisitedSize = isVisited.size();

	bool isNewConnectedComponent = true;

	int startPointIdx;
	int nextPointIdx;

	while (unVisitedSize != 0)
	{
		vector<int> tmpComponent;

		if (isNewConnectedComponent)
		{
			for (int i = isVisited.size() - 1; i >0; i--)
			{
				//对于每一个起点，加入连通分支，标记为已访问，未访问点数减一
				startPointIdx = i;
				tmpComponent.push_back(startPointIdx);
				isVisited.at(startPointIdx) = true;
				unVisitedSize--;
				isNewConnectedComponent = false;
			}
		}

		//当前点包含哪些邻居
		vector<int> currPointNeighbor = pointNeighbor.find(startPointIdx)->second;
		if (currPointNeighbor.size() > 2)
		{
			//用于debug
			cout << "Error! Vertex has more than 2 neighors!!" << endl;
		}

		//从一个点出发，直到回到该点,得到一个连通分支
		while (true)
		{
			bool isStop = false;
			//确定下一个查找的点
			for (int i = 0; i < currPointNeighbor.size(); i++)
			{
				int tmpIdx = currPointNeighbor.at(i);
				if (isVisited.at(tmpIdx) == false)
				{
					nextPointIdx = tmpIdx;
					isVisited.at(nextPointIdx) = true;
					tmpComponent.push_back(nextPointIdx);
					break;
				}
				//退出循环条件，当前点的相邻点都被访问过了
				if (i == currPointNeighbor.size() - 1)
				{
					isStop = true;
				}
			}

			if (isStop)
			{
				//找到一个连通分支，保存，找下一个
				ConnectedComponent.push_back(tmpComponent);
				isNewConnectedComponent = true;
				break;
			}
			else
			{
				currPointNeighbor = pointNeighbor.find(nextPointIdx)->second;
			}

		}

	}
}

void SolidTexture::getRealConnectComponent(vector<vector<int>> index, vector<point2D> PointDict, vector<vector<point2D>>& coord)
{
	int tmpIndex;

	for (int i = 0; i < index.size(); i++)
	{
		vector<point2D> tmpCoord;
		for (int j = 0; j < index.at(i).size(); j++)
		{
			tmpIndex = index.at(i).at(j);

			tmpCoord.push_back(PointDict.at(tmpIndex));
		}
		coord.push_back(tmpCoord);
	}
}

bool SolidTexture::IsPolyClockwise(std::vector<point2D>& vPts)
{
	//沿着多边形的边求曲线积分,若积分为正,则是沿着边界曲线正方向(逆时针),反之为顺时针
	double d = 0;
	const size_t nSize = vPts.size();
	for (int i = 0; i < nSize - 1; ++i)
	{
		d += -0.5 * (vPts[i + 1].y + vPts[i].y)*(vPts[i + 1].x - vPts[i].x);
	}

	//这条边不能忘记
	d += -0.5 * (vPts[0].y + vPts[nSize - 1].y)*(vPts[0].x - vPts[nSize - 1].x);

	//小于零为顺时针，大于零为逆时针
	return d < 0.0;;
}

void SolidTexture::adjustContourCCW(vector<vector<point2D>>& coord)
{
	vector<vector<point2D>> tmpcoord;
	for (int i = 0; i < coord.size(); i++)
	{
		vector<point2D> tmpContour = coord.at(i);
		vector<point2D> reverseContour;
		if (IsPolyClockwise(tmpContour))
		{
			//如果是顺时针，那么对轮廓中点的顺序做修改
			for (auto rit = tmpContour.rbegin(); rit != tmpContour.rend(); rit++)
			{
				reverseContour.push_back(*rit);
			}
			tmpcoord.push_back(reverseContour);
		}
		else
		{
			//如果是逆时针，不做处理，直接保存
			tmpcoord.push_back(tmpContour);
		}
	}
	//更新
	coord = tmpcoord;
}

void SolidTexture::ConstructNewEdgeTable(float yMin, float yMax, vector<vector<NewEdgeTable>>& net, vector<point2D> modelPolygon, vector<vector<point2D>> texturePolygon)
{
	float tmpy;
	point2D p1, p2;
	NewEdgeTable node;

	//相邻两点构成一条边
	//找到各边的最小y值，确定该边在net中的那个位置出现
	for (int i = 0; i < modelPolygon.size(); i++)
	{
		p1 = modelPolygon.at(i);

		if (i == modelPolygon.size() - 1)
		{
			//最后一个点的特殊处理
			p2 = modelPolygon.at(0);
		}
		else
		{
			p2 = modelPolygon.at(i + 1);
		}

		if (p1.y < p2.y)
		{
			tmpy = p1.y;
			node.xMin = p1.x;
			node.yMax = p2.y;
		}
		else
		{
			tmpy = p2.y;
			node.xMin = p2.x;
			node.yMax = p1.y;
		}
		node.invSlope = (p2.y - p1.y) / (p2.x - p1.x);
		//得到当前线段的最低点y
		tmpy = p1.y < p2.y ? p1.y : p2.y;

		//计算当前边从第几条扫描线开始参与运算
		int scanlineIdx = (tmpy - yMin) / scanLineGap;

		net.at(scanlineIdx).push_back(node);
	}

	//同样方法对texturePolygon做处理
	for (int i = 0; i < texturePolygon.size(); i++)
	{
		for (int j = 0; j < texturePolygon.at(i).size(); j++)
		{
			p1 = texturePolygon.at(i).at(j);

			if (i == texturePolygon.at(i).size() - 1)
			{
				//最后一个点的特殊处理
				p2 = texturePolygon.at(i).at(0);
			}
			else
			{
				p2 = texturePolygon.at(i).at(j + 1);
			}

			if (p1.y < p2.y)
			{
				tmpy = p1.y;
				node.xMin = p1.x;
				node.yMax = p2.y;
			}
			else
			{
				tmpy = p2.y;
				node.xMin = p2.x;
				node.yMax = p1.y;
			}
			node.invSlope = (p2.y - p1.y) / (p2.x - p1.x);
			//得到当前线段的最低点y
			tmpy = p1.y < p2.y ? p1.y : p2.y;

			//计算当前边从第几条扫描线开始参与运算
			int scanlineIdx = (tmpy - yMin) / scanLineGap;

			net.at(scanlineIdx).push_back(node);
		}
	}
}

void SolidTexture::fillPoly(vector<point2D> modelPolygon, vector<vector<point2D>> texturePolygon)
{
	//1.找到y的最大最小值
	//这里的texturePolygon是确保在modelPolygon内部的
	float yMin = 99999, yMax = -99999;
	float tmpy;

	for (int i = 0; i < modelPolygon.size(); i++)
	{
		tmpy = modelPolygon.at(i).y;
		if (tmpy < yMin)
		{
			yMin = tmpy;
		}
		else if (tmpy > yMax)
		{
			yMax = tmpy;
		}
	}

	//2.计算扫描线条数，构建新边表
	int scanLineNum = (yMax - yMin) / scanLineGap;

	//新边表对应的数据结构是：当前扫描线有哪些边新加入活性边
	vector<vector<NewEdgeTable>> net(scanLineNum);

	ConstructNewEdgeTable(yMin, yMax, net, modelPolygon, texturePolygon);

	//3.初始化活性边表
	vector<ActiveEdgeTable> aet, aet2;

	//4.依次处理各扫描线
	vector<vector<point2D>> intersectPoint;//保存扫描线与边界的交点

	for (int i = 0; i < scanLineNum; i++)
	{
		vector<point2D> tmpIntersectPoint;
		ActiveEdgeTable node;

		//1.判断aet中的边是否需要被去除
		tmpy = yMin + i*scanLineGap;//当前扫描线对应的y值
		for (int j = 0; j < aet.size(); j++)
		{
			if (aet.at(j).yMax > tmpy)
			{
				//保留当前边为活性边
				aet2.push_back(aet.at(j));
			}

		}
		aet = aet2;//更新活性边表

				   //2.如果不去除，更新x值
		for (int j = 0; j < aet.size(); j++)
		{
			aet.at(j).x += aet.at(j).dletaX*scanLineGap;
			point2D tmpPoint;
			tmpPoint.x = aet.at(j).x;
			tmpPoint.y = yMin + i*scanLineGap;

			tmpIntersectPoint.push_back(tmpPoint);
		}

		//把交点保存
		intersectPoint.push_back(tmpIntersectPoint);

		//3.看有没有新的边加入
		if (net.at(i).size()>0)
		{
			NewEdgeTable newEdge;
			for (int j = 0; j < net.at(i).size(); j++)
			{
				newEdge = net.at(i).at(j);
				node.x = newEdge.xMin;
				node.dletaX = newEdge.invSlope;
				node.yMax = newEdge.yMax;

				aet.push_back(node);
			}
		}
	}
}

