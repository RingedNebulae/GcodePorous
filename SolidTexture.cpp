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
void SolidTexture::generateNewGcode(Gcode & currGcode,Gcode &newGcode)
{
	MarchingSquares Ms;
	vector<point2D> texturePoints;
	vector<point2D> pointDict;
	vector<int> pointIndex;
	vector<Line> line;//每条边对应的顶点数组
	map<int, vector<int>> pointNeighbor;
	vector<vector<int>> ConnectedComponent;//点的索引构成的轮廓数组
	vector<vector<point2D>> coord;//点实际坐标构成的轮廓数组

	//得到模型内轮廓对应的多边形,并保证是逆时针存储
	vector<vector<vector<point2D>>> modelPolygon;
	currGcode.prepareModelPolygon(currGcode, modelPolygon);
	for (int i = 0; i < modelPolygon.size(); i++)
	{
		adjustContourCCW(modelPolygon.at(i));
	}

	newGcode.startGcode = currGcode.startGcode;
	newGcode.endGcode = currGcode.endGcode;

	//对currGcode中wallinner部分进行处理
	for (int i = 0; i < modelPolygon.size(); i++)
	{
		cout << "layer " << i << endl;
		//生成新的Gcode
		LayerInfo newlayer;

		//对于每一层先用marchingsquares算法从texture field中得到线段保存在texturePoints中
		//int textureIdx = i / 10;//使得每10层对应于field中的每一层
		int textureIdx = i;

		Ms.createMarchingSquares(textureField.at(textureIdx), texturePoints);

		//建立点的
		getPointsDictionary(texturePoints, pointDict, pointIndex);
		getPointIdxEachLine(pointIndex, line);
		getlineByPoint(line, pointNeighbor);

		//得到连通分支
		constructConnectedComponent(pointNeighbor, ConnectedComponent);
		getRealConnectComponent(ConnectedComponent, pointDict, coord);
		
		//保证轮廓是逆时针存储
		adjustContourCCW(coord);

		Operation operation;
		
		//逐个轮廓判断位置关系
		for (int j = 0; j < modelPolygon.at(i).size(); j++)
		{
			//gcode轮廓
			contour newContour;
			
			//polyPosition是与给定的模型轮廓相关的
			polyPosition polygons;

			vector<point2D> oneModelPolygon = modelPolygon.at(i).at(j);
			operation.classifyPolygons(oneModelPolygon,polygons,coord);
			vector<Point_2> copy_oneModelPolygon;//要保留的模型轮廓！！
			operation.convert2CGALpoint(oneModelPolygon, copy_oneModelPolygon);

			//检查是不是简单多边形
			vector<vector<Point_2>> acturalintersectPolys;
			for (int idx = 0; idx < polygons.intersectPolys.size(); idx++)
			{
				Polygon_2 testPolygon;
				for (int idxx = 0;idxx < polygons.intersectPolys.at(idx).size();idxx++)
				{
					testPolygon.push_back(polygons.intersectPolys.at(idx).at(idxx));
				}
				if (testPolygon.is_simple())
					acturalintersectPolys.push_back(polygons.intersectPolys.at(idx));
				else
				{
					cout << "non-manifold polygon!" << endl;
				}
			}

			//对于相交的做difference
			for (int k = 0; k <acturalintersectPolys.size(); k++)
			{
				operation.doDiffOperate(copy_oneModelPolygon, acturalintersectPolys.at(k));
			}

			//在modelPolygon内部的texture polygons
			vector<vector<point2D>> insidePolys;

			//数据类型转化
			oneModelPolygon.clear();//先将数组清空，再作为转化后的输入
			operation.convertPoint_2Topoint2D(copy_oneModelPolygon, oneModelPolygon);
			operation.convertPoint_2Topoint2D(polygons.insidePolys, insidePolys);

			vector<vector<point2D>> intersectPoint;//保存扫描线与边界的交点

			//对于在内部的做扫描转换
			fillPoly(oneModelPolygon, insidePolys, intersectPoint);

			//将计算得到的交点按x递增排序
			for (int tmpidx = 0; tmpidx < intersectPoint.size(); tmpidx++)
			{
				sort(intersectPoint.at(tmpidx).begin(), intersectPoint.at(tmpidx).end());
			}

			vector<eachLine> newWallinner;
			vector<eachLine> contourGcode;
			vector<eachLine> inFillGcode;

			//将轮廓线转化为Gcode
			operation.convertContour2Gcode(oneModelPolygon, newWallinner);

			//将内部填充线转化为Gcode
			operation.convertContour2Gcode(insidePolys,contourGcode);

			//将内部填充线转化为gcode表达
			operation.convertInfillLine2Gcode(intersectPoint, inFillGcode);

			//将newWallinner与contourGcode合并
			newWallinner.insert(newWallinner.end(), contourGcode.begin(), contourGcode.end());
			newContour.wallInnerContainer = newWallinner;
			newContour.typeSequence.push_back(WALL_INNER);
			newContour.fillContainer = inFillGcode;
			newContour.typeSequence.push_back(FILL);

			//存入layercontainer中
			newlayer.contourContainer.push_back(newContour);
		}
		newGcode.mainBody.push_back(newlayer);

		//清理变量
		texturePoints.clear();
		pointDict.clear();
		pointIndex.clear();
		line.clear();//每条边对应的顶点数组
		pointNeighbor.clear();
		ConnectedComponent.clear();//点的索引构成的轮廓数组
		coord.clear();//点实际坐标构成的轮廓数组
	}

}

void SolidTexture::generateNewGcodeTPMS(Gcode & currGcode, Gcode & newGcode)
{

	MarchingSquares Ms;
	vector<point2D> texturePoints;
	vector<point2D> pointDict;
	vector<int> pointIndex;
	vector<Line> line;//每条边对应的顶点数组
	map<int, vector<int>> pointNeighbor;
	vector<vector<int>> ConnectedComponent;//点的索引构成的轮廓数组
	vector<vector<point2D>> coord;//点实际坐标构成的轮廓数组

	//得到模型内轮廓对应的多边形,并保证是逆时针存储
	vector<vector<vector<point2D>>> modelPolygon;
	currGcode.prepareModelPolygon(currGcode, modelPolygon);
	for (int i = 0; i < modelPolygon.size(); i++)
	{
		adjustContourCCW(modelPolygon.at(i));
	}

	newGcode.startGcode = currGcode.startGcode;
	newGcode.endGcode = currGcode.endGcode;

	//对currGcode中wallinner部分进行处理
	for (int i = 0; i < modelPolygon.size(); i++)
	{
		cout << "layer " << i << endl;
		//生成新的Gcode
		LayerInfo newlayer;

		//对于每一层先用marchingsquares算法从texture field中得到线段保存在texturePoints中
		//int textureIdx = i / 10;//使得每10层对应于field中的每一层
		int textureIdx = i;

		Ms.createMarchingSquares(tpmsField.at(textureIdx), texturePoints);

		//建立点的
		getPointsDictionary(texturePoints, pointDict, pointIndex);
		getPointIdxEachLine(pointIndex, line);
		getlineByPoint(line, pointNeighbor);

		//得到连通分支
		constructConnectedComponent(pointNeighbor, ConnectedComponent);
		getRealConnectComponent(ConnectedComponent, pointDict, coord);

		//保证轮廓是逆时针存储
		adjustContourCCW(coord);

		Operation operation;

		//逐个轮廓判断位置关系
		for (int j = 0; j < modelPolygon.at(i).size(); j++)
		{
			//gcode轮廓
			contour newContour;

			//polyPosition是与给定的模型轮廓相关的
			polyPosition polygons;

			vector<point2D> oneModelPolygon = modelPolygon.at(i).at(j);
			operation.classifyPolygons(oneModelPolygon, polygons, coord);
			vector<Point_2> copy_oneModelPolygon;//要保留的模型轮廓！！
			operation.convert2CGALpoint(oneModelPolygon, copy_oneModelPolygon);

			//检查是不是简单多边形
			vector<vector<Point_2>> acturalintersectPolys;
			for (int idx = 0; idx < polygons.intersectPolys.size(); idx++)
			{
				Polygon_2 testPolygon;
				for (int idxx = 0; idxx < polygons.intersectPolys.at(idx).size(); idxx++)
				{
					testPolygon.push_back(polygons.intersectPolys.at(idx).at(idxx));
				}
				if (testPolygon.is_simple())
					acturalintersectPolys.push_back(polygons.intersectPolys.at(idx));
				else
				{
					cout << "non-manifold polygon!" << endl;
				}
			}

			//对于相交的做difference
			for (int k = 0; k <acturalintersectPolys.size(); k++)
			{
				operation.doDiffOperate(copy_oneModelPolygon, acturalintersectPolys.at(k));
			}

			//在modelPolygon内部的texture polygons
			vector<vector<point2D>> insidePolys;

			//数据类型转化
			oneModelPolygon.clear();//先将数组清空，再作为转化后的输入
			operation.convertPoint_2Topoint2D(copy_oneModelPolygon, oneModelPolygon);
			operation.convertPoint_2Topoint2D(polygons.insidePolys, insidePolys);

			vector<vector<point2D>> intersectPoint;//保存扫描线与边界的交点

			//对于在内部的做扫描转换
			fillPoly(oneModelPolygon, insidePolys, intersectPoint);

			//将计算得到的交点按x递增排序
			for (int tmpidx = 0; tmpidx < intersectPoint.size(); tmpidx++)
			{
				sort(intersectPoint.at(tmpidx).begin(), intersectPoint.at(tmpidx).end());
			}

			vector<eachLine> newWallinner;
			vector<eachLine> contourGcode;
			vector<eachLine> inFillGcode;

			//将轮廓线转化为Gcode
			operation.convertContour2Gcode(oneModelPolygon, newWallinner);

			//将内部填充线转化为Gcode
			operation.convertContour2Gcode(insidePolys, contourGcode);

			//将内部填充线转化为gcode表达
			operation.convertInfillLine2Gcode(intersectPoint, inFillGcode);

			//将newWallinner与contourGcode合并
			newWallinner.insert(newWallinner.end(), contourGcode.begin(), contourGcode.end());
			newContour.wallInnerContainer = newWallinner;
			newContour.typeSequence.push_back(WALL_INNER);
			newContour.fillContainer = inFillGcode;
			newContour.typeSequence.push_back(FILL);

			//存入layercontainer中
			newlayer.contourContainer.push_back(newContour);
		}
		newGcode.mainBody.push_back(newlayer);

		//清理变量
		texturePoints.clear();
		pointDict.clear();
		pointIndex.clear();
		line.clear();//每条边对应的顶点数组
		pointNeighbor.clear();
		ConnectedComponent.clear();//点的索引构成的轮廓数组
		coord.clear();//点实际坐标构成的轮廓数组
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
	int j;
	for (int i = 0; i < inPointIndex.size()-1; i+=2)
	{
		tmpline.p1 = inPointIndex.at(i);
		j = i + 1;
		tmpline.p2 = inPointIndex.at(j);
		//如果一条线段，起点与终点相同，不计入这条线段
		//if (inPointIndex.at(i) == inPointIndex.at(j))
		//{
		//	continue;
		//}
		//else
		//{
		outLine.push_back(tmpline);//起点终点相同的线段暂时计入，在构建连通分量时再删去
		//}
	}
}

//返回的pointneighbor，记录了每个顶点有哪些相邻的顶点
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

	while (unVisitedSize > 0)
	{
		vector<int> tmpComponent;

		if (isNewConnectedComponent)
		{
			//如果当前还有点未遍历，且新开始一个连通分量
			//从标记为未遍历的点开始
			for (int i = 0; i < isVisited.size(); i++)
			{
				if (isVisited.at(i) == false)
				{
					//对于每一个起点，加入连通分支，标记为已访问，未访问点数减一
					cout << i << endl;
					startPointIdx = i;
					tmpComponent.push_back(startPointIdx);
					if (startPointIdx >= isVisited.size())
					{
						cout << "startindex" << startPointIdx << "out of range!" << endl;
					}
					isVisited.at(startPointIdx) = true;
					unVisitedSize--;
					isNewConnectedComponent = false;
					break;
				}
			}
		}

		//当前点包含哪些邻居
		map<int, vector<int>>::iterator it = pointNeighbor.find(startPointIdx);
		if (it == pointNeighbor.end())
		{
			cout << "element" << startPointIdx << "don't exitst!" << endl;
			isNewConnectedComponent = true;
			continue;
		}
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
					if (nextPointIdx >= isVisited.size())
					{
						cout << "visit" << nextPointIdx <<"out of range!"<< endl;
					}
					isVisited.at(nextPointIdx) = true;
					unVisitedSize--;
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

	//暂时不考虑外轮廓
	//modelPolygon.clear();

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
			node.yMin = p1.y;
			node.yMax = p2.y;
		}
		else
		{
			tmpy = p2.y;
			node.xMin = p2.x;
			node.yMin = p2.y;
			node.yMax = p1.y;
		}
		node.invSlope = (p2.x - p1.x) / (p2.y - p1.y);
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

			if (j == texturePolygon.at(i).size() - 1)
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
				node.yMin = p1.y;
				node.yMax = p2.y;
			}
			else
			{
				tmpy = p2.y;
				node.xMin = p2.x;
				node.yMin = p2.y;
				node.yMax = p1.y;
			}
			node.invSlope = (p2.x - p1.x) / (p2.y - p1.y);
			//得到当前线段的最低点y
			tmpy = p1.y < p2.y ? p1.y : p2.y;

			//计算当前边从第几条扫描线开始参与运算
			int scanlineIdx = (tmpy - yMin) / scanLineGap;

			net.at(scanlineIdx).push_back(node);
		}
	}
}

bool SolidTexture::isEdgeInAET(vector<ActiveEdgeTable> aet, NewEdgeTable newEdge)
{
	if (aet.size() == 0)
	{
		return false;
	}

	for (int i = 0; i < aet.size(); i++)
	{
		ActiveEdgeTable tmpAet = aet.at(i);
		if (fabs(newEdge.xMin - tmpAet.x) <10e-5
			&&fabs(newEdge.yMax - tmpAet.yMax)<10e-5)
		{
			//如果有的话就return ture了
			return true;
		}
	}
	//如果前面没有return，那么只能在这return false
	return false;
}

void SolidTexture::fillPoly(vector<point2D> modelPolygon, vector<vector<point2D>> texturePolygon, vector<vector<point2D>> &intersectPoint)
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
	//由于除后截断取整，应当加1，保证都被包含
	int scanLineNum =1 + (yMax - yMin) / scanLineGap;

	//新边表对应的数据结构是：当前扫描线有哪些边新加入活性边
	vector<vector<NewEdgeTable>> net(scanLineNum);

	ConstructNewEdgeTable(yMin, yMax, net, modelPolygon, texturePolygon);

	//3.初始化活性边表
	vector<ActiveEdgeTable> aet, aet2;

	//4.依次处理各扫描线
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
		aet2.clear();

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
				//判断当前边是否已在活性边数组中
				if (isEdgeInAET(aet,newEdge))
				{
					continue;
				}
				else
				{
					node.x = newEdge.xMin;
					node.dletaX = newEdge.invSlope;
					node.yMax = newEdge.yMax;
					node.yMin = newEdge.yMin;
					aet2.push_back(node);
				}
			}
		}
		//4.计算新加入的边与扫描线的交点
		for (int j = 0; j < aet2.size(); j++)
		{
			aet2.at(j).x =aet2.at(j).x + aet2.at(j).dletaX*(tmpy-aet2.at(j).yMin);
			point2D tmpPoint;
			tmpPoint.x = aet2.at(j).x;
			tmpPoint.y = yMin + i*scanLineGap;

			tmpIntersectPoint.push_back(tmpPoint);
		}
		aet.insert(aet.end(), aet2.begin(), aet2.end());//将新加入的边存入aet
		aet2.clear();
	}
}

void SolidTexture::generateTPMSfield()
{
	int voxelRes = 200;
	double xmin = 95, ymin = 70, zmin = 0;
	float gap = 0.25;
	double x, y, z;
	double tmpF;

	vector<double> tmpTPMSfield;

	for (int i = 0; i < voxelRes; i++)
	{
		for (int j = 0; j < voxelRes; j++)
		{
			for (int k = 0; k < voxelRes; k++)
			{
				x = xmin + i*gap;
				y = ymin + j*gap;
				z = zmin + k*gap;

				tmpF = 10 * (cos(x)*sin(y) + cos(y)*sin(z) + cos(z)*sin(x))
					- 0.5*(cos(2*x)*cos(2*y) + cos(2*y)*cos(2*z) + cos(2*z)*cos(2*z))
					- 12;
				tmpTPMSfield.push_back(tmpF);
			}
		}
		tpmsField.push_back(tmpTPMSfield);
		tmpTPMSfield.clear();
	}

}

