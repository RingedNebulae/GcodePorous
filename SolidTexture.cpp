#include "SolidTexture.h"



SolidTexture::SolidTexture()
{
}


SolidTexture::~SolidTexture()
{
}


//�����ڲ����׵�gcode����Ҫ�߼�
//1.��ÿһ����ȡwall-outer,wall-inner,skin����,wall-outer,skin���ֱ���,����wall-inner���ּ���infill
//2.mainbady���ֵ�Eֵȫ�����¼���
//3.ÿһ������marching squares�㷨��texture field�õ���ֵ��
//4.���ݵ�ֵ�߽�������C
//5.��������C��ģ������G(wall-inner)�Ĺ�ϵ��C����,������C��G�ⲿ��,��ȥ
//6.����C��G�ཻ��,ִ��G-C
//7.����C��G�ڲ���,ִ�ж����ɨ��ת��
void SolidTexture::generateNewGcode(Gcode & currGcode)
{
	MarchingSquares Ms;
	vector<point2D> texturePoints;
	vector<point2D> pointDict;
	vector<int> pointIndex;
	vector<Line> line;//ÿ���߶�Ӧ�Ķ�������
	map<int, vector<int>> pointNeighbor;
	vector<vector<int>> ConnectedComponent;//����������ɵ���������
	vector<vector<point2D>> coord;//��ʵ�����깹�ɵ���������

	//��currGcode��wallouter���ֽ��д���
	for (int i = 0; i < currGcode.mainBody.size(); i++)
	{
		cout << "layer " << i << endl;

		//����ÿһ������marchingsquares�㷨��texture field�еõ��߶α�����texturePoints��
		//int textureIdx = i / 10;//ʹ��ÿ10���Ӧ��field�е�ÿһ��
		int textureIdx = i;

		Ms.createMarchingSquares(textureField.at(textureIdx), texturePoints);

		//
		getPointsDictionary(texturePoints, pointDict, pointIndex);
		getPointIdxEachLine(pointIndex, line);
		getlineByPoint(line, pointNeighbor);

		//�õ���ͨ��֧
		constructConnectedComponent(pointNeighbor, ConnectedComponent);
		getRealConnectComponent(ConnectedComponent, pointDict, coord);
		
		//��֤��������ʱ��洢
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

					//��TextureGcode�л�ȡ���뵱ǰ���������ĵ�
					newPoint = getClosestPoint(texturePoints, tmpPoint);

					float tmpDist = getLength(tmpPoint, newPoint);
					//cout << tmpDist << endl;

					if (tmpDist < offsetDist_upperBound)
					{
						//����ǰ���ƶ����µ�λ��
						currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k).GcommandLine.X.value = newPoint.x;
						currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k).GcommandLine.Y.value = newPoint.y;
					}
					else if (tmpDist >= offsetDist_upperBound)
					{
						//���㽫���ط����ƶ���upper bound֮���Ӧ�ĵ�
						insertPoint(offsetDist_upperBound, tmpPoint, newPoint, addPoint);
						//cout << "move to upper bound" << endl;
						//�����ط����ƶ���upperbound
						currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k).GcommandLine.X.value = addPoint.x;
						currGcode.mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k).GcommandLine.Y.value = addPoint.y;
					}
				}
			}
		}
		//����������׼���µ�һ��
		texturePoints.clear();
	}
}

void SolidTexture::densifyGcode(Gcode & currGcode)
{
	bool isFirstPoint;
	//const float pointGapLength = 0.2;//��gcode�е�ļ����ɢΪ0.2mm
	const float pointGapLength = 0.15;

	point2D startPosition;//�߶����
	point2D endPosition;//�߶��յ�
	point2D addPoint;

	vector<eachLine> tmpWallOuterContainer;
	eachLine addLine;

	for (int i = 0; i < currGcode.mainBody.size(); i++)
	{
		//��gcode�е��߶εȾ����
		for (int j = 0; j < currGcode.mainBody.at(i).contourContainer.size(); j++)
		{
			isFirstPoint = true;

			//�õ�ʵ�����
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

					//���㵱ǰ�߶γ���
					float tmpLineLength = getLength(startPosition, endPosition);
					//������Ҫ���Ӷ��ٸ���
					int addPointNum = tmpLineLength / pointGapLength;
					//���ӵ�
					for (int i = 1; i < addPointNum; i++)
					{
						insertPoint(pointGapLength*i, startPosition, endPosition, addPoint);
						addLine.GcommandLine.X.value = addPoint.x;
						addLine.GcommandLine.Y.value = addPoint.y;
						addLine.GcommandLine.E.value = addPoint.e;
						tmpWallOuterContainer.push_back(addLine);
					}
					//�������
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
			//����������׼���µ�����
			tmpWallOuterContainer.clear();
		}
	}

}

void SolidTexture::insertPoint(float length, point2D v1, point2D v2, point2D & addPoint)
{
	float tmpLength = getLength(v1, v2);//v1��v2�ľ���

	addPoint.x = v1.x + length*(v2.x - v1.x) / tmpLength;
	addPoint.y = v1.y + length*(v2.y - v1.y) / tmpLength;
	addPoint.e = v1.e + length*(v2.e - v1.e) / tmpLength;
}

void SolidTexture::readinTextureField2(string filepath)
{
	//solidTexture�ֱ���
	int resolutionX = 200;
	int resolutionY = 200;
	int resolutionZ = 200;

	//�����������
	double inputbuffer;//Ϊ�˶����ļ��ı��ڸ�ʽת��
	vector<double> tmpVector;

	//��ȡ�ļ�
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

	//texturePoints�б������ms�ؽ��õ������ж���
	for (int i = 0; i < inTexturePoints.size(); i++)
	{
		tmpPoint = inTexturePoints.at(i);
		int index = isInDict(outPointDict, tmpPoint);
		if (index >= 0)
		{
			//dict���Ѿ���������˲�������
			outPointIndex.push_back(index);
			continue;
		}
		else
		{
			//û���ҵ��Ļ��ӽ�ȥ,����¼�õ��index
			outPointDict.push_back(tmpPoint);
			outPointIndex.push_back(dictSize);
			dictSize++;
		}
	}
}

//����ÿ���߶�Ӧ�Ķ��������
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
				//����ÿһ����㣬������ͨ��֧�����Ϊ�ѷ��ʣ�δ���ʵ�����һ
				startPointIdx = i;
				tmpComponent.push_back(startPointIdx);
				isVisited.at(startPointIdx) = true;
				unVisitedSize--;
				isNewConnectedComponent = false;
			}
		}

		//��ǰ�������Щ�ھ�
		vector<int> currPointNeighbor = pointNeighbor.find(startPointIdx)->second;
		if (currPointNeighbor.size() > 2)
		{
			//����debug
			cout << "Error! Vertex has more than 2 neighors!!" << endl;
		}

		//��һ���������ֱ���ص��õ�,�õ�һ����ͨ��֧
		while (true)
		{
			bool isStop = false;
			//ȷ����һ�����ҵĵ�
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
				//�˳�ѭ����������ǰ������ڵ㶼�����ʹ���
				if (i == currPointNeighbor.size() - 1)
				{
					isStop = true;
				}
			}

			if (isStop)
			{
				//�ҵ�һ����ͨ��֧�����棬����һ��
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
	//���Ŷ���εı������߻���,������Ϊ��,�������ű߽�����������(��ʱ��),��֮Ϊ˳ʱ��
	double d = 0;
	const size_t nSize = vPts.size();
	for (int i = 0; i < nSize - 1; ++i)
	{
		d += -0.5 * (vPts[i + 1].y + vPts[i].y)*(vPts[i + 1].x - vPts[i].x);
	}

	//�����߲�������
	d += -0.5 * (vPts[0].y + vPts[nSize - 1].y)*(vPts[0].x - vPts[nSize - 1].x);

	//С����Ϊ˳ʱ�룬������Ϊ��ʱ��
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
			//�����˳ʱ�룬��ô�������е��˳�����޸�
			for (auto rit = tmpContour.rbegin(); rit != tmpContour.rend(); rit++)
			{
				reverseContour.push_back(*rit);
			}
			tmpcoord.push_back(reverseContour);
		}
		else
		{
			//�������ʱ�룬����������ֱ�ӱ���
			tmpcoord.push_back(tmpContour);
		}
	}
	//����
	coord = tmpcoord;
}

void SolidTexture::ConstructNewEdgeTable(float yMin, float yMax, vector<vector<NewEdgeTable>>& net, vector<point2D> modelPolygon, vector<vector<point2D>> texturePolygon)
{
	float tmpy;
	point2D p1, p2;
	NewEdgeTable node;

	//�������㹹��һ����
	//�ҵ����ߵ���Сyֵ��ȷ���ñ���net�е��Ǹ�λ�ó���
	for (int i = 0; i < modelPolygon.size(); i++)
	{
		p1 = modelPolygon.at(i);

		if (i == modelPolygon.size() - 1)
		{
			//���һ��������⴦��
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
		//�õ���ǰ�߶ε���͵�y
		tmpy = p1.y < p2.y ? p1.y : p2.y;

		//���㵱ǰ�ߴӵڼ���ɨ���߿�ʼ��������
		int scanlineIdx = (tmpy - yMin) / scanLineGap;

		net.at(scanlineIdx).push_back(node);
	}

	//ͬ��������texturePolygon������
	for (int i = 0; i < texturePolygon.size(); i++)
	{
		for (int j = 0; j < texturePolygon.at(i).size(); j++)
		{
			p1 = texturePolygon.at(i).at(j);

			if (i == texturePolygon.at(i).size() - 1)
			{
				//���һ��������⴦��
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
			//�õ���ǰ�߶ε���͵�y
			tmpy = p1.y < p2.y ? p1.y : p2.y;

			//���㵱ǰ�ߴӵڼ���ɨ���߿�ʼ��������
			int scanlineIdx = (tmpy - yMin) / scanLineGap;

			net.at(scanlineIdx).push_back(node);
		}
	}
}

void SolidTexture::fillPoly(vector<point2D> modelPolygon, vector<vector<point2D>> texturePolygon)
{
	//1.�ҵ�y�������Сֵ
	//�����texturePolygon��ȷ����modelPolygon�ڲ���
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

	//2.����ɨ���������������±߱�
	int scanLineNum = (yMax - yMin) / scanLineGap;

	//�±߱���Ӧ�����ݽṹ�ǣ���ǰɨ��������Щ���¼�����Ա�
	vector<vector<NewEdgeTable>> net(scanLineNum);

	ConstructNewEdgeTable(yMin, yMax, net, modelPolygon, texturePolygon);

	//3.��ʼ�����Ա߱�
	vector<ActiveEdgeTable> aet, aet2;

	//4.���δ�����ɨ����
	vector<vector<point2D>> intersectPoint;//����ɨ������߽�Ľ���

	for (int i = 0; i < scanLineNum; i++)
	{
		vector<point2D> tmpIntersectPoint;
		ActiveEdgeTable node;

		//1.�ж�aet�еı��Ƿ���Ҫ��ȥ��
		tmpy = yMin + i*scanLineGap;//��ǰɨ���߶�Ӧ��yֵ
		for (int j = 0; j < aet.size(); j++)
		{
			if (aet.at(j).yMax > tmpy)
			{
				//������ǰ��Ϊ���Ա�
				aet2.push_back(aet.at(j));
			}

		}
		aet = aet2;//���»��Ա߱�

				   //2.�����ȥ��������xֵ
		for (int j = 0; j < aet.size(); j++)
		{
			aet.at(j).x += aet.at(j).dletaX*scanLineGap;
			point2D tmpPoint;
			tmpPoint.x = aet.at(j).x;
			tmpPoint.y = yMin + i*scanLineGap;

			tmpIntersectPoint.push_back(tmpPoint);
		}

		//�ѽ��㱣��
		intersectPoint.push_back(tmpIntersectPoint);

		//3.����û���µı߼���
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
