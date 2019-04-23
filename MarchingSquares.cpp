#include "MarchingSquares.h"

MarchingSquares::MarchingSquares()
{
}


MarchingSquares::~MarchingSquares()
{
}

unsigned MarchingSquares::getCase(float values[], float isoValue)
{
	unsigned bits = 0;
	for (int i = 0; i < 4; i++) {
		if (values[i] > isoValue)
			bits |= 1 << i;
	}
	return bits;
}

pair<float, float> MarchingSquares::getEdgeVertex(float point1[2], float point2[2], float value1, float value2, float isoValue)
{
	pair<float, float> p = pair<float, float>();
	if (value1 < value2) {
		p.first = (point1[0] * (value2 - isoValue) + point2[0] * (isoValue - value1)) / (value2 - value1);
		p.second = (point1[1] * (value2 - isoValue) + point2[1] * (isoValue - value1)) / (value2 - value1);

	}
	else {
		p.first = (point2[0] * (value1 - isoValue) + point1[0] * (isoValue - value2)) / (value1 - value2);
		p.second = (point2[1] * (value1 - isoValue) + point1[1] * (isoValue - value2)) / (value1 - value2);
	}
	return p;
}

pair<float, float> MarchingSquares::getBottomEdgeVertex(float points[][2], float values[], float isoValue)
{
	return getEdgeVertex(points[0], points[1], values[0], values[1], isoValue);
}

pair<float, float> MarchingSquares::getTopEdgeVertex(float points[][2], float values[], float isoValue)
{
	return getEdgeVertex(points[2], points[3], values[2], values[3], isoValue);
}

pair<float, float> MarchingSquares::getRightEdgeVertex(float points[][2], float values[], float isoValue)
{
	return getEdgeVertex(points[1], points[2], values[1], values[2], isoValue);
}

pair<float, float> MarchingSquares::getLeftEdgeVertex(float points[][2], float values[], float isoValue)
{
	return getEdgeVertex(points[3], points[0], values[3], values[0], isoValue);
}

void MarchingSquares::createUniformGrid(UniformGrid & grid, vector<double> field)
{
	for (int i = 0; i < grid.numPoints(); i++)
	{
		float p[2];
		grid.getPoint(i, p);
		float value = field.at(i);

		grid.pointScalars().setC0Scalar(i, value);
	}
}

/*
* Preprocess the isoline verticies
*/
void MarchingSquares::createMarchingSquares(vector<double> field, vector<point2D> &points)
{
	float isoValues[] = { 1 };
	UniformGrid	grid(nX, nY, xMin, yMin, xMax, yMax);

	createUniformGrid(grid,field);

	Isolines isolines(isoValues, 1);

	for (int l = 0; l < isolines.numLines(); l++)
	{
		Isoline * line = new Isoline();
		float isoValue = isolines.getIsolineValue(l);
		for (int i = 0; i < grid.numCells(); i++)
		{
			int cellHolder[4];
			int numVertices = grid.getCell(i, cellHolder);
			float points[4][2];
			float values[4];
			for (int j = 0; j < numVertices; j++)
			{
				grid.getPoint(cellHolder[j], points[j]);
				values[j] = grid.pointScalars().getC0Scalar(cellHolder[j]);
			}

			unsigned bits = getCase(values, isoValue);
			// TODO: Remove repeated cases
			switch ((int)bits) {
			case 0: //0000
				break;
			case 1: //0001
				line->addPoint(getBottomEdgeVertex(points, values, isoValue));
				line->addPoint(getLeftEdgeVertex(points, values, isoValue));
				break;
			case 2: //0010
				line->addPoint(getBottomEdgeVertex(points, values, isoValue));
				line->addPoint(getRightEdgeVertex(points, values, isoValue));
				break;
			case 3: //0011
				line->addPoint(getLeftEdgeVertex(points, values, isoValue));
				line->addPoint(getRightEdgeVertex(points, values, isoValue));
				break;
			case 4: //0100
				line->addPoint(getRightEdgeVertex(points, values, isoValue));
				line->addPoint(getTopEdgeVertex(points, values, isoValue));
				break;
			case 5: //0101
				line->addPoint(getRightEdgeVertex(points, values, isoValue));
				line->addPoint(getTopEdgeVertex(points, values, isoValue));
				line->addPoint(getBottomEdgeVertex(points, values, isoValue));
				line->addPoint(getLeftEdgeVertex(points, values, isoValue));
				break;
			case 6: //0110
				line->addPoint(getBottomEdgeVertex(points, values, isoValue));
				line->addPoint(getTopEdgeVertex(points, values, isoValue));
				break;
			case 7: //0111
				line->addPoint(getTopEdgeVertex(points, values, isoValue));
				line->addPoint(getLeftEdgeVertex(points, values, isoValue));
				break;
			case 8: //1000
				line->addPoint(getTopEdgeVertex(points, values, isoValue));
				line->addPoint(getLeftEdgeVertex(points, values, isoValue));
				break;
			case 9: //1001
				line->addPoint(getBottomEdgeVertex(points, values, isoValue));
				line->addPoint(getTopEdgeVertex(points, values, isoValue));
				break;
			case 10://1010
				line->addPoint(getLeftEdgeVertex(points, values, isoValue));
				line->addPoint(getTopEdgeVertex(points, values, isoValue));
				line->addPoint(getBottomEdgeVertex(points, values, isoValue));
				line->addPoint(getRightEdgeVertex(points, values, isoValue));
				break;
			case 11://1011
				line->addPoint(getRightEdgeVertex(points, values, isoValue));
				line->addPoint(getTopEdgeVertex(points, values, isoValue));
				break;
			case 12://1100
				line->addPoint(getLeftEdgeVertex(points, values, isoValue));
				line->addPoint(getRightEdgeVertex(points, values, isoValue));
				break;
			case 13://1101
				line->addPoint(getBottomEdgeVertex(points, values, isoValue));
				line->addPoint(getRightEdgeVertex(points, values, isoValue));
				break;
			case 14://1110
				line->addPoint(getBottomEdgeVertex(points, values, isoValue));
				line->addPoint(getLeftEdgeVertex(points, values, isoValue));
				break;
			case 15://1111
				break;
			default:
				cout << "WTF!?";
			}
		}
		isolines.setLine(l, line);
	}
	//从重建的线中拿到点的坐标
	getPointsFromIsolines(points, isolines);

}

void MarchingSquares::getPointsFromIsolines(vector<point2D> &points, Isolines tmpIsolines)
{
	point2D tmpPoint;

	int tmpNum = tmpIsolines.getIsolineValues().size();

	for (int i = 0; i < tmpNum; i++)
	{
		for (int j = 0; j < tmpIsolines.getLine(i)->numPoints()-1; j++)
		{
			tmpPoint.x = tmpIsolines.getPoint(i, j).first;
			tmpPoint.y = tmpIsolines.getPoint(i, j).second;
			//将点存入
			points.push_back(tmpPoint);
		}
	}

}
