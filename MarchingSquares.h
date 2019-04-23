#pragma once

#include <math.h>
#include <iostream>
#include <vector>
#include "Gcode.h"
#include "Isolines.h"
#include "UniformGrid.h"

class MarchingSquares
{
public:
	MarchingSquares();
	~MarchingSquares();

	unsigned getCase(float values[], float isoValue);
	pair<float, float> getEdgeVertex(float point1[2], float point2[2], float value1, float value2, float isoValue);
	pair<float, float> getBottomEdgeVertex(float points[][2], float values[], float isoValue);
	pair<float, float> getTopEdgeVertex(float points[][2], float values[], float isoValue);
	pair<float, float> getRightEdgeVertex(float points[][2], float values[], float isoValue);
	pair<float, float> getLeftEdgeVertex(float points[][2], float values[], float isoValue);

	void createUniformGrid(UniformGrid &grid, vector<double> field);
	void createMarchingSquares(vector<double> field,vector<point2D> &points);

	void getPointsFromIsolines(vector<point2D> &points,Isolines tmpIsolines);

	float xMin = 95, xMax = 145;
	float yMin = 70, yMax = 120;
	int nX = 200, nY = 200;
	
};

