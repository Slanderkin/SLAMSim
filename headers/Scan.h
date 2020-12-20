#pragma once
#ifndef SCAN_H
#define SCAN_H
#include "World.h"
#include "StandardImports.h"



class Scan {
	

private:
	double mean = 0.0;
	double stddev = 0.001;
	std::mt19937 generator;
	std::normal_distribution<double> dist;
public:

	struct Observation
	{
		std::array<float, 360> theta;
		std::array<float, 360> distance;
	};

	Scan();
	Scan::Observation* performScan(float &cx, float &cy,float &cRad,float &maxRange, const World &world);
	Scan::Observation* computeScanDerivatives(float minDist, Scan::Observation* obs);
	
	float raycast_circle(sf::CircleShape circle, float scanAngle, float x0, float y0);
	float raycast_wall(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4);
	float hz; //currently unused
	std::time_t timeStamp;
	bool doGaussian;
};



#endif