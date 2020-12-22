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
	Scan::Observation* performScan(Vector2 origin,float &cRad,float &maxRange, const World &world);
	Scan::Observation* computeScanDerivatives(float minDist, Scan::Observation* obs);
	
	float raycast_circle(sf::CircleShape circle, float scanAngle, Vector2 origin);
	float raycast_wall(Vector2 origin, Vector2 end, Vector2 corA, Vector2 corB);
	float hz; //currently unused
	std::time_t timeStamp;
	bool doGaussian;
};



#endif