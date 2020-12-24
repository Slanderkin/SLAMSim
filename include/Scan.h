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
	Scan::Observation* computeScanDerivatives( Scan::Observation* obs);
	float raycast_circle(sf::CircleShape circle, float scanAngle, Vector2 origin);
	float raycast_wall(Vector2 origin, Vector2 end, Vector2 corA, Vector2 corB);
	std::vector<Eigen::Vector2f> findCylinders(Scan::Observation* derivative, Scan::Observation* foundScan, float jump);
	std::vector<Eigen::Matrix2f> getCylinders(float jump, std::vector<Eigen::Vector2f> cylinders,Vector2 origin);

	float hz; //currently unused
	std::time_t timeStamp;
	bool doGaussian;
	std::vector<Eigen::Matrix2f> cylinders;
};



#endif