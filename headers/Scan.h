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
	Scan();
	void performScan(float &cx, float &cy,float &cRad,float &maxRange, const World &world);
	float* getCircleCollisionPoint(sf::CircleShape circle, sf::Vector2f &colPoint, float &x1, float &y1, float &x2, float &y2, float &distAway);
	float raycast_circle(sf::CircleShape circle, float scanAngle, float x0, float y0);
	void getBorderCollisionPoint(sf::Vector2f &colPoint, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4);
	sf::VertexArray scanLines [360];
	sf::CircleShape endCircles[360];
	float vertexPoints[360][4];
	int angles[360];
	float hz; //currently unused
	std::time_t timeStamp;
	float ranges[360];
	bool doGaussian;
};



#endif