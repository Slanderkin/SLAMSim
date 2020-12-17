#pragma once
#ifndef SCAN_H
#define SCAN_H
#include "World.h"
#include "StandardImports.h"


class Scan {



public:
	Scan(int);
	Scan();
	void performScan(float &cx, float &cy,float &maxRange, const World &world);
	void getCircleCollisionPoint(sf::CircleShape circle, sf::Vector2f &colPoint, sf::VertexArray &currLine, float &distAway);
	void getBorderCollisionPoint(sf::Vector2f &colPoint, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4);
	sf::VertexArray scanLines [360];
	sf::CircleShape endCircles[360];
	int angles [360];
	int maxAngle;
	float hz; //currently unused
};



#endif