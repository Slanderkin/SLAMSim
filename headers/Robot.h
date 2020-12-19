#pragma once

#include "StandardImports.h"
#include "World.h"
#include "Scan.h"
#include "StandardImports.h"

#ifndef ROBOT_H
#define ROBOT_H

class Robot {

private:
	void update();
	void checkCircleCol(const World &world, float vs[2]);
	void checkBorderCol(const World& world, float vs[2]);
	
public:
	Robot(float centerIn[],float headingIn,sf::Color color,float velocityIn[],float maxRangeIn,float radiusIn, Scan &scan);

	float center [2];
	float heading;
	sf::Color color;
	float velocity [2];
	float maxRange;
	sf::CircleShape circle;
	sf::RectangleShape dirLine;
	float radius;
	Scan scan;
	void checkBorderCol(const World& world, float linVelocity,float heading);
	void forward(const World &world);
	void turn(bool isLeft);
	void checkScanCol(const World& world);
};
#endif