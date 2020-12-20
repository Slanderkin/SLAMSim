#pragma once

#include "StandardImports.h"
#include "World.h"
#include "Scan.h"


#ifndef ROBOT_H
#define ROBOT_H

class Robot {

private:
	void update();
	void checkCircleCol(const World &world, Vector2 velocity);
	void checkBorderCol(const World& world, Vector2 velocity);
	
public:
	Robot(Vector2 center,float headingIn,sf::Color color,Vector2 velocity,float maxRangeIn,float radiusIn, Scan &scan); 

	Vector2 center;
	float heading;
	sf::Color color;
	Vector2 velocity;//Velocity is linV,angV
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