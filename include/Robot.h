#pragma once

#include "StandardImports.h"
#include "World.h"
#include "Scan.h"


#ifndef ROBOT_H
#define ROBOT_H

class Robot {

private:
	
	void checkCircleCol(const World &world, Vector2 velocity);
	void checkBorderCol(const World& world, Vector2 velocity);
	float* circlesCollided(float c1x, float c1y, float c2x, float c2y, float c1rad, float c2rad);
	
public:
	Robot(Vector2 center,float headingIn,sf::Color color,Vector2 velocity,float maxRangeIn,float radiusIn, World *world); 

	void update();

	void attachObs(Scan::Observation *o);
	void addDrawView(DrawView *dv);
	void draw();


	Scan scan;
	Vector2 center;
	float heading;
	sf::Color color;
	Vector2 velocity;//Velocity is linV,angV
	float maxRange;
	float radius;
	World *world;
	std::vector<DrawView*> drawViews;
	bool drawRays;
	sf::CircleShape circle;
	sf::RectangleShape dirLine;
	
	Scan::Observation *obs;
	
	sf::CircleShape range;

	void move(Vector2 control);
	void forward(const World &world);
	void turn(bool isLeft);
	void checkScanCol(const World& world);
};
#endif