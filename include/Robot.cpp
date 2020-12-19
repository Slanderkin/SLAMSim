#include "Robot.h"
#include "Collision.h"

Robot::Robot(float center[], float heading, sf::Color color, float velocity[], float maxRange, float radius, Scan &scan)
	: scan()
{
	this->center[0] = center[0]; this->center[1] = center[1];//X,Y
	this->heading = heading;
	this->color = color;
	this->velocity[0] = velocity[0];this->velocity[1] = velocity[1]; //Linear,Angular
	this->maxRange = maxRange;
	this->radius = radius;
	this->scan = scan;

	circle = sf::CircleShape(10.f);
	dirLine = sf::RectangleShape(sf::Vector2f(20.f, 2.f));
	circle.setFillColor(color);
	circle.setPosition(Robot::center[0] - radius, Robot::center[1] - radius);
	dirLine.setPosition(Robot::center[0], Robot::center[1]);
	dirLine.setRotation(heading);
	dirLine.setFillColor(color);
}

void Robot::forward(const World &world) {
	float vX = velocity[0] * cos(heading * M_PI / 180);
	float vY = velocity[0] * sin(heading * M_PI / 180);
	float vs[2] = { vX,vY };
	checkBorderCol(world, vs);
	checkCircleCol(world, vs);
	update();
	
}

void Robot::turn(bool isLeft) {
	if (isLeft){
		heading -= velocity[1];
	}
	else {
		heading += velocity[1];
	}
	update();
}

void Robot::update() {
	circle.setPosition(center[0] - radius, center[1] - radius);
	dirLine.setPosition(center[0], center[1]);
	dirLine.setRotation(heading);
}

void Robot::checkBorderCol(const World &world, float vs[2]) {
	if (center[0] + vs[0] - radius <= world.border[0]) {
		center[0] = world.border[0]+radius;

	}
	else if (center[0] + vs[0]+radius >= world.size[0] - world.border[0]) {
		center[0] = world.size[0] - world.border[0]-radius;

	}
	else {
		center[0] += vs[0];
	}
	if (center[1] + vs[1] -radius <= world.border[1]) {
		center[1] = world.border[1]+radius;

	}
	else if (center[1] + vs[1] + radius >= world.size[1] -world.border[1]) {
		center[1] = world.size[1] - world.border[1]-radius;


	}
	else {
		center[1] += vs[1];
	}

}

void Robot::checkBorderCol(const World& world, float linVelocity, float heading) {
	float vs[2] = { linVelocity * cos(heading * M_PI / 180) ,linVelocity * sin(heading * M_PI / 180) };
	
	if (center[0] + vs[0] - radius <= world.border[0]) {
		center[0] = world.border[0] + radius;

	}
	else if (center[0] + vs[0] + radius >= world.size[0] - world.border[0]) {
		center[0] = world.size[0] - world.border[0] - radius;

	}
	if (center[1] + vs[1] - radius <= world.border[1]) {
		center[1] = world.border[1] + radius;

	}
	else if (center[1] + vs[1] + radius >= world.size[1] - world.border[1]) {
		center[1] = world.size[1] - world.border[1] - radius;


	}

}

void Robot::checkCircleCol(const World &world,float vs[2]) {

	for (sf::CircleShape circle : world.circles) {
		float circleRad = circle.getRadius();
		sf::Vector2f position = circle.getPosition();
		float* collisionPoint = circlesCollided(center[0] + vs[0], center[1] + vs[1], position.x + circleRad, position.y + circleRad, radius, circleRad);
		center[0] += collisionPoint[0];
		center[1] += collisionPoint[1];
		collisionPoint[0] = 0.f;
		collisionPoint[1] = 0.f;
	}
	
}

void Robot::checkScanCol(const World &world) {

}