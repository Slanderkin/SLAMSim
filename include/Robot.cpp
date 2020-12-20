#include "Robot.h"
#include "Misc.h"
#include "StandardImports.h"

Robot::Robot(Vector2 center, float heading, sf::Color color, Vector2 velocity, float maxRange, float radius, Scan &scan)
	: scan()
{
	this->center = center;//X,Y
	this->heading = heading;
	this->color = color;
	this->velocity = velocity; //Linear,Angular
	this->maxRange = maxRange;
	this->radius = radius;
	this->scan = scan;

	circle = sf::CircleShape(10.f);
	dirLine = sf::RectangleShape(sf::Vector2f(20.f, 2.f));
	circle.setFillColor(color);
	circle.setPosition(this->center.x - radius, this->center.y - radius);
	dirLine.setPosition(this->center.x, this->center.y);
	dirLine.setRotation(heading);
	dirLine.setFillColor(color);
}

void Robot::forward(const World &world) {
	float vX = velocity.x * cos(heading * M_PI / 180);
	float vY = velocity.x * sin(heading * M_PI / 180);

	checkBorderCol(world, Vector2(vX, vY));
	checkCircleCol(world, Vector2(vX, vY));
	update();
	
}

void Robot::turn(bool isLeft) {
	if (isLeft){
		heading -= velocity.y;
	}
	else {
		heading += velocity.y;
	}
	update();
}

void Robot::update() {
	circle.setPosition(center.x - radius, center.y - radius);
	dirLine.setPosition(center.x, center.y);
	dirLine.setRotation(heading);
}

void Robot::checkBorderCol(const World &world, Vector2 velocity) {
	if (center.x + velocity.x - radius <= world.border.x) {
		center.x = world.border.x+radius;

	}
	else if (center.x + velocity.x+radius >= world.size.x - world.border.x) {
		center.x = world.size.x - world.border.x-radius;

	}
	else {
		center.x += velocity.x;
	}
	if (center.y + velocity.y -radius <= world.border.y) {
		center.y = world.border.y+radius;

	}
	else if (center.y + velocity.y + radius >= world.size.y -world.border.y) {
		center.y = world.size.y - world.border.y-radius;


	}
	else {
		center.y += velocity.y;
	}

}

void Robot::checkBorderCol(const World& world, float linVelocity, float heading) {
	Vector2 vs = { linVelocity * (float)cos(heading * M_PI / 180) ,linVelocity * (float)sin(heading * M_PI / 180) };
	
	if (center.x + vs.x - radius <= world.border.x) {
		center.x = world.border.x + radius;

	}
	else if (center.x + vs.x + radius >= world.size.x - world.border.x) {
		center.x = world.size.x - world.border.x - radius;

	}
	if (center.y + vs.y - radius <= world.border.y) {
		center.y = world.border.y + radius;

	}
	else if (center.y + vs.y + radius >= world.size.y - world.border.y) {
		center.y = world.size.y - world.border.y - radius;


	}

}

void Robot::checkCircleCol(const World &world, Vector2 velocity) {

	for (sf::CircleShape circle : world.circles) {
		float circleRad = circle.getRadius();
		sf::Vector2f position = circle.getPosition();
		float* collisionPoint = circlesCollided(center.x + velocity.x, center.y + velocity.y, position.x + circleRad, position.y + circleRad, radius, circleRad);
		center.x += collisionPoint[0];
		center.y += collisionPoint[1];
		collisionPoint[0] = 0.f;
		collisionPoint[1] = 0.f;
	}
	
}

void Robot::checkScanCol(const World &world) {

}