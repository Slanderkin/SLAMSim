#include "Robot.h"
#include "StandardImports.h"

Robot::Robot(Vector2 center, float heading, sf::Color color, Vector2 velocity, float maxRange, float radius, Scan &scan)
	: scan()
{
	this->center = center;//X,Y
	this->heading = heading; //In deg
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

/*
TODO:
Make the turning more accurate to the two wheel model


Modles a two wheel robot seperated  by a width of 2*radius
The vector control consists of the distance travelled by each wheel (l,r)
*/
void Robot::move(const World& world, Vector2 control) {
	float l = control.x;
	float r = control.y;
	float alpha = 0;
	float R = 0;
	float x1 = center.x;
	float y1 = center.y;

	if (r == -l) {
		if (r < l) {
			turn(true);

		}
		else {
			turn(false);

		}
	}
	else if (r < l) {
		alpha = (l - r) / (radius*2);
		R = r / alpha;
		x1 = center.x + (R + radius) * (sin(heading*M_PI/180)-sin(heading * M_PI / 180-alpha));
		y1 = center.y + (R + radius) * (-cos(heading * M_PI / 180) + cos(heading * M_PI / 180 - alpha));
		heading = (fmod((heading * M_PI / 180 - alpha + M_PI), (2 * M_PI)) - M_PI)*180/M_PI;
	}
	else if (l < r) {
		alpha = (r - l) / (radius * 2);
		R = l / alpha;
		x1 = center.x + (R + radius) * (-sin(heading * M_PI / 180) + sin(heading * M_PI / 180 + alpha));
		y1 = center.y + (R + radius) * (cos(heading * M_PI / 180) - cos(heading * M_PI / 180 + alpha));
		heading = (fmod((heading * M_PI / 180 + alpha + M_PI), (2 * M_PI)) - M_PI)*180/M_PI;

	}
	else {
		x1 = center.x + l * cos(heading * M_PI / 180);
		y1 = center.y + l * sin(heading * M_PI / 180);
	}

	checkBorderCol(world, Vector2(x1,y1));
	checkCircleCol(world, Vector2(x1, y1));
	update();

}

/*
void Robot::forward(const World &world) {
	float vX = velocity.x * cos(heading * M_PI / 180);
	float vY = velocity.x * sin(heading * M_PI / 180);

	checkBorderCol(world, Vector2(vX, vY));
	checkCircleCol(world, Vector2(vX, vY));
	update();
	
}
*/

/*
TODO:
Update to accurately reflect the two wheel model given a control input


*/
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

void Robot::checkBorderCol(const World &world, Vector2 newPos) {
	if (newPos.x - radius <= world.border.x) {
		center.x = world.border.x+radius;

	}
	else if (newPos.x >= world.size.x - world.border.x) {
		center.x = world.size.x - world.border.x-radius;

	}
	else {
		center.x = newPos.x;
	}
	if (newPos.y -radius <= world.border.y) {
		center.y = world.border.y+radius;

	}
	else if (newPos.y + radius >= world.size.y -world.border.y) {
		center.y = world.size.y - world.border.y-radius;


	}
	else {
		center.y = newPos.y;
	}

}

void Robot::checkCircleCol(const World &world, Vector2 newPos) {
	for (int i = 0; i < world.circles.size();i++) {

		float circleRad = world.circles[i].getRadius();
		sf::Vector2f position = world.circles[i].getPosition();
		float* collisionPoint = circlesCollided(newPos.x, newPos.y, position.x + circleRad, position.y + circleRad, radius, circleRad);
		center.x += collisionPoint[0];
		center.y += collisionPoint[1];
		collisionPoint[0] = 0.f;
		collisionPoint[1] = 0.f;
	}
	
}

float* Robot::circlesCollided(float c1x, float c1y, float c2x, float c2y, float c1rad, float c2rad) {
	static float result[2];
	float distanceBetweenCirclesSquared = (c2x - c1x) * (c2x - c1x) + (c2y - c1y) * (c2y - c1y);
	if (distanceBetweenCirclesSquared < (c1rad + c2rad) * (c1rad + c2rad)) {
		float angle = atan2f(c2y - c1y, c2x - c1x);
		float distBetweenCircles = sqrt(distanceBetweenCirclesSquared);
		float distToMove = c1rad + c2rad - distBetweenCircles;
		result[0] = -cos(angle) * distToMove;
		result[1] = -sin(angle) * distToMove;
	}
	return result;
}
