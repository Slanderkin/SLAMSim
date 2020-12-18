#include "World.h"
#include "Scan.h"
#include "StandardImports.h"


Scan::Scan(int maxAngle) {
	this->hz = 0;
	this->maxAngle = maxAngle;


	for (int i = 0;i < this->maxAngle;i++) {
		this->angles[i] = i;
		sf::VertexArray line(sf::Lines, 2);
		this->scanLines[i] = line;
		sf::CircleShape circle(3);
		circle.setPosition(-10, -10);
		this->endCircles[i] = circle;
		for (int j = 0; j < 4; j++) {
			this->vertexPoints[i][j] = { 0 };
		}
	}
}

Scan::Scan() {
	this->hz = 0;
	this->maxAngle = 360;
	for (int i = 0;i < maxAngle;i++) {
		this->angles[i] = i;
		sf::VertexArray line(sf::Lines, 2);
		this->scanLines[i] = line;
		sf::CircleShape circle(3);
		circle.setPosition(-10, -10);
		circle.setFillColor(sf::Color::Blue);
		this->endCircles[i] = circle;
		for (int j = 0; j < 4; j++) {
			this->vertexPoints[i][j] = { 0 };
		}
	}
}


void Scan::performScan(float& cx, float& cy,float &cRad ,float& maxRange, const World& world) {

	for (int i = 0;i < 360;i++) {
		sf::Vector2f colPoint(-1, -1);
		float currMag = -1;
		float distAway = 100000000000;
		this->scanLines[i][0].position = sf::Vector2f(cx, cy);
		this->scanLines[i][1].position = sf::Vector2f(cx + maxRange * cos(i * M_PI / 180), cy - maxRange * sin(i * M_PI / 180));
		vertexPoints[i][0] = {cx};
		vertexPoints[i][1] = { cy };
		vertexPoints[i][2] = { cx + maxRange * (float)cos(i * M_PI / 180) };
		vertexPoints[i][3] = { cy - maxRange * (float)sin(i * M_PI / 180) };
		float* returnArrPtr;
		float blanks[3] = { -1,-1,-1 };
		returnArrPtr = blanks;
		for (int z = 0; z < world.circles.size(); z++) {
			float circlePos[2] = { world.circles[z].getPosition().x,world.circles[z].getPosition().y };
			float circleRad = world.circles[z].getRadius();

			if (!(circleRad + circlePos[0] < cx || circleRad + circlePos[1] > cy) && i<=90) {
				returnArrPtr = getCircleCollisionPoint(world.circles[z], colPoint, vertexPoints[i][0], vertexPoints[i][1], vertexPoints[i][2], vertexPoints[i][3], distAway);
			}
			else if (!(circleRad + circlePos[0] > cx || circleRad + circlePos[1] > cy) && (i<=180 && i >=90)) {
				returnArrPtr = getCircleCollisionPoint(world.circles[z], colPoint, vertexPoints[i][0], vertexPoints[i][1], vertexPoints[i][2], vertexPoints[i][3], distAway);
			}
			else if (!(circleRad + circlePos[0] > cx || circleRad + circlePos[1] < cy) && (i <= 270 && i >= 180)) {
				returnArrPtr = getCircleCollisionPoint(world.circles[z], colPoint, vertexPoints[i][0], vertexPoints[i][1], vertexPoints[i][2], vertexPoints[i][3], distAway);
			}
			else if (!(circleRad + circlePos[0] < cx || circleRad + circlePos[1] < cy) && (i <= 360 && i >= 270)) {
				returnArrPtr = getCircleCollisionPoint(world.circles[z], colPoint, vertexPoints[i][0], vertexPoints[i][1], vertexPoints[i][2], vertexPoints[i][3], distAway);
			}

			if ((*(returnArrPtr+2) < currMag && *(returnArrPtr + 2) != -1) || (currMag == -1 && *(returnArrPtr + 2) != -1)) {
				colPoint.x = *(returnArrPtr);
				colPoint.y = *(returnArrPtr+1);
			}
			
		}
		if (colPoint.x == -1) {

			for (int j = 0; j < 4; j++) {
				getBorderCollisionPoint(colPoint, scanLines[i][0].position.x, scanLines[i][0].position.y, scanLines[i][1].position.x, scanLines[i][1].position.y, world.edges[j][0], world.edges[j][1], world.edges[j][2], world.edges[j][3]);
			}

		}
		else {

		}
		this->scanLines[i][1].position = sf::Vector2f(colPoint);
		this->endCircles[i].setPosition(colPoint.x - endCircles[i].getRadius(), colPoint.y - endCircles[i].getRadius());
	}



}

float* Scan::getCircleCollisionPoint(sf::CircleShape circle, sf::Vector2f& colPoint, float& x1, float& y1, float& x2, float& y2, float& distAway) {

	float* retArr = new float[3];//x,y,Mag
	retArr[0] = { -1 };
	retArr[1] = { -1 };
	retArr[2] = { -1 };

	float R = circle.getRadius();
	float cx = circle.getPosition().x+R;
	float cy = circle.getPosition().y+R;
	float minMag;
	if (colPoint.x == -1) {
			minMag = 10000000000;
	}
	else {
		minMag = (colPoint.x-x1) * (colPoint.x - x1) + (colPoint.y-y1) * (colPoint.y - y1);
	}
		
	float Q[2] = { cx, cy };
	float V[2] = { x2 - x1,y2 - y1 };
	float a = V[0] * V[0] + V[1] * V[1];
	float b = 2 * V[0] * (x1 - Q[0]) + 2 * V[1] * (y1 - Q[1]);
	float c = x1 * x1 + y1 * y1 + Q[0] * Q[0] + Q[1] * Q[1] - 2 * (x1 * Q[0] + y1 * Q[1]) - R * R;
	float disc = b * b - 4 * a * c;

	if (disc < 0) {
		return retArr;
	}
	else {
		float sqrt_disc = sqrt(disc);
		float t1 = (-b + sqrt_disc) / (2 * a);
		float t2 = (-b - sqrt_disc) / (2 * a);

		if (!((0 <= t1 && t1 <= 1) || (0 <= t2 && t2 <= 1))) {

			return retArr;
		}
		else {
			float t1Mag = pow(t1 * V[0], 2) + pow(t1 * V[1], 2);
			float t2Mag = pow(t2 * V[0], 2) + pow(t2 * V[1], 2);
			if (t1Mag < t2Mag && t1Mag < minMag) {
				retArr[0] = t1 * V[0] + x1;
				retArr[1] = t1 * V[1] + y1;
				retArr[2] = (retArr[0]-cx)* (retArr[0] - cx) + (retArr[1]-cy)* (retArr[1] - cy);
				return retArr;
			}
			else if (t2Mag < t1Mag && t2Mag < minMag) {
				retArr[0] = t2 * V[0] + x1;
				retArr[1] = t2 * V[1] + y1;
				retArr[2] = (retArr[0] - cx) * (retArr[0] - cx) + (retArr[1] - cy) * (retArr[1] - cy);
				return retArr;
			}
			else {
				return retArr;
			}
		}
	}
}

void Scan::getBorderCollisionPoint(sf::Vector2f& colPoint, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {

	float uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
	float uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

	if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {

		colPoint.x = x1 + (uA * (x2 - x1));
		colPoint.y = y1 + (uA * (y2 - y1));
	}
}
