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
		float distAway = 100000000;
		this->scanLines[i][0].position = sf::Vector2f(cx, cy);
		this->scanLines[i][1].position = sf::Vector2f(cx + maxRange * cos(i * M_PI / 180), cy - maxRange * sin(i * M_PI / 180));
		vertexPoints[i][0] = {cx};
		vertexPoints[i][1] = { cy };
		vertexPoints[i][2] = { cx + maxRange * (float)cos(i * M_PI / 180) }; //Check janky math
		vertexPoints[i][3] = { cy - maxRange * (float)sin(i * M_PI / 180) };//Check janky math
		for (int z = 0; z < world.circles.size(); z++) {
			float circlePos[2] = { world.circles[z].getPosition().x,world.circles[z].getPosition().y };
			float circleRad = world.circles[z].getRadius();
			/*
			TR Check:
				If Not(Circle Center + Rad < Self X or Circle Center + Rad > Self Y) And Angle<=90:
					Check
			TL Check:
				If Not(Circle Center + Rad > Self X or Circle Center + Rad > Self Y) And (Angle <=180 And Angle >90):
					Check
			BL Check:
				If Not(Circle Center + Rad > Self X or Circle Center + Rad < Self Y) And (Angle <=270 And Angle >180):
					Check
			BR Check:
				If Not(Circle Center + Rad < Self X or Circle Center + Rad < Self Y) And (Angle <=360 And Angle >270):
					Check

			
			*/
			if (!(circleRad + circlePos[0] < cx || circleRad + circlePos[1] > cy) && i<=90) {
				getCircleCollisionPoint(world.circles[z], colPoint, vertexPoints[i][0], vertexPoints[i][1], vertexPoints[i][2], vertexPoints[i][3], distAway);
			}
			else if (!(circleRad + circlePos[0] > cx || circleRad + circlePos[1] > cy) && (i<=180 && i >=90)) {
				getCircleCollisionPoint(world.circles[z], colPoint, vertexPoints[i][0], vertexPoints[i][1], vertexPoints[i][2], vertexPoints[i][3], distAway);
			}
			else if (!(circleRad + circlePos[0] > cx || circleRad + circlePos[1] < cy) && (i <= 270 && i >= 180)) {
				getCircleCollisionPoint(world.circles[z], colPoint, vertexPoints[i][0], vertexPoints[i][1], vertexPoints[i][2], vertexPoints[i][3], distAway);
			}
			else if (!(circleRad + circlePos[0] < cx || circleRad + circlePos[1] < cy) && (i <= 360 && i >= 270)) {
				getCircleCollisionPoint(world.circles[z], colPoint, vertexPoints[i][0], vertexPoints[i][1], vertexPoints[i][2], vertexPoints[i][3], distAway);
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

void Scan::getCircleCollisionPoint(sf::CircleShape circle, sf::Vector2f& colPoint, float &x1,float &y1,float &x2,float &y2, float& distAway) {


	float currDist = pow(x2 - x1, 2) + pow(y2-y1, 2);
	if (currDist < distAway){
		float R = circle.getRadius();
		float minMag = 100000000;
		float Q[2] = { circle.getPosition().x + R, circle.getPosition().y + R };
		float V[2] = { x2 - x1,y2 - y1};
		float a = V[0]*V[0] + V[1]*V[1];
		float b = 2 * V[0] * (x1 - Q[0]) + 2 * V[1] * (y1 - Q[1]);
		float c = x1*x1 + y1*y1 + Q[0]*Q[0] + Q[1]*Q[1] - 2 * (x1 * Q[0] + y1 * Q[1]) - R*R;
		float disc = b*b - 4 * a * c;

		if (disc < 0) {
			colPoint.x = -1;
			colPoint.y = -1;
		}
		else {
			float sqrt_disc = sqrt(disc);
			float t1 = (-b + sqrt_disc) / (2 * a);
			float t2 = (-b - sqrt_disc) / (2 * a);

			if (!((0 <= t1 && t1 <= 1) || (0 <= t2 && t2 <= 1))) {

				colPoint.x = -1;
				colPoint.y = -1;
			}
			else {
				float t1Mag = pow(t1 * V[0], 2) + pow(t1 * V[1], 2);
				float t2Mag = pow(t2 * V[0], 2) + pow(t2 * V[1], 2);
				if (t1Mag < t2Mag && t1Mag < minMag) {
					colPoint.x = t1 * V[0];
					colPoint.y = t1 * V[1];
					distAway = currDist;
				}
				else if (t2Mag < t1Mag && t2Mag < minMag) {
					colPoint.x = t2 * V[0] + x1;
					colPoint.y = t2 * V[1] + y1;
					distAway = currDist;
				}

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
