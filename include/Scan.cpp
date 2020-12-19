#include "World.h"
#include "Scan.h"
#include "StandardImports.h"



Scan::Scan() :
	dist(mean, stddev),
	generator(std::random_device{}())
{
	this->hz = 0;
	this->timeStamp = NULL;
	this->doGaussian = false;


	
	for (int i = 0;i < 360;i++){
		this->angles[i] = i;
		sf::VertexArray line(sf::Lines, 2);
		this->scanLines[i] = line;
		sf::CircleShape circle(3);
		circle.setPosition(-10, -10);
		circle.setFillColor(sf::Color::Blue);
		this->endCircles[i] = { circle };
		for (int j = 0; j < 4; j++) {
			this->vertexPoints[i][j] = { 0 };
		}
	}
}

/*
TODO According to Ryland:
also in my humble opinion I think scan should return a struct with a list of angles and a list of distances
and then convert that to cartesian points and offset to bot position in renderer

*/
/*
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
		
		std::unique_ptr<float[]> returnArrPtr (new float[3]);

		returnArrPtr[0] =-1;returnArrPtr[1] = -1;returnArrPtr[2] = -1;

		for (int z = 0; z < world.circles.size(); z++) {
			float circlePos[2] = { world.circles[z].getPosition().x,world.circles[z].getPosition().y };
			float circleRad = world.circles[z].getRadius();

			if (!(circleRad + circlePos[0] < cx +cRad|| circleRad + circlePos[1] > cy +cRad) && i<=90) {
				returnArrPtr.reset(getCircleCollisionPoint(world.circles[z], colPoint, vertexPoints[i][0], vertexPoints[i][1], vertexPoints[i][2], vertexPoints[i][3], distAway));
			}
			else if (!(circleRad + circlePos[0] > cx + cRad || circleRad + circlePos[1] > cy + cRad) && (i<=180 && i >=90)) {
				returnArrPtr.reset(getCircleCollisionPoint(world.circles[z], colPoint, vertexPoints[i][0], vertexPoints[i][1], vertexPoints[i][2], vertexPoints[i][3], distAway));
			}
			else if (!(circleRad + circlePos[0] > cx + cRad || circleRad + circlePos[1] < cy + cRad) && (i <= 270 && i >= 180)) {
				returnArrPtr.reset(getCircleCollisionPoint(world.circles[z], colPoint, vertexPoints[i][0], vertexPoints[i][1], vertexPoints[i][2], vertexPoints[i][3], distAway));
			}
			else if (!(circleRad + circlePos[0] < cx + cRad || circleRad + circlePos[1] < cy + cRad) && (i <= 360 && i >= 270)) {
				returnArrPtr.reset(getCircleCollisionPoint(world.circles[z], colPoint, vertexPoints[i][0], vertexPoints[i][1], vertexPoints[i][2], vertexPoints[i][3], distAway));
			}

			if ((returnArrPtr[2] < currMag && returnArrPtr[2] != -1) || (currMag == -1 && returnArrPtr[2] != -1)) {
				colPoint.x = returnArrPtr[0];
				colPoint.y = returnArrPtr[1];
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

	this->timeStamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

}
*/
void Scan::performScan(float& cx, float& cy, float& cRad, float& maxRange, const World& world) {
	float scanAngle;
	for (int i = 0; i < 360; i++) {
		scanAngle = (i - 180) * M_PI / 180;
		sf::Vector2f colPoint(-1, -1);
		float newDist = -1;
		float minDist = 100000000000;
		this->scanLines[i][0].position = sf::Vector2f(cx, cy);
		this->scanLines[i][1].position = sf::Vector2f(cx + maxRange * cos(scanAngle), cy + maxRange * sin(scanAngle));
		vertexPoints[i][0] = { cx };
		vertexPoints[i][1] = { cy };
		vertexPoints[i][2] = { cx + maxRange * (float)cos(scanAngle) };
		vertexPoints[i][3] = { cy + maxRange * (float)sin(scanAngle) };
		std::unique_ptr<float[]> returnArrPtr(new float[3]);

		returnArrPtr[0] = -1; returnArrPtr[1] = -1; returnArrPtr[2] = -1;

		for (int z = 0; z < world.circles.size(); z++) {
			newDist = -1;

			float circ_r = world.circles[z].getRadius();
			float circ_x = world.circles[z].getPosition().x + circ_r;
			float circ_y = world.circles[z].getPosition().y + circ_r;
			
			if (circ_y - circ_r < cy && scanAngle <= 0)
			{
				if (circ_x - circ_r < cx && scanAngle <= -M_PI / 2)
				{
					newDist = raycast_circle(world.circles[z], scanAngle, cx, cy);
				}
				else if (circ_x + circ_r > cx && scanAngle >= -M_PI / 2)
				{
					newDist = raycast_circle(world.circles[z], scanAngle, cx, cy);
				}
			}
			else if (circ_y + circ_r > cy && scanAngle >= 0)
			{
				if (circ_x - circ_r < cx && scanAngle >= M_PI / 2)
				{
					newDist = raycast_circle(world.circles[z], scanAngle, cx, cy);
				}
				else if (circ_x + circ_r > cx && scanAngle <= M_PI / 2)
				{
					newDist = raycast_circle(world.circles[z], scanAngle, cx, cy);
				}
			}
			
			if (newDist > 0 && newDist < minDist)
			{
				minDist = newDist;
				colPoint.x = minDist * cos(scanAngle) + cx;
				colPoint.y = minDist * sin(scanAngle) + cy;
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

	std::unique_ptr<float[]> retArr(new float[3]);//x,y,Mag
	retArr[0] = -1;
	retArr[1] = -1;
	retArr[2] = -1;

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
		return retArr.release();
	}
	else {
		float sqrt_disc = sqrt(disc);
		float t1 = (-b + sqrt_disc) / (2 * a);
		float t2 = (-b - sqrt_disc) / (2 * a);

		if (!((0 <= t1 && t1 <= 1) || (0 <= t2 && t2 <= 1))) {

			return retArr.release();
		}
		else {
			float t1Mag = pow(t1 * V[0], 2) + pow(t1 * V[1], 2);
			float t2Mag = pow(t2 * V[0], 2) + pow(t2 * V[1], 2);
			if (t1Mag < t2Mag && t1Mag < minMag) {

				if (this->doGaussian) {
					t1 += dist(generator);
				}
				retArr[0] = t1 * V[0] + x1;
				retArr[1] = t1 * V[1] + y1;
				retArr[2] = (retArr[0] -cx)* (retArr[0] - cx) + (retArr[1] -cy)* (retArr[1] - cy);

				return retArr.release();
				
			}
			else if (t2Mag < t1Mag && t2Mag < minMag) {
				if (this->doGaussian) {
					t2 += dist(generator);
				}
				retArr[0] = t2 * V[0] + x1;
				retArr[1] = t2 * V[1] + y1;
				retArr[2] = (retArr[0] - cx) * (retArr[0] - cx) + (retArr[1] - cy) * (retArr[1] - cy);
				return retArr.release();
			}
			else {
				return retArr.release();
			}
		}
	}
}

float Scan::raycast_circle(sf::CircleShape circle, float scanAngle, float x0, float y0) {

	float rc = circle.getRadius();

	float dx = circle.getPosition().x + rc - x0;
	float dy = circle.getPosition().y + rc - y0;

	float d = sqrt(dx * dx + dy * dy);
	float noise = 0;

	float theta = abs(scanAngle - atan2(dy, dx));

	if (theta > abs(atan2(rc, d))) return -1.;

	float l = d * tan(theta);
	float a = M_PI / 2 - theta;
	if (this->doGaussian) {
		noise = 1250*dist(generator);
	}
	return sqrt(l * l + d * d) - rc * sin(a - asin(l * sin(a) / rc)) / sin(a) + noise;

}

void Scan::getBorderCollisionPoint(sf::Vector2f& colPoint, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {

	float uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
	float uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

	if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {
		if (this->doGaussian) {
			uA += dist(generator);
		}
		colPoint.x = x1 + (uA * (x2 - x1));
		colPoint.y = y1 + (uA * (y2 - y1));
	}
}
