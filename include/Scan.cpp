#include "Scan.h"



Scan::Scan() :
	dist(mean, stddev),
	generator(std::random_device{}())
{
	this->hz = 0;
	this->timeStamp = NULL;
	this->doGaussian = false;
}

Scan::Observation* Scan::computeScanDerivatives(float minDist, Scan::Observation* obs) {
	Scan::Observation* der = new Scan::Observation;
	der->theta = std::array<float, 360>();
	der->distance = std::array<float, 360>();
	der->distance[0] = 0;der->distance[359] = 0;
	der->theta[0] = obs->theta[0];der->theta[359] = obs->theta[359];
	float l = 0;
	float r = 0;
	float currDer = 0;
	for (int i = 1; i < obs->theta.size()-1;i++) {
		der->theta[i] = obs->theta[i];
		l = obs->distance[i - 1.0];
		r = obs->distance[i + 1.0];
		if (l > minDist && r > minDist) {
			currDer = (r - l) / (2 * (float)M_PI / 180);
			der->distance[i] = currDer;
		}
		else {
			der->distance[i] = 0;
		}
	}

}

Scan::Observation* Scan::performScan(float& cx, float& cy, float& cRad, float& maxRange, const World& world) {
	Scan::Observation* obs = new Scan::Observation;
	obs->theta = std::array<float, 360> ();
	obs->distance = std::array<float, 360>();

	float scanAngle;
	for (int i = 0; i < 360; i++) {
		scanAngle = (i - 180) * M_PI / 180;
		sf::Vector2f colPoint(-1, -1);
		float newDist = -1;
		float minDist = 100000000000;

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
			
			if (doGaussian && newDist > 0)
			{
				newDist += 1000 * dist(generator);
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
				newDist = getBorderCollisionDist(cx, cy, cx + maxRange * cos(scanAngle), cy + maxRange * sin(scanAngle), world.edges[j][0], world.edges[j][1], world.edges[j][2], world.edges[j][3]);
				if (newDist != -1 && newDist < minDist)
				{
					minDist = newDist;
				}
			}

		}

		obs->theta[i] = scanAngle;
		obs->distance[i] = minDist;
	}

	return obs;
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
	
	float theta_max = asin(rc / d);
	float theta = abs(scanAngle - atan2(dy, dx));

	if (theta > theta_max && abs(theta - 2 * M_PI) > theta_max) return -1.;
	float l = d * tan(theta);
	float c = M_PI / 2 - theta;
	float a = M_PI - asin(l / rc * sin(c)) - c;

	return sqrt(l * l + d * d) - rc * sin(a) / sin(c);

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

float Scan::getBorderCollisionDist( float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {

	float uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
	float uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
	float dx = 0;
	float dy = 0;
	if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {
		if (this->doGaussian) {
			uA += dist(generator);
		}
		dx = uA * (x2 - x1);
		dy = uA * (y2 - y1);
		return sqrt(dx * dx + dy * dy);
	}
	return -1;
	
}
