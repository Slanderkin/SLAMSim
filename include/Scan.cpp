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
	return der;

}

Scan::Observation* Scan::performScan(float& cx, float& cy, float& cRad, float& maxRange, const World& world) {
	Scan::Observation* obs = new Scan::Observation;
	obs->theta = std::array<float, 360> ();
	obs->distance = std::array<float, 360>();

	float scanAngle;
	for (int i = 0; i < 360; i++) {
		scanAngle = (i - 180.) * M_PI / 180;
		float newDist;
		float minDist = NULL;

		for (int z = 0; z < world.circles.size(); z++) {
			newDist = -1;

			float circ_r = world.circles[z].getRadius();
			float circ_x = world.circles[z].getPosition().x + circ_r;
			float circ_y = world.circles[z].getPosition().y + circ_r;
			
			//Quadrant filter for performance
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

			// See if this is first distance or shortest distance
			if (newDist > 0 && (minDist == NULL || newDist < minDist))
			{
				minDist = newDist;
			}

		}
		// If no distance was found see what wall it hit
		if (minDist == NULL) {

			for (int j = 0; j < 4; j++) {
				newDist = raycast_wall(cx, cy, cx + maxRange * cos(scanAngle), cy + maxRange * sin(scanAngle), world.edges[j][0], world.edges[j][1], world.edges[j][2], world.edges[j][3]);
				if (newDist != -1 && (minDist == NULL || newDist < minDist))
				{
					minDist = newDist;
				}
			}

		}

		// Apply noise
		if (doGaussian)
		{
			minDist += 1000 * dist(generator);
		}

		// Add to obs
		obs->theta[i] = scanAngle;
		obs->distance[i] = minDist;
	}

	return obs;
}

/*
	RAYCAST HANDLING
*/
float Scan::raycast_circle(sf::CircleShape circle, float scanAngle, float x0, float y0) {

	// First calculate the angle between the two tangent lines that intersect with the observer
	float rc = circle.getRadius();

	float dx = circle.getPosition().x + rc - x0;
	float dy = circle.getPosition().y + rc - y0;

	float d = sqrt(dx * dx + dy * dy);
	
	float theta_max = asin(rc / d);
	float theta = abs(scanAngle - atan2(dy, dx));

	// If relative scan angle is outside of the range, return
	if (theta > theta_max && abs(theta - 2 * M_PI) > theta_max) return -1.;

	// If ray passes through circle, see how much of it is inside the circle, return distance-enclosed
	float l = d * tan(theta);
	float c = M_PI / 2 - theta;
	float a = M_PI - asin(l / rc * sin(c)) - c;

	return sqrt(l * l + d * d) - rc * sin(a) / sin(c);

}

float Scan::raycast_wall( float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {

	float uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
	float uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
	float dx = 0;
	float dy = 0;
	if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {
		dx = uA * (x2 - x1);
		dy = uA * (y2 - y1);
		return sqrt(dx * dx + dy * dy);
	}
	return -1;
	
}
