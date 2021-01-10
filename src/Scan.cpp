#include "Scan.h"

Scan::Scan() :
	dist(mean, stddev),generator(std::random_device{}()), hz(0),timeStamp(-1),doGaussian(false)
{

}

Scan::Observation* Scan::computeScanDerivatives(Scan::Observation* obs) {
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

		currDer = (r - l) / (obs->theta[i+1]-obs->theta[i-1]);
		der->distance[i] = currDer;

	}
	return der;

}

Scan::Observation* Scan::performScan(Vector2 origin, float& cRad, float& maxRange, const World& world) {
	Timer timer("Scan");
	Scan::Observation* obs = new Scan::Observation;
	obs->theta = std::array<float, 360> ();
	obs->distance = std::array<float, 360>();

	float scanAngle;
	for (int i = 0; i < 360; i++) {
		scanAngle = (i - 180.) * M_PI / 180;
		float newDist;
		float minDist = NAN;

		for (int z = 0; z < world.circles.size(); z++) {
			newDist = -1;

			float circ_r = world.circles[z].getRadius();
			float circ_x = world.circles[z].getPosition().x + circ_r;
			float circ_y = world.circles[z].getPosition().y + circ_r;
			
			//Quadrant filter for performance
			if (circ_y - circ_r < origin.y && scanAngle <= 0)
			{
				if (circ_x - circ_r < origin.x && scanAngle <= -M_PI / 2)
				{
					newDist = raycast_circle(world.circles[z], scanAngle, origin);
				}
				else if (circ_x + circ_r > origin.x && scanAngle >= -M_PI / 2)
				{
					newDist = raycast_circle(world.circles[z], scanAngle, origin);
				}
			}
			else if (circ_y + circ_r > origin.y && scanAngle >= 0)
			{
				if (circ_x - circ_r < origin.x && scanAngle >= M_PI / 2)
				{
					newDist = raycast_circle(world.circles[z], scanAngle, origin);
				}
				else if (circ_x + circ_r > origin.x && scanAngle <= M_PI / 2)
				{
					newDist = raycast_circle(world.circles[z], scanAngle, origin);
				}
			}

			// See if this is first distance or shortest distance
			if (newDist > 0 && (std::isnan(minDist) || newDist < minDist))
			{
				minDist = newDist;
			}

		}
		// If no distance was found see what wall it hit
		if (std::isnan(minDist)) {
			minDist = getBorderDist(world.worldVerticies,maxRange,scanAngle,origin);
			/*for (int j = 0; j < 4; j++) {
				newDist = raycast_wall(origin, origin + Vector2(maxRange * cos(scanAngle), maxRange * sin(scanAngle)), Vector2(world.edges[j][0], world.edges[j][1]), Vector2(world.edges[j][2], world.edges[j][3]));
				if (newDist != -1 && (std::isnan(minDist) || newDist < minDist))
				{
					minDist = newDist;
				}
			}*/

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
	Scan::Observation* ders = computeScanDerivatives(obs);

	std::vector<Eigen::Vector2f> cylinderVec = findCylinders(ders, obs,750);
	this->cylinders = getCylinders(100 * 180 / M_PI,cylinderVec,origin);
	return obs;
}

/*
	RAYCAST HANDLING
*/
float Scan::raycast_circle(sf::CircleShape circle, float scanAngle, Vector2 origin) {

	// First calculate the angle between the two tangent lines that intersect with the observer
	float rc = circle.getRadius();

	float dx = circle.getPosition().x + rc - origin.x;
	float dy = circle.getPosition().y + rc - origin.y;

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

float Scan::raycast_wall( Vector2 origin, Vector2 end, Vector2 corA, Vector2 corB) {

	float uA = ((corB.x - corA.x) * (origin.y - corA.y) - (corB.y - corA.y) * (origin.x - corA.x)) / ((corB.y - corA.y) * (end.x - origin.x) - (corB.x - corA.x) * (end.y - origin.y));
	float uB = ((end.x - origin.x) * (origin.y - corA.y) - (end.y - origin.y) * (origin.x - corA.x)) / ((corB.y - corA.y) * (end.x - origin.x) - (corB.x - corA.x) * (end.y - origin.y));
	float dx = 0;
	float dy = 0;
	if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {
		dx = uA * (end.x - origin.x);
		dy = uA * (end.y - origin.y);
		return sqrt(dx * dx + dy * dy);
	}
	return -1;
	
}

float Scan::getBorderDist(sf::VertexArray worldVerticies,float maxRange,float scanAngle,Vector2 origin){
	float minDist = NAN;
	float newDist;
	Vector2 end =  origin +  Vector2(maxRange * cos(scanAngle), maxRange * sin(scanAngle));
	for(int i =1; i < worldVerticies.getVertexCount(); i++){
		newDist = raycast_wall( origin,end, Vector2(worldVerticies[i-1].position.x,worldVerticies[i-1].position.y), Vector2(worldVerticies[i].position.x,worldVerticies[i].position.y));
		if (newDist != -1 && (std::isnan(minDist) || newDist < minDist)){
			minDist = newDist;
		}
	}
	return minDist;
}

/*
TODO:
Fix edge case of cylinder going across -pi,pi

*/
std::vector<Eigen::Vector2f> Scan::findCylinders(Scan::Observation* derivative, Scan::Observation* foundScan, float jump) {
	Scan::Observation* der = derivative;
	Scan::Observation* scan = foundScan;
	bool onCylinder = false;
	bool didLoop = false;
	float sumRay = 0;
	float sumDepth = 0;
	int rays = 0;
	int maxSize = der->distance.size();
	std::vector<Eigen::Vector2f> cylinderList = {}; 
	for (int i = 0; i < maxSize;i++) {
		if (der->distance[i] < -jump) {
			onCylinder = true;
			sumRay = 0;
			sumDepth = 0;
			rays = 0;
		}
		else if(der->distance[i] > jump && onCylinder && rays !=0) {
			if (rays <= 60){
				cylinderList.push_back(Eigen::Vector2f(sumRay / rays, sumDepth / rays));		
				if(didLoop){
					break;
				}
			}
			onCylinder = false;
		}
		else {
			if(didLoop){
				sumRay += der->theta[i]+2*M_PI;
				
			}
			else{
				sumRay += der->theta[i];
			}
			sumDepth += scan->distance[i];
			rays++;
		}
		if(i == (maxSize -1) && onCylinder){
			i = -1;
			didLoop = true;
		}
	}
	return cylinderList;
}

std::vector<Eigen::Matrix2f> Scan::getCylinders(float jump, std::vector<Eigen::Vector2f> cylinders,Vector2 origin) {
	std::vector<Eigen::Vector2f> cylinderList = cylinders;
	std::vector<Eigen::Matrix2f> toRet = {};
	Eigen::Matrix2f cylinder;
	float offset = 10;
	for (int i = 0; i < cylinderList.size();i++) {
		cylinderList[i][1] += offset;
		cylinder << cylinderList[i][1], cylinderList[i][0],
			cylinderList[i][1] * cos(cylinderList[i][0])+origin.x, cylinderList[i][1] * sin(cylinderList[i][0])+origin.y;
		if (!(isnan(cylinder(0,0)) || isnan(cylinder(0,1)) ) || isnan(cylinder(1,0))  || isnan(cylinder(1,1)) ){
			toRet.push_back(cylinder);
		}
		
	}
	return toRet;
}
