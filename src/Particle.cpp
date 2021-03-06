#include "Particle.h"
#include "StandardImports.h"

//using Vector2 = Eigen::Vector2f;

Particle::Particle(Eigen::Vector2f position, float heading, sf::CircleShape marker) {

	this->position = position;
	this->heading = heading;
	this->landMarkLocations = {};
	this->landMarkCov = {};
	this->landmarkCounters = {};
	this->marker = marker;
	this->marker.setOrigin(marker.getRadius(),marker.getRadius());
	this->marker.setPosition(position[0],position[1]);
	this->marker.setRotation(heading-90);
	this->radius = 10;

}

/*
Particle::Particle(const Particle& other)
	:heading(other.heading),radius(other.radius)
{
	position = Eigen::Vector2f(other.position[0],other.position[1]);
	marker = sf::CircleShape(radius);
	marker.setOrigin(radius,radius);
	marker.setPosition(position[0],position[1]);
	marker.setRotation(heading-90);
}*/

/*
Moves this particle according to the velocity given.
vel is the linear and angular velocity respectively
*/
void Particle::move(const Eigen::Vector2f& vel) {

	float l = vel(0);
	float r = vel(1);
	float alpha = 0;
	float R = 0;
	float cx = marker.getPosition().x;
	float cy = marker.getPosition().y;
	float x1 = cx;
	float y1 = cy;


	if (r == -l) {
		if (r < l) {
			heading -= 2*(l-r)/radius;

		}
		else {
			heading += 2*(r-l)/radius;

		}
		marker.setRotation(heading - 90);
		return;
	}
	else if (r < l) {
		alpha = (l - r) / (radius*2);
		R = r / alpha;
		x1 = cx + (R + radius) * (sin(heading*M_PI/180)-sin(heading * M_PI / 180-alpha));
		y1 = cy + (R + radius) * (-cos(heading * M_PI / 180) + cos(heading * M_PI / 180 - alpha));
		heading = (fmod((heading * M_PI / 180 - alpha + M_PI), (2 * M_PI)) - M_PI)*180/M_PI;
	}
	else if (l < r) {
		alpha = (r - l) / (radius * 2);
		R = l / alpha;
		x1 = cx + (R + radius) * (-sin(heading * M_PI / 180) + sin(heading * M_PI / 180 + alpha));
		y1 = cy + (R + radius) * (cos(heading * M_PI / 180) - cos(heading * M_PI / 180 + alpha));
		heading = (fmod((heading * M_PI / 180 + alpha + M_PI), (2 * M_PI)) - M_PI)*180/M_PI;

	}
	else {
		x1 = cx + l * cos(heading * M_PI / 180);
		y1 = cy + l * sin(heading * M_PI / 180);
	}
	position = {x1,y1};
	marker.setPosition(sf::Vector2f(x1,y1));
	marker.setRotation(heading - 90);

}

Eigen::Vector2f Particle::landMarkPose(const int landMarkNum) {
	
	Eigen::Vector2f diff = landMarkLocations[landMarkNum] - position;
	float mag = sqrt(diff(0) * diff(0) + diff(1) * diff(1));
	float alpha = fmod(atan2(diff(1), diff(0)) + M_PI, (2 * M_PI))-M_PI;

	return Eigen::Vector2f(mag, alpha);

}


Eigen::Matrix2f Particle::hForLandMark(const int landMarkNum) {

	Eigen::Matrix2f toRet;
	Eigen::Vector2f pose = landMarkPose(landMarkNum);
	toRet << pose[0], pose[1],
		 position[0], position[1];
	return toRet;
}

Eigen::Matrix2f Particle::dhLandmark(const Eigen::Vector2f& landMarkPos) {

	Eigen::Vector2f diff = landMarkPos - position;
	float q = diff(0) * diff(0) + diff(1) * diff(1);
	float qSqrt = sqrt(q);
	float dr_dmx = diff(0) / qSqrt;
	float dr_dmy = diff(1) / qSqrt;
	float dalpha_dmx = -diff(1) / q;
	float dalpha_dmy = diff(0) / q;

	Eigen::Matrix2f toRet;
	toRet << dr_dmx, dr_dmy,
		dalpha_dmx, dalpha_dmy;
	return toRet;

}

std::vector<Eigen::Matrix2f> Particle::get_H_QL(const int landMarkNum, const Eigen::Matrix2f& Qt_cov) {
	Eigen::Matrix2f H = dhLandmark(landMarkLocations[landMarkNum]);
	Eigen::Matrix2f landMarkCov = this->landMarkCov[landMarkNum];
	Eigen::Matrix2f QL = Qt_cov;
	QL.noalias() += H*(landMarkCov * H.transpose());

	std::vector<Eigen::Matrix2f> toRet = {H,QL};
	return toRet;

}

/*
 Gets the likelihood of a measurement corresponding to a landmark
 Mesurement holds range,angle (deg)
 Qt_cov holds the variances Sxx,Sxy,Syy:
	|Sxx,Sxy|
	|Sxy,Syy|
*/

float Particle::getWl(int landMarkNum,const Eigen::Vector2f& measurement, const Eigen::Matrix2f& Qt_cov) {
	
	Eigen::Matrix2f h = hForLandMark(landMarkNum);
	Eigen::Vector2f deltaZ = measurement - Eigen::Vector2f(h(0,0), h(0, 1));
	std::vector<Eigen::Matrix2f> H_QL = get_H_QL(landMarkNum, Qt_cov);
	Eigen::Matrix2f QL = H_QL[1];
	float determinate;
	bool isInvertible;
	Eigen::Matrix2f inverse;
	QL.computeInverseAndDetWithCheck(inverse,determinate,isInvertible);
	float result;
	float wl;
	float stuff;
	if(isInvertible){
		stuff = deltaZ.transpose() * (inverse * deltaZ);
		result = -0.5 * deltaZ.transpose() * (inverse * deltaZ);
		wl = 1 / (2 * M_PI * sqrt(determinate)) * std::exp(result);
		
		if (!(isinf(wl))){
			return wl;
		}
		else{
			
		}
		
	}
	return 0;	

}

/*
Gets the likelihood of the measurement for each landmark
Mesurement holds range,angle (deg)
Qt_cov holds the variances Sxx,Sxy,Syy:
	|Sxx,Sxy|
	|Sxy,Syy|
*/
std::vector<float> Particle::getLikelihoods(int numLandmarks,const Eigen::Vector2f& measurement,const Eigen::Matrix2f& Qt_cov) {
	std::vector<float> toRet;
	for (int i = 0;i < numLandmarks;i++) {
		toRet.push_back(getWl(i,measurement,Qt_cov));
	}
	return toRet;
}

/*
Adds a landmark to this particle
Mesurement holds range,angle (deg)
Qt_cov holds the variances Sxx,Sxy,Syy:
	|Sxx,Sxy|
	|Sxy,Syy|
*/
bool Particle::initializeLandmark(const Eigen::Vector2f& measurement,const Eigen::Matrix2f& Qt_cov) {
	Eigen::Vector2f landmarkPos = {measurement[0]*cos(measurement[1])+position[0] ,measurement[0]*sin(measurement[1])+position[1]};
	Eigen::Matrix2f H = dhLandmark(landmarkPos);
	bool isInvertible;
	Eigen::Matrix2f inverse;
	H.computeInverseWithCheck(inverse,isInvertible);
	if (isInvertible){
		Eigen::Matrix2f local_landMarkCov = inverse * (Qt_cov * inverse.transpose());
		landMarkLocations.push_back(landmarkPos);
		landMarkCov.push_back(local_landMarkCov);
		return true;
	}
	return false;
}
/*
Updates a landmark corresponding to landMarkNum for this particle
Mesurement holds range,angle (deg)
Qt_cov holds the variances Sxx,Sxy,Syy:
	|Sxx,Sxy|
	|Sxy,Syy|
*/

void Particle::updateLandmark(int landMarkNum, const Eigen::Vector2f& measurement, const Eigen::Matrix2f& Qt_cov) {
	Eigen::Matrix2f h = hForLandMark(landMarkNum);

	Eigen::Vector2f deltaZ = measurement - Eigen::Vector2f(h(0, 0), h(0, 1));
	std::vector<Eigen::Matrix2f> H_QL = get_H_QL(landMarkNum, Qt_cov);
	Eigen::Matrix2f K = (landMarkCov[landMarkNum]* H_QL[0].transpose())* H_QL[1].inverse();
	if (!(isnan(K(0)) || isnan(K(1)) || isnan(K(2)) || isnan(K(3)) || isnan(deltaZ(0))) || isnan(deltaZ(1))  ){
		landMarkLocations[landMarkNum] += K * deltaZ;
		landMarkCov[landMarkNum] = (Eigen::Matrix2f::Identity(2,2)-(K*H_QL[0]))* landMarkCov[landMarkNum];
	}
	
}


float Particle::update_particle(int numLandmarks, float minLikleihood, const Eigen::Vector2f& measurement, const Eigen::Matrix2f& Qt_cov) {

	std::vector<float> likelihoods = getLikelihoods(numLandmarks, measurement, Qt_cov);

	float maxIndex;
	float maxLikelihood =-1;
	if (likelihoods.size() != 0) {
		for(int i =0;i < likelihoods.size(); i++){
			if(likelihoods[i] > maxLikelihood){
				maxIndex = i;
				maxLikelihood = likelihoods[i];
			}
		}
	}
	
	if (likelihoods.size() == 0 || maxLikelihood < minLikleihood) {
		bool didInit = initializeLandmark(measurement,Qt_cov);
		if (didInit){landmarkCounters.push_back(1);}
		return minLikleihood;
	}
	else {
		updateLandmark(maxIndex, measurement, Qt_cov);
		landmarkCounters[maxIndex] += 2;
		return maxLikelihood;
	}
}

void Particle::decrementVisibleLandmarkCounters() {

	for (int i = 0; i < landmarkCounters.size(); i++) {
		landmarkCounters[i]--;
	}

}


void Particle::removeBadLandmarks() {

	std::vector<float> counter = {};
	std::vector<Eigen::Vector2f> location = {};
	std::vector<Eigen::Matrix2f> cov = {};

	for (int i = 0; i < landmarkCounters.size(); i++) {
		if (landmarkCounters[i] >= 0)
		{
			counter.push_back(landmarkCounters[i]);
			location.push_back(landMarkLocations[i]);
			cov.push_back(landMarkCov[i]);
		}
			
	}
	landmarkCounters = counter;
	landMarkLocations = location;
	landMarkCov = cov;



}