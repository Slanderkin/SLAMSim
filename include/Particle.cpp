#include "Particle.h"
#include "StandardImports.h"

//using Vector2 = Eigen::Vector2f;

Particle::Particle(Eigen::Vector2f position, float heading) {

	this->position = position;
	this->heading = heading;
	this->landMarkLocations = {};
	this->landMarkCov = {};
	this->landmarkCounters = {};

}
/*
Moves this particle according to the velocity given.
vel is the linear and angular velocity respectively
*/
void Particle::move(Eigen::Vector2f vel) {
	//vel is linear,angular
	Eigen::Vector2f linVel( vel(0) * (float)cos(heading * M_PI / 180),vel(0) * (float)sin(heading * M_PI / 180) );
	
	if (vel(0) == vel(1)) {
		position = position + linVel;
	}
	//Right hand turn, subtracting angle
	else if (vel(1) < 0) {
		position = position + linVel;
		heading += vel(1);
	}
	//Lefft hand turn, adding angle
	else if (vel(1) > 0) {
		position = position + linVel;
		heading += vel(1);
	}

}

Eigen::Vector2f Particle::landMarkPose(int landMarkNum) {
	
	Eigen::Vector2f diff = landMarkLocations[landMarkNum] - position;
	float mag = sqrt(diff(0) * diff(0) + diff(1) * diff(1));
	float alpha = fmod(atan2(diff(1), diff(0)) - M_PI/180 * this->heading + M_PI, (2 * M_PI))-M_PI;

	return Eigen::Vector2f(mag, alpha);

}


Eigen::Matrix2f Particle::hForLandMark(int landMarkNum) {

	Eigen::Matrix2f toRet;
	toRet << landMarkPose(landMarkNum)(0), landMarkPose(landMarkNum)(1),
		 position(0), position(1);
	return toRet;
}

Eigen::Matrix2f Particle::dhLandmark(Eigen::Vector2f landMarkPos) {

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

std::vector<Eigen::Matrix2f> Particle::get_H_QL(int landMarkNum, Eigen::Matrix2f Qt_cov) {
	Eigen::Matrix2f H = dhLandmark(landMarkLocations[landMarkNum]);
	Eigen::Matrix2f landMarkCov = this->landMarkCov[landMarkNum];
	Eigen::Matrix2f QL = H*(landMarkCov*H.transpose()) + Qt_cov;
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

float Particle::getWl(int landMarkNum, Eigen::Vector2f measurement, Eigen::Matrix2f Qt_cov) {
	
	Eigen::Matrix2f h = hForLandMark(landMarkNum);
	Eigen::Vector2f deltaZ = measurement - Eigen::Vector2f(h(1,0), h(1, 1));
	std::vector<Eigen::Matrix2f> H_QL = get_H_QL(landMarkNum, Qt_cov);
	Eigen::Matrix2f QL = H_QL[1];
	float result = -0.5 * deltaZ.transpose() * (QL.inverse() * deltaZ);
	float wl = 1 / (2 * M_PI * sqrt(QL.determinant())) * pow(M_E, result);
	return wl;

}

/*
Gets the likelihood of the measurement for each landmark
Mesurement holds range,angle (deg)
Qt_cov holds the variances Sxx,Sxy,Syy:
	|Sxx,Sxy|
	|Sxy,Syy|
*/
std::vector<float> Particle::getLikelihoods(int numLandmarks, Eigen::Vector2f measurement, Eigen::Matrix2f Qt_cov) {
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


void Particle::initializeLandmark(Eigen::Vector2f measurement, Eigen::Matrix2f Qt_cov) {
	Eigen::Vector2f landmarkPos = {position(0)+measurement(0)*(float)cos(measurement(1)*M_PI/180),position(1) + measurement(0) *(float) sin(measurement(1) * M_PI / 180) };
	Eigen::Matrix2f H = dhLandmark(landmarkPos);
	Eigen::Matrix2f local_landMarkCov = H.inverse() * (Qt_cov * H.transpose());
	landMarkLocations.push_back(landmarkPos);
	landMarkCov.push_back(local_landMarkCov);
}
/*
Updates a landmark corresponding to landMarkNum for this particle
Mesurement holds range,angle (deg)
Qt_cov holds the variances Sxx,Sxy,Syy:
	|Sxx,Sxy|
	|Sxy,Syy|
*/

void Particle::updateLandmark(int landMarkNum, Eigen::Vector2f measurement, Eigen::Matrix2f Qt_cov) {
	Eigen::Matrix2f h = hForLandMark(landMarkNum);
	Eigen::Vector2f deltaZ = measurement - Eigen::Vector2f(h(1, 0), h(1, 1));;
	std::vector<Eigen::Matrix2f> H_QL = get_H_QL(landMarkNum, Qt_cov);
	Eigen::Matrix2f K = (landMarkCov[landMarkNum]* H_QL[0].transpose())* H_QL[1].inverse();
	landMarkLocations[landMarkNum] += K * deltaZ;
	landMarkCov[landMarkNum] = (Eigen::Matrix2f::Identity(2,2)-(K*H_QL[0]))* landMarkCov[landMarkNum];
}


float Particle::update_particle(int numLandmarks, float minLikleihood, Eigen::Vector2f measurement, Eigen::Matrix2f Qt_cov) {

	std::vector<float> likelihoods = getLikelihoods(numLandmarks, measurement, Qt_cov);
	//std::vector<float>::iterator result = );
	float maxIndex = likelihoods[std::distance(likelihoods.begin(), std::max_element(std::begin(likelihoods), std::end(likelihoods)))];

	if (likelihoods.size() == 0 ||  likelihoods[maxIndex]< minLikleihood) {
		initializeLandmark(measurement,Qt_cov);
		landmarkCounters.push_back(1);
		return minLikleihood;
	}
	else {
		updateLandmark(maxIndex, measurement, Qt_cov);
		landmarkCounters[maxIndex] += 2;
		return likelihoods[maxIndex];
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

