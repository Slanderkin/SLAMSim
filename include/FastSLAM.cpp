#include "FastSLAM.h"


FastSLAM::FastSLAM(float robotWidth, Eigen::Vector2f controlFactors, Eigen::Vector2f measurementStddev, float minimumLikelihood, std::vector<Particle> initialParticles) :
dist(mean, stddev),
generator(std::random_device{}())
 {
	this->robotWidth = robotWidth;
	this->controlFactors = controlFactors;
	this->measurementStddev = measurementStddev;
	this->minimumLikelihood = minimumLikelihood;
	this->particles = initialParticles;


}


void FastSLAM::predict(Eigen::Vector2f control) {

	float l = control(0);
	float r = control(1);
	float lStd = sqrt((controlFactors(0)*l)* (controlFactors(0) * l) + (controlFactors(1)*(l-r))* (controlFactors(1) * (l - r)));
	float rStd = sqrt((controlFactors(0) * r) * (controlFactors(0) * r) + (controlFactors(1) * (l - r)) * (controlFactors(1) * (l - r)));


	std::mt19937 generator(std::random_device{}());
	std::normal_distribution<double> distl(0,lStd);
	std::normal_distribution<double> distr(0, rStd);


	for (int i = 0;i < particles.size();i++) {

		l += distl(generator);
		r += distr(generator);
		particles[i].move(Eigen::Vector2f(l, r));
	}
}