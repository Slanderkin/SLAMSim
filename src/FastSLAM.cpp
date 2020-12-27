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

	float l0 = control(0);
	float r0 = control(1);
	float lStd = sqrt((controlFactors(0) * l0) * (controlFactors(0) * l0) + (controlFactors(1) * (l0 - r0)) * (controlFactors(1) * (l0 - r0)));
	float rStd = sqrt((controlFactors(0) * r0) * (controlFactors(0) * r0) + (controlFactors(1) * (l0 - r0)) * (controlFactors(1) * (l0 - r0)));
	std::mt19937 generatorL(std::random_device{}());
	std::mt19937 generatorR(std::random_device{}());
	std::normal_distribution<double> distl(0,lStd);
	std::normal_distribution<double> distr(0, rStd);
	float l=0;
	float r=0;

	for (int i = 0;i < particles.size();i++) {
		l=l0;
		r=r0;
		if(l == -r){
			float delta = distl(generatorL);
			l -= delta;
			r += delta;
			std::cout << l << "__" << r << std::endl;
		}
		else{
		l += distl(generatorL);
		r += distr(generatorR);
		}
		particles[i].move(Eigen::Vector2f(l, r));
	}
}

std::vector<float> FastSLAM::updateComputeWeights(std::vector<Eigen::Matrix2f> cylinders) {
	Eigen::Matrix2f Qt_cov;
	Qt_cov << measurementStddev(0) * measurementStddev(0), 0,
		0, measurementStddev(1)* measurementStddev(1);
	std::vector<float> toRet;
	float numLandmarks = 0;
	float weight;
	for (int i = 0; i < particles.size();i++) {
		particles[i].decrementVisibleLandmarkCounters();
		numLandmarks = particles[i].landMarkLocations.size();
		weight = 1;
		for (int j = 0;j < cylinders.size();j++) {
			weight *= particles[i].update_particle(numLandmarks, minimumLikelihood, Eigen::Vector2f(cylinders[i](0,0), cylinders[i](0, 1)),Qt_cov);
		}
		toRet.push_back(weight);
		particles[i].removeBadLandmarks();
	}
	return toRet;
}

std::vector<Particle> FastSLAM::resample(std::vector<float> weights) {
	std::vector<Particle> toRet;
	float maxInd = std::distance(weights.begin(), std::max_element(std::begin(weights), std::end(weights)));
	int index = rand() % particles.size();
	float offset = 0;

	for (int i = 0;i < particles.size();i++) {
		offset += (float(rand()) / float((RAND_MAX)) * 2 * weights[maxInd]);
		while (offset > weights[index]) {
			offset -= weights[index];
			index = (index + 1) % weights.size();

		}
		toRet.push_back(particles[index]);
	}


	return toRet;
}

void FastSLAM::correct(std::vector<Eigen::Matrix2f> cylinders) {
	std::vector<float> weights = updateComputeWeights(cylinders);
	particles = resample(weights);

}