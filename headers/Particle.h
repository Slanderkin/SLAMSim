#pragma once
#ifndef PARTICLE_H
#define PARTICLE_H
#include "StandardImports.h"


class Particle {

private:
	Eigen::Vector2f landMarkPose(int landMarkNum);

public:

	Particle(Eigen::Vector2f position, float heading);
	Eigen::Vector2f position;
	float heading;
	std::vector<float> landmarkCounters;
	std::vector<Eigen::Vector2f> landMarkLocations;
	std::vector<Eigen::Matrix2f> landMarkCov;

	void move(Eigen::Vector2f vel);
	Eigen::Matrix2f hForLandMark(int landMarkNum);
	Eigen::Matrix2f dhLandmark(Eigen::Vector2f landMarkPos);
	std::vector<Eigen::Matrix2f> get_H_QL(int landMarkNum, Eigen::Matrix2f);
	float getWl(int landMarkNum, Eigen::Vector2f measurement, Eigen::Matrix2f Qt_cov);
	std::vector<float> getLikelihoods(int numLandmarks, Eigen::Vector2f measurement, Eigen::Matrix2f Qt_cov);
	void initializeLandmark(Eigen::Vector2f measurement, Eigen::Matrix2f Qt_cov);
	void updateLandmark(int landMarkNum, Eigen::Vector2f measurement, Eigen::Matrix2f Qt_cov);
	float update_particle(int numLandmarks,float minLikleihood, Eigen::Vector2f measurement, Eigen::Matrix2f Qt_cov);
	void decrementVisibleLandmarkCounters();
	void removeBadLandmarks();
};



#endif // !PARTICLE_H
