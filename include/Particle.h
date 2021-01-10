#pragma once
#ifndef PARTICLE_H
#define PARTICLE_H
#include "StandardImports.h"


class Particle {

private:
	Eigen::Vector2f landMarkPose(const int landMarkNum);

public:

	Particle(Eigen::Vector2f position, float heading, sf::CircleShape marker);
	
	Eigen::Vector2f position;
	float heading;
	float radius;
	sf::CircleShape marker;
	
	std::vector<float> landmarkCounters;
	std::vector<Eigen::Vector2f> landMarkLocations;
	std::vector<Eigen::Matrix2f> landMarkCov;
	
	
	
	//==========Computational Functions==========//
	void move(const Eigen::Vector2f& vel);
	Eigen::Matrix2f hForLandMark(const int landMarkNum);
	Eigen::Matrix2f dhLandmark(const Eigen::Vector2f& landMarkPos);
	std::vector<Eigen::Matrix2f> get_H_QL(const int landMarkNum, const Eigen::Matrix2f& Qt_cov);
	float getWl(int landMarkNum,const Eigen::Vector2f& measurement, const Eigen::Matrix2f& Qt_cov);
	std::vector<float> getLikelihoods(int numLandmarks, const Eigen::Vector2f& measurement,const Eigen::Matrix2f& Qt_cov);
	bool initializeLandmark(const Eigen::Vector2f& measurement,const Eigen::Matrix2f& Qt_cov);
	void updateLandmark(int landMarkNum, const Eigen::Vector2f& measurement, const Eigen::Matrix2f& Qt_cov);
	float update_particle(int numLandmarks,float minLikleihood, const Eigen::Vector2f& measurement, const Eigen::Matrix2f& Qt_cov);
	void decrementVisibleLandmarkCounters(float maxRange);
	void removeBadLandmarks();
	Particle makeDeepCopy();
};



#endif // !PARTICLE_H
