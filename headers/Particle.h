#pragma once
#ifndef PARTICLE_H
#define PARTICLE_H
#include "StandardImports.h"
class Particle {

private:
	Vector2 landMarkPose(int landMarkNum);

public:

	Particle(Vector2 position, float heading);
	Vector2 position;
	float heading;
	std::vector<Vector2> landMarkLocations;
	std::vector< std::vector<Vector2>> landMarkCov;

	void move(Vector2 vel);
	std::vector<Vector2> hForLandMark(int landMarkNum);
	std::vector<Vector2> dhLandmark(Vector2 landMarkPos);
	std::vector<Vector2> get_H_QL(int landMarkNum,std::vector<Vector2> Qt_cov);
	float getWl(int landMarkNum,float measurement, std::vector<Vector2> Qt_cov);

};



#endif // !PARTICLE_H
