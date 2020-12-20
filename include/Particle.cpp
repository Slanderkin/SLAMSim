#include "Particle.h"
#include "StandardImports.h"


Particle::Particle(Vector2 position, float heading) {

	this->position = position;
	this->heading = heading;


}

void Particle::move(Vector2 vel) {
	//vel is linear,angular
	Vector2 linVel = { vel.x * (float)cos(heading * M_PI / 180),vel.x * (float)sin(heading * M_PI / 180) };
	
	if (vel.x == vel.y) {
		position = position + Vector2(linVel.x,linVel.y);
	}
	//Right hand turn, subtracting angle
	else if (vel.y < 0) {
		position = position + Vector2(linVel.x, linVel.y);
		heading += vel.y;
	}
	//Lefft hand turn, adding angle
	else if (vel.y > 0) {
		position = position + Vector2(linVel.x, linVel.y);
		heading += vel.y;
	}

}

Vector2 Particle::landMarkPose(int landMarkNum) {

	Vector2 diff = landMarkLocations[landMarkNum] - position;
	float mag = sqrt(diff.x * diff.x + diff.y * diff.y);
	float alpha = fmod(atan2(diff.y, diff.x) - M_PI * this->heading + M_PI, (2 * M_PI))-M_PI;

	return Vector2(mag, alpha);
}


std::vector<Vector2> Particle::hForLandMark(int landMarkNum) {

	std::vector<Vector2> toRet = { landMarkPose(landMarkNum), position };
	return toRet;
}

std::vector<Vector2> Particle::dhLandmark(Vector2 landMarkPos) {

	Vector2 diff = landMarkPos - position;
	float q = diff.x * diff.x + diff.y * diff.y;
	float qSqrt = sqrt(q);
	float dr_dmx = diff.x / qSqrt;
	float dr_dmy = diff.y / qSqrt;
	float dalpha_dmx = -diff.y / q;
	float dalpha_dmy = diff.x / q;

	std::vector<Vector2> toRet = { Vector2(dr_dmx,dr_dmy), Vector2(dalpha_dmx,dalpha_dmy) };
	return toRet;
}

std::vector<Vector2> Particle::get_H_QL(int landMarkNum, std::vector<Vector2> Qt_cov) {
	std::vector<Vector2> H = dhLandmark(landMarkLocations[landMarkNum]);
	std::vector<Vector2> landMarkCov = this->landMarkCov[landMarkNum];
	float tl = H[0].x * (landMarkCov[0].x * H[0].x + landMarkCov[0].y * H[0].y) + H[0].y * (landMarkCov[1].x * H[0].x + landMarkCov[1].y * H[0].y) + Qt_cov[0].x;
	float tr = H[0].x * (landMarkCov[0].x * H[1].x + landMarkCov[0].y * H[1].y) + H[0].y * (landMarkCov[1].x * H[1].x + landMarkCov[1].y * H[1].y) + Qt_cov[0].y;
	float bl = H[1].x * (landMarkCov[0].x * H[0].x + landMarkCov[0].y * H[0].y) + H[1].y * (landMarkCov[1].x * H[0].x + landMarkCov[1].y * H[0].y) + Qt_cov[1].x;
	float br = H[1].x * (landMarkCov[0].x * H[1].x + landMarkCov[0].y * H[1].y) + H[1].y * (landMarkCov[1].x * H[1].x + landMarkCov[1].y * H[1].y) + Qt_cov[1].y;
	std::vector<Vector2> toRet= { H[0],H[1], Vector2(tl,tr),Vector2(bl,br)};
	return toRet;

}

//Under construction
float Particle::getWl(int landMarkNum,float measurement, std::vector<Vector2> Qt_cov) {
	std::vector<Vector2> h = hForLandMark(landMarkNum);
	float deltaZ = measurement - h[0].x;
	std::vector<Vector2> H_QL = get_H_QL(landMarkNum, Qt_cov);
	return 0;
}