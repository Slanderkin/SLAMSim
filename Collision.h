#pragma once
#ifndef COLLISION_H
#define COLLISION_H

#include "StandardImports.h"

float* circlesCollided(float c1x,float c1y,float c2x,float c2y,float c1rad,float c2rad) {
	static float result[2];
	float distanceBetweenCirclesSquared = (c2x-c1x)*(c2x-c1x)+(c2y-c1y)*(c2y-c1y);
	if (distanceBetweenCirclesSquared < (c1rad + c2rad)* (c1rad + c2rad)) {
		float angle = atan2f(c2y - c1y,c2x - c1x);
		float distBetweenCircles = sqrt(distanceBetweenCirclesSquared);
		float distToMove = c1rad + c2rad - distBetweenCircles;
		result[0] = -cos(angle) * distToMove;
		result[1] = -sin(angle) * distToMove;
	}
	return result;
}




#endif