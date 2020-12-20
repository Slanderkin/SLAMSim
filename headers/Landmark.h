#pragma once
#ifndef LANDMARK_H
#define LANDMARK_H
#include "StandardImports.h"

class Landmark {


private:

public:

	Landmark(Vector2 location);


	float x;
	float y;
	
	sf::CircleShape landMark;
	sf::CircleShape errorEllipse;

	void updateLocation(Vector2 newLocation);

};




#endif // 
