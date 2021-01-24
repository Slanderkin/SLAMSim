#pragma once
#ifndef LANDMARK_H
#define LANDMARK_H
#include "StandardImports.h"

class Landmark {
//**********MAY NEED TO BE DEPRICATED**********//

private:

public:
	Landmark();
	Landmark(Vector2 location);
	Landmark(Vector2 location, std::vector<float> variances);


	Vector2 location;
	std::vector<float> variances; //Sxx, Syy, Sxy
	
	sf::CircleShape landMark;
	sf::CircleShape errorEllipse;
	float ellipseHeading;

	void updateLocation(Vector2 newLocation);
	void updateEllipse();

};




#endif // 
