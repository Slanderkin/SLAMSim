#pragma once

#include "StandardImports.h"


#ifndef WORLD_H
#define WORLD_H



class World {

private:

	void init(float size[], float border[], sf::Color color);

public:
	World(float size[], float border[],sf::Color color);

	float size [2];
	std::vector<std::vector<float>> edges;
	sf::Color color;
	float border [2];
	std::vector<sf::CircleShape> circles;

	sf::RectangleShape borderRect; 
	void addCircle(sf::CircleShape);

};
#endif