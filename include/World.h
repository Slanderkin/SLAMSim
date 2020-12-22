#pragma once

#include "StandardImports.h"


#ifndef WORLD_H
#define WORLD_H



class World {


public:
	World(Vector2 size, Vector2 border,sf::Color color);

	Vector2 size;
	Vector2 border;
	std::vector<std::vector<float>> edges;
	sf::Color color;
	std::vector<sf::CircleShape> circles;
	sf::RectangleShape borderRect; 
	void addCircle(sf::CircleShape);
	bool drawWorld;
};
#endif