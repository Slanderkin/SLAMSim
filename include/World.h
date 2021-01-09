#pragma once

#include "StandardImports.h"


#ifndef WORLD_H
#define WORLD_H



class World {

private:
	int getFileSize(std::string fileName);
	
	std::string fileName;
	int fileRows;

public:
	World(Vector2 size, Vector2 border,sf::Color color);
	World();
	void addDrawView(DrawView *dv);
	void draw();

	bool drawWorld;
	Vector2 size;
	Vector2 border;
	sf::Color color;
	sf::VertexArray worldVerticies;
	std::vector<sf::CircleShape> circles;
	std::vector<DrawView*> drawViews;

	sf::RenderWindow *window;

	
	





	void addCircle(sf::CircleShape);
	void loadWorldVerticies(int numRows);

};
#endif