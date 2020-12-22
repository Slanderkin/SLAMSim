#pragma once
#pragma once
#ifndef BUTTON_H
#define BUTTON_H
#include "StandardImports.h"

class Button {
private:
	float width;
	float height;
	float x;
	float y;
	
	

public:
	
	
	Button(float width, float height,float x,float y, sf::Color color,bool *toToggle, sf::Text& text);
	bool isClicked(float mouseX, float mouseY);
	bool checkToggle(float mouseX, float mouseY);
	void checkColor();
	bool *toToggle;
	sf::RectangleShape rect;
	sf::Color color;
	sf::Text text;
};



#endif