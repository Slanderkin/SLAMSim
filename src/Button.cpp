#include "Button.h"
#include "StandardImports.h"

Button::Button(float width,float height,float x,float y, sf::Color color, bool *toToggle, sf::Text& text) {
	this->toToggle = toToggle;
	this->width = width;
	this->height = height;
	this->x = x;
	this->y = y;
	this->color = color;
	this->rect = sf::RectangleShape(sf::Vector2f(width,height));
	rect.setPosition(sf::Vector2f(x, y));
	rect.setFillColor(color);

	
	this->text = text;
	this->text.setPosition(sf::Vector2f(x + width / 2-text.getLocalBounds().width/2, y + height / 2- text.getLocalBounds().height));

}

bool Button::toggle()
{
	*toToggle = !*toToggle;
	rect.setFillColor((*toToggle)?(sf::Color::Green):(sf::Color::Red));
	return *toToggle;
}


//Deprecated
bool Button::isClicked(float mouseX,float mouseY) {
	return((mouseX >= x && mouseX <= width + x) && (mouseY >= y && mouseY <= height + y));
}