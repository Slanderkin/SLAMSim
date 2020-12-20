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

bool Button::checkToggle(float mouseX, float mouseY)
{
	if ((mouseX >= x && mouseX <= width + x) && (mouseY >= y && mouseY <= height + y))
	{
		*toToggle = !*toToggle;
		updateColor();
		return true;
	}
	return false;
}

void Button::updateColor() {
	if (*toToggle) {
		this->rect.setFillColor(sf::Color::Green);
	}
	else {
		this->rect.setFillColor(sf::Color::Red);
	}
}

bool Button::isClicked(float mouseX,float mouseY) {
	return((mouseX >= x && mouseX <= width + x) && (mouseY >= y && mouseY <= height + y));
	
}