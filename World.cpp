#include "World.h"

World::World(float size[], float border[], sf::Color color) {
	this->size[0] = size[0];this->size[1] = size[1];
	this->border[0] = border[0];this->border[1] = border[1];
	this->color = color;
	this->edges = { {border[0], border[1], size[0] - border[0],border[1]},{ border[0], border[1], border[0],size[0] -  border[1]},{size[0] - border[0], border[1], size[0] -  border[0],size[1] - border[1] },{ border[0], size[1] -  border[1], size[0] - border[0],size[1] -  border[1] } };

	circles = { }; //TopRight,TopLeft,BotLeft,BotRight

	borderRect = sf::RectangleShape(sf::Vector2f(this->size[0] - 2 * this->border[0], this->size[1] - 2 * this->border[1]));
	borderRect.setPosition(sf::Vector2f(this->border[0], this->border[1]));
	borderRect.setOutlineThickness(5);
	borderRect.setFillColor(sf::Color::Black);
}


void World::addCircle(sf::CircleShape circle) {
	World::circles.push_back(circle);

}