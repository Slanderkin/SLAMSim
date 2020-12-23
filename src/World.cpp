#include "World.h"
#include "StandardImports.h"

World::World(){}

World::World(Vector2 size, Vector2 border, sf::Color color) {
	this->size = size;
	this->border = border;
	this->color = color;
	this->edges = { {border.x, border.y, size.x - border.x,border.y},{ border.x, border.y, border.x,size.x -  border.y},{size.x - border.x, border.y, size.x -  border.x,size.y - border.y },{ border.x, size.y -  border.y, size.x - border.x,size.y -  border.y } };
	this->drawWorld = false;

	circles = std::vector<sf::CircleShape>(); //TopRight,TopLeft,BotLeft,BotRight

	borderRect = sf::RectangleShape(sf::Vector2f(this->size.x - 2 * this->border.x, this->size.y - 2 * this->border.y));
	borderRect.setPosition(sf::Vector2f(this->border.x, this->border.y));
	borderRect.setOutlineThickness(5);
	borderRect.setOutlineColor(sf::Color::Magenta);
	borderRect.setFillColor(sf::Color::Black);
	
}

void World::addCircle(sf::CircleShape circle) {
	circle.setFillColor(sf::Color::Magenta);

	this->circles.push_back(circle);

}

void World::attachWindow(sf::RenderWindow *w)
{
	this->window = w;
}

void World::draw()
{
	if(this->drawWorld == true)
	{
		this->window->draw(this->borderRect);

		for (sf::CircleShape circle : this->circles) {
			this->window->draw(circle);
		}
	}
}