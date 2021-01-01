#include "World.h"
#include "StandardImports.h"

World::World(){}

World::World(Vector2 size, Vector2 border, sf::Color color) {
	this->size = size;
	this->border = border;
	this->color = color;
	this->edges = { {border.x, border.y, size.x - border.x,border.y},{ border.x, border.y, border.x,size.x -  border.y},{size.x - border.x, border.y, size.x -  border.x,size.y - border.y },{ border.x, size.y -  border.y, size.x - border.x,size.y -  border.y } };
	this->drawWorld = false;

	this->drawViews = std::vector<DrawView*>();

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

void World::addDrawView(DrawView *dv)
{
	this->drawViews.push_back(dv);
}

void World::draw()
{
	Timer timer("World draw");
	for (DrawView *dv : drawViews)
	{
		dv->window->setView(*(dv->view));
		if(this->drawWorld == true)
		{
			dv->window->draw(this->borderRect);

			for (sf::CircleShape circle : this->circles) {
				dv->window->draw(circle);
			}
		}
	}
}