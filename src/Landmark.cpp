#include "Landmark.h"
#include "StandardImports.h"

Landmark::Landmark(Vector2 location, std::vector<float> variances) {
	this->location = location;
	this->variances = variances;
	this->ellipseHeading = 20;

	this->landMark = sf::CircleShape(10);
	landMark.setFillColor(sf::Color::Magenta);
	landMark.setRadius(10.f);
	landMark.setPosition(sf::Vector2f(location.x,location.y));

	this->errorEllipse = sf::CircleShape(20);
	errorEllipse.setFillColor(sf::Color::Transparent);
	errorEllipse.setOutlineColor(sf::Color::Blue);
	errorEllipse.setOutlineThickness(2);
	errorEllipse.setRadius(10.f);
	errorEllipse.setPosition(sf::Vector2f(location.x, location.y));
	errorEllipse.setRotation(ellipseHeading);
}

Landmark::Landmark(Vector2 location) {
	this->location = location;
	this->variances = { 0,0,0 };
	this->ellipseHeading = 20;

	this->landMark = sf::CircleShape(10);
	landMark.setFillColor(sf::Color::Magenta);
	landMark.setRadius(10.f);
	landMark.setPosition(sf::Vector2f(location.x, location.y));

	this->errorEllipse = sf::CircleShape(20);
	errorEllipse.setFillColor(sf::Color::Transparent);
	errorEllipse.setOutlineColor(sf::Color::Blue);
	errorEllipse.setOutlineThickness(2);
	errorEllipse.setRadius(10.f);
	errorEllipse.setPosition(sf::Vector2f(location.x, location.y));
	errorEllipse.setRotation(ellipseHeading);
}

Landmark::Landmark() {
	this->location = { -1,-1 };
	this->variances = { 0,0,0 };
	this->ellipseHeading = 20;

	this->landMark = sf::CircleShape(10);
	landMark.setFillColor(sf::Color::Magenta);
	landMark.setRadius(10.f);
	landMark.setPosition(sf::Vector2f(location.x, location.y));

	this->errorEllipse = sf::CircleShape(20);
	errorEllipse.setFillColor(sf::Color::Transparent);
	errorEllipse.setOutlineColor(sf::Color::Blue);
	errorEllipse.setOutlineThickness(2);
	errorEllipse.setRadius(10.f);
	errorEllipse.setPosition(sf::Vector2f(location.x, location.y));
	errorEllipse.setRotation(ellipseHeading);
}


void Landmark::updateLocation(Vector2 newLocation) {

}

void Landmark::updateEllipse() {




}