#include "WindowManager.h"

WindowManager::WindowManager(sf::RenderWindow &w)
{
    this->rays_on = false;
}

void WindowManager::addButton(sf::Text t, sf::Color c, bool *b)
{
    
}

void WindowManager::attachObservation(Scan::Observation *o)
{
    this->obs = o;
}