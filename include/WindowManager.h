#pragma once
#include "StandardImports.h"
#include "Scan.h"
#include "Button.h"

class WindowManager
{
public:
    WindowManager(sf::RenderWindow &w);
    void attachObservation(Scan::Observation *o);
    void addButton(sf::Text t, sf::Color c, bool *b);
    void update();

private:
    Scan::Observation *obs;
    bool rays_on;
    bool noise_on;
    bool environment_on;
    std::vector<Button> buttons;
    sf::RenderWindow window;
};