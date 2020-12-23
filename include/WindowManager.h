#pragma once
#include "StandardImports.h"
#include "Scan.h"

class WindowManager
{
public:
    WindowManager(sf::RenderWindow &w);
    void attachObservation(Span::Observation *o);
    
    void addButton(sf::Text t, sf::Color c, bool b);
    void addDrawEvent(void (*f)());
    void update();

private:
    Span::Observation *obs;
    std::vector<void (*)()> drawEvents;
    bool rays_on;
    bool noise_on;
    bool environment_on;
    std::vector<Button> buttons;
    sf::RenderWindow window;
}