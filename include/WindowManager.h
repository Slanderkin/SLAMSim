#pragma once
#include "StandardImports.h"
#include "Scan.h"
#include "Button.h"

class WindowManager
{
public:
    WindowManager();
    void clear();
    void display();
    void updateCenters();
    DrawView* requestDrawView(int win, sf::FloatRect view_rect, sf::FloatRect viewport_rect, Vector2 *center);

private:
    std::vector<DrawView *> drawViews;
    std::vector<sf::RenderWindow *> windows;
    std::vector<Button> buttons;
    sf::RenderWindow window;
};