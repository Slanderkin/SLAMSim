#include "WindowManager.h"

WindowManager::WindowManager()
{
    drawViews = std::vector<DrawView *>();
    windows = std::vector<sf::RenderWindow *>();
    sf::RenderWindow *window = new sf::RenderWindow(sf::VideoMode(1400, 700), "Window");
    window->setFramerateLimit(60);
    windows.push_back(window);
}

DrawView* WindowManager::requestDrawView(int win, sf::FloatRect view_rect, sf::FloatRect viewport_rect, Vector2 *center)
{
    // Theres a problem with this if statement I think so probs broke for more windows currently
    if(win > windows.size()-1)
    {
        std::cout << "Creating window created" << std::endl;
        for(int i = windows.size(); i <= win; i++)
        {
            sf::RenderWindow *window = new sf::RenderWindow(sf::VideoMode(1400, 700), "Window");
            window->setFramerateLimit(60);
            windows.push_back(window);
            std::cout << "Window created" << std::endl;
        }
    }
    std::cout << "End of if" << std::endl;
    DrawView *frame = new DrawView;
    frame->window = windows[win];
    frame->view = new sf::View(view_rect);
    frame->view->setViewport(viewport_rect);
    if(center != NULL)
    {
        frame->center = center;
    }
    else
    {
        frame->center = new Vector2(view_rect.width/2, view_rect.height/2);
    }
    drawViews.push_back(frame);
    return frame;
    
}

void WindowManager::updateCenters()
{
    for (DrawView *dv : drawViews)
    {
        dv->view->setCenter(sf::Vector2f(dv->center->x, dv->center->y));
    }
}

void WindowManager::clear()
{
    for(sf::RenderWindow *w : windows) w->clear();
}

void WindowManager::display()
{
    for(sf::RenderWindow *w : windows) w->display();
}