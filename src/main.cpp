#include "StandardImports.h"

#include "Robot.h"
#include "World.h"
#include "Scan.h"
#include "Button.h"
#include "Gui.h"
#include "Particle.h"
#include "FastSLAM.h"

Vector2 operator+(Vector2 a, Vector2 b)
{
    return { a.x + b.x, a.y + b.y };
}

Vector2 operator-(Vector2 a, Vector2 b)
{
    return { a.x - b.x, a.y - b.y };
}

Vector2 operator*(float c, Vector2 v)
{
    return { c * v.x, c * v.y };
}

Vector2 operator/(Vector2 v, float c)
{
    return { v.x / c, v.y / c };
}

int main()
{

    std::vector<DrawView *> drawViews = std::vector<DrawView *>();
    std::vector<sf::RenderWindow *> windows = std::vector<sf::RenderWindow *>();

    Vector2 size = { 1200,800 };
    Vector2 border = { 150,150 };

    Vector2 center = size/2;
    float heading = 0;
    Vector2 velocity = {5,3}; //Linear,angular
    float maxRange = 1000;
    float radius = 10.f;

    std::vector<Button> buttonList;

    //sf::RenderWindow window(sf::VideoMode((unsigned int)size.x, (unsigned int)size.y), "SLAM Sim!");
    sf::RenderWindow window(sf::VideoMode(1000, 500), "SLAM Sim!");
    windows.push_back(&window);
    //sf::RenderWindow window2(sf::VideoMode((unsigned int)size.x, (unsigned int)size.y), "SLAM Sim!");
    //windows.push_back(&window2);
    window.setFramerateLimit(60);

    // Initialize World
    World world(size, border, sf::Color::White);    
    

    // Initialize Robot
    Robot robot(center, heading, sf::Color::Red, velocity, maxRange, radius, &world);
    

    DrawView *env_frame = new DrawView;
    env_frame->window = &window;
    env_frame->view = new sf::View(sf::FloatRect(0.f, 0.f, 1200.f, 800.f));
    env_frame->view->setViewport(sf::FloatRect(0.25f, 0.f, 0.75f, 1.f));
    env_frame->center = new Vector2(600.f,400.f);
    drawViews.push_back(env_frame);
    world.addDrawView(env_frame);
    robot.addDrawView(env_frame);

    DrawView *robot_frame = new DrawView;
    robot_frame->window = &window;
    robot_frame->view = new sf::View(sf::FloatRect(0.f, 0.f, 800.f, 1600.f));
    robot_frame->view->setViewport(sf::FloatRect(0.f, 0.f, 0.25f, 1.f));
    robot_frame->center = &(robot.center);
    drawViews.push_back(robot_frame);
    world.addDrawView(robot_frame);
    robot.addDrawView(robot_frame);

    DrawView *button_frame = new DrawView;
    button_frame->window = &window;
    button_frame->view = new sf::View(sf::FloatRect(0.f, 0.f, 1600.f, 200.f));
    button_frame->view->setViewport(sf::FloatRect(0.f, 0.f, 1.f, 0.25f));
    button_frame->center = new Vector2(800,100);
    drawViews.push_back(button_frame);
    //Particle initialization
    int numParticles = 10;
    std::vector<Particle> initialParticles;
    for (int i = 0;i < numParticles;i++) {
        initialParticles.push_back(Particle(Eigen::Vector2f(200, 200), 45, sf::CircleShape(10, 3)));
    }
    FastSLAM fastSLAM(robot.radius, Eigen::Vector2f(.1, .1), Eigen::Vector2f(20, 15), 0.001,initialParticles);

    //Import the fount
    sf::Font font;
    font.loadFromFile("tahoma.ttf");

    // Button value setup
    bool *bools[3] = { &world.drawWorld, &robot.scan.doGaussian, &robot.drawRays };
    float sizes[3][2] = { {150.f,50.f}, {150.f,50.f},{150.f,50.f} };
    float pos[3][2] = { {10.f,10.f},{200.f,10.f},{390.f,10.f} };
    std::string textStr[3] = { "Draw World","Gaussian", "Draw Lines" };

    // Button creation
    sf::Color color = sf::Color::Red;
    for (int i = 0; i < 3; i++) {
        sf::Text text(textStr[i], font, 24);
        text.setFillColor(sf::Color::White);
        Button button(sizes[i][0], sizes[i][1], pos[i][0], pos[i][1], color, bools[i], text);
        buttonList.push_back(button);
    }
    
    
    
    // Main loop
    while (window.isOpen())
    {
        
        // Section where update events are called 
        robot.update();
        
        //===============Draw Section===============
        for(sf::RenderWindow *w : windows) w->clear();
        for (DrawView *dv : drawViews)
        {
            dv->view->setCenter(sf::Vector2f(dv->center->x, dv->center->y));
            //printf("x: %f, y: %f", dv->center->x, dv->center->y);
        }

        // Have world and robot draw what they need
        world.draw();
        robot.draw();

        
        for (int i = 0;i < fastSLAM.particles.size();i++) {
            robot_frame->window->setView(*(robot_frame->view));
            robot_frame->window->draw(fastSLAM.particles[i].marker);
            env_frame->window->setView(*(env_frame->view));
            env_frame->window->draw(fastSLAM.particles[i].marker);
        }
        
        // Draw the buttons
        button_frame->window->setView(*(button_frame->view));
        for (int i = 0; i < buttonList.size();i++) {
            window.draw(buttonList[i].rect);
            window.draw(buttonList[i].text);
        }
    
        // Display the completed window
    for(sf::RenderWindow *w : windows) w->display();
        env_frame->window->setView(*(env_frame->view));

        // ===============Update Section===============
        sf::Event event;
        while (window.pollEvent(event))
        {
            switch (event.type) {
            case sf::Event::Closed:
                window.close();
                break;
                
            case sf::Event::MouseButtonPressed:
                //
                if (event.mouseButton.button == sf::Mouse::Left){
                    float rad = 10;
                    sf::Vector2f worldMousePos = window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
                    if (!(worldMousePos.x - rad < world.border.x || worldMousePos.y - rad < world.border.y || worldMousePos.x +rad > world.size.x - world.border.x || worldMousePos.y + rad> world.size.y - world.border.y)) {
                        sf::CircleShape newCircle(rad);
                        newCircle.setPosition(worldMousePos - sf::Vector2f(rad, rad));
                        world.addCircle(newCircle);
                    }
                    
                    for (int i = 0; i < buttonList.size(); i++) {
                        if (buttonList[i].isClicked((float)event.mouseButton.x, (float)event.mouseButton.y)) {
                            buttonList[i].toggle();
                            break;
                        }
                    }

                }
                break;
            
            case sf::Event::KeyPressed:
                if (event.key.code == sf::Keyboard::R) {
                    buttonList[2].toggle();
                }
                else if (event.key.code == sf::Keyboard::G) {
                    buttonList[1].toggle();
                }
                else if (event.key.code == sf::Keyboard::F) {
                    buttonList[0].toggle();

                }
                
            default:
                break;
            }
                
        }

        Vector2 control(0, 0);
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
        {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
                control = { 6, 4 };
            }
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
                control = { 4,6 };
            }
            else {
                // w key is pressed: move our character
                control = { 5,5 };
            }

        }
        else {
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::A)){
                control = { 14, -14 };

            }
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
                control = { -14, 14 };

            }

        }
        robot.move(world, control);
        if(control.y != 0 && control.x !=0){
            fastSLAM.predict(Eigen::Vector2f(control.x, control.y));
            //fastSLAM.correct(robot.scan.cylinders);
        }
    }

    return 0;
}


