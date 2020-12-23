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

void draw_obs(sf::RenderWindow &win, Scan::Observation *obs, Vector2 robot_center, bool draw_rays)
{
    float circ_size = 2;
    sf::VertexArray line(sf::Lines, 2);
    sf::CircleShape end_circ(circ_size);
    sf::Vector2f circ_offset = sf::Vector2f(circ_size, circ_size);
    end_circ.setFillColor(sf::Color::Green);
    sf::Vector2f end_pos;
    for (int i = 0; i < obs->theta.size(); i++)
    {
        end_pos = sf::Vector2f(robot_center.x + obs->distance[i] * cos(obs->theta[i]), robot_center.y + obs->distance[i] * sin(obs->theta[i]));
        end_circ.setPosition(end_pos - circ_offset);
        win.draw(end_circ);
        if(draw_rays)
        {
            line[0].position = sf::Vector2f(robot_center.x, robot_center.y);
            line[1].position = end_pos;
            win.draw(line);
        }
    }
           
}

int main()
{
    Vector2 size = { 1000,800 };
    Vector2 border = { 150,150 };

    Vector2 center = size/2;
    float heading = 0;
    Vector2 velocity = {5,3}; //Linear,angular
    float maxRange = 1000;
    float radius = 10.f;

    bool doLine = false;
    bool placeHolder = false; 

    std::vector<Button> buttonList;

    Scan scan;
    Robot robot(center, heading, sf::Color::Red, velocity, maxRange,radius,scan);
    World world(size, border, sf::Color::White);    
    sf::RenderWindow window(sf::VideoMode((unsigned int)size.x, (unsigned int)size.y), "SLAM Sim!");
    window.setVerticalSyncEnabled(true);

    //Particle initialization
    int numParticles = 10;
    std::vector<Particle> initialParticles;
    for (int i = 0;i < numParticles;i++) {
        initialParticles.push_back(Particle(Eigen::Vector2f(200, 200), 45, sf::CircleShape(10, 3)));
    }

    FastSLAM fastSLAM(robot.radius, Eigen::Vector2f(.1, .1), Eigen::Vector2f(20, 15), 0.001,initialParticles);

    //Draw buttons and their text, this needs to be its own function at some point
    sf::Font font;
    font.loadFromFile("tahoma.ttf");

    bool *bools[3] = { &world.drawWorld, &robot.scan.doGaussian, &doLine };
    float sizes[3][2] = { {150.f,50.f},{150.f,50.f},{150.f,50.f} };
    float pos[3][2] = { {10.f,10.f},{200.f,10.f},{390.f,10.f} };
    std::string textStr[3] = { "Draw World","Gaussian", "Draw Lines" };

        
    sf::Color color = sf::Color::Red;
    for (int i = 0; i < 3; i++) {
        sf::Text text(textStr[i], font, 24);
        text.setFillColor(sf::Color::White);
        Button button(sizes[i][0], sizes[i][1], pos[i][0], pos[i][1], color, bools[i], text);
        buttonList.push_back(button);
    }
    
    Scan::Observation* obs;
    
    // Main loop
    while (window.isOpen())
    {
        Vector2 robot_center = Vector2(robot.center.x, robot.center.y);

        obs = robot.scan.performScan(robot_center, robot.radius, robot.maxRange, world);
        std::cout << robot.scan.cylinders.size() << "\n";
        //robot.checkBorderCol(world,robot.velocity.x,robot.heading );    
        
        sf::Event event;
        while (window.pollEvent(event))
        {
            switch (event.type) {
            case sf::Event::Closed:
                window.close();
                break;
                
            case sf::Event::MouseButtonPressed:
                if (event.mouseButton.button == sf::Mouse::Left){
                    float rad = 10;
                    if (!(event.mouseButton.x - rad < world.border.x || event.mouseButton.y - rad < world.border.y || event.mouseButton.x +rad > world.size.x - world.border.x || event.mouseButton.y + rad> world.size.y - world.border.y)) {
                        sf::CircleShape newCircle(rad);
                        newCircle.setPosition(event.mouseButton.x - rad, event.mouseButton.y - rad);
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
                robot.move(world, control);
               
            }
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
                control = { 4,6 };
                robot.move(world, control);
                
            }
            else {
                // w key is pressed: move our character
                control = { 5,5 };
                robot.move(world, control);
            }
            
            //fastSLAM.predict(Eigen::Vector2f(control.x, control.y));
            //fastSLAM.correct(robot.scan.cylinders);

        }
        else {
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::A)){
                control = { 1, -1 };
                robot.move(world, control);
            }
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
                control = { -1, 1 };
                robot.move(world, control);
            }

        }
        
        //===============Draw Section===============
        window.clear();


        if (world.drawWorld) {
            window.draw(world.borderRect);

            for (sf::CircleShape circle : world.circles) {
                window.draw(circle);
            }
        }

        /*
        for (int i = 0;i < robot.scan.cylinders.size();i++) {
            sf::CircleShape cs(7);
            cs.setPosition(robot.scan.cylinders[i](2) + 7, robot.scan.cylinders[i](3) + 7);
            window.draw(cs);
        }
        
        for (int i = 0;i < fastSLAM.particles.size();i++) {
            window.draw(fastSLAM.particles[i].marker);
        }
        */
        //Figure out a way to fix this ugly hardcoding
        
        for (int i = 0; i < buttonList.size();i++) {
            window.draw(buttonList[i].rect);
            window.draw(buttonList[i].text);
        }
    
        draw_obs(window, obs, robot_center, doLine);
        window.draw(robot.circle);
        window.draw(robot.dirLine);
        window.display();
    
        free(obs);
    }

    return 0;
}


