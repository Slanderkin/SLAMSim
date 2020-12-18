#include "Robot.h"
#include "World.h"
#include "Scan.h"
#include "StandardImports.h"

int main()
{
    float size[2] = { 700,600 };
    float border[2] = { 50,50 };

    float center[2] = { size[0] / 2,size[1] / 2 };
    float heading = 0;
    float velocity[2] = {5,3};
    float maxRange = 1000;
    float radius = 10.f;

    Scan scan;
    Robot robot(center, heading, sf::Color::Red, velocity, maxRange,radius,scan);
    World world(size, border, sf::Color::White);    
    sf::RenderWindow window(sf::VideoMode(size[0], size[1]), "SFML works!");
    window.setFramerateLimit(60);

    while (window.isOpen())
    {
        robot.scan.performScan(robot.center[0],robot.center[1],robot.radius,robot.maxRange,world);
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
                    sf::CircleShape newCircle(rad);
                    newCircle.setPosition(event.mouseButton.x-rad, event.mouseButton.y-rad);
                    world.addCircle(newCircle);

                }
                break;
            

                
            default:
                break;
            }
                
        }


        
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
        {
            // left key is pressed: move our character
            robot.forward(world);
        }
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::A)){
            robot.turn(true);
        }
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
            robot.turn(false);
        }


        //===============Draw Section===============
        window.clear();
        window.draw(world.borderRect);
        
        for (sf::CircleShape circle : world.circles) {
            window.draw(circle);
        }

        for (sf::CircleShape circle : robot.scan.endCircles) {
            window.draw(circle);
        }
        for (sf::VertexArray vertArr : robot.scan.scanLines) {
            window.draw(vertArr);
        }
        window.draw(robot.circle);
        window.draw(robot.dirLine);
        window.display();
    }

    return 0;
}