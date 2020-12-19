#include "Robot.h"
#include "World.h"
#include "Scan.h"
#include "StandardImports.h"

int main()
{
    float size[2] = { 800,600 };
    float border[2] = { 50,50 };

    float center[2] = { size[0] / 2,size[1] / 2 };
    float heading = 0;
    float velocity[2] = {5,3};
    float maxRange = 1000;
    float radius = 10.f;
    bool doLine = false;

    Scan scan;
    Robot robot(center, heading, sf::Color::Red, velocity, maxRange,radius,scan);
    World world(size, border, sf::Color::White);    
    sf::RenderWindow window(sf::VideoMode(size[0], size[1]), "SLAM Sim!");
    window.setFramerateLimit(60);

    while (window.isOpen())
    {
        robot.scan.performScan(robot.center[0],robot.center[1],robot.radius,robot.maxRange,world);
        robot.checkBorderCol(world,robot.velocity[0],robot.heading );
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
                    if (!(event.mouseButton.x - rad < world.border[0] || event.mouseButton.y - rad < world.border[1] || event.mouseButton.x +rad > world.size[0] - world.border[0] || event.mouseButton.y + rad> world.size[1] - world.border[1])) {
                        sf::CircleShape newCircle(rad);
                        newCircle.setPosition(event.mouseButton.x - rad, event.mouseButton.y - rad);
                        world.addCircle(newCircle);
                    }
                    
                    

                }
                break;
            
            case sf::Event::KeyPressed:
                if (event.key.code == sf::Keyboard::R) {
                    doLine = !doLine;
                }
                else if (event.key.code == sf::Keyboard::G) {
                    robot.scan.doGaussian = !robot.scan.doGaussian;

                }
                else if (event.key.code == sf::Keyboard::F) {
                    world.drawWorld = !world.drawWorld;

                }
                
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


        if (world.drawWorld) {
            window.draw(world.borderRect);

            for (sf::CircleShape circle : world.circles) {
                window.draw(circle);
            }
        }

        

        //Figure out a way to fix this ugly hardcoding
        for (int i = 0; i < 360; i++) {
            window.draw(robot.scan.endCircles[i]);
        }
        if (doLine) {
            for (sf::VertexArray vertArr : robot.scan.scanLines) {
                window.draw(vertArr);
            }
        }
        
        window.draw(robot.circle);
        window.draw(robot.dirLine);
        window.display();
    }

    return 0;
}