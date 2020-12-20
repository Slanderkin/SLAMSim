#include "StandardImports.h"

#include "Robot.h"
#include "World.h"
#include "Scan.h"
#include "Button.h"

#include "Gui.h"





struct ObsDraw {
    std::array<sf::CircleShape, 360> endPoints;
    std::array<sf::VertexArray, 360> scanLines;
};

int main()
{
    float size[2] = { 1000,800 };
    float border[2] = { 150,150 };

    float center[2] = { size[0] / 2,size[1] / 2 };
    float heading = 0;
    float velocity[2] = {5,3};
    float maxRange = 1000;
    float radius = 10.f;
    bool doLine = false;
    bool placeHolder = false;

    std::vector<Button> buttonList;

    Scan scan;
    Robot robot(center, heading, sf::Color::Red, velocity, maxRange,radius,scan);
    World world(size, border, sf::Color::White);    
    sf::RenderWindow window(sf::VideoMode(size[0], size[1]), "SLAM Sim!");
    window.setFramerateLimit(60);

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
    ObsDraw* od = new ObsDraw;
    
    // Struct holding things to draw
    od->endPoints = std::array<sf::CircleShape, 360>();
    od->scanLines = std::array<sf::VertexArray, 360>();
    
    // Populate the struct with new objects
    float pointRad = 2;
    for (sf::CircleShape &cs : od->endPoints) {
        cs = sf::CircleShape(pointRad);
        cs.setFillColor(sf::Color::Green);
    }
    
    sf::VertexArray line(sf::Lines, 2);
    for (sf::VertexArray &va : od->scanLines) va = line; //Watch doing these kind of enhanced for loops with sf:: stuff, as long as its out of a loop its fine but otherwise it tanks perf

    // Main loop
    while (window.isOpen())
    {
        sf::Vector2f robot_center = sf::Vector2f(robot.center[0], robot.center[1]);

        obs = robot.scan.performScan(robot_center.x, robot_center.y, robot.radius, robot.maxRange, world);
        robot.checkBorderCol(world,robot.velocity[0],robot.heading );
        for (int i = 0; i < obs->theta.size(); i++)
        {
            sf::Vector2f end = sf::Vector2f(robot_center.x + obs->distance[i] * cos(obs->theta[i]) - pointRad, robot_center.y + obs->distance[i] * sin(obs->theta[i]) - pointRad);
            od->endPoints[i].setPosition(end);
            od->scanLines[i][0].position = robot_center;
            od->scanLines[i][1].position = end;
        }
        
        
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
                    
                    for (int i = 0; i < buttonList.size(); i++) {
                        if (buttonList[i].checkToggle(event.mouseButton.x, event.mouseButton.y)) {
                            buttonList[i].checkColor();
                            break;
                        }
                    }

                }
                break;
            
            case sf::Event::KeyPressed:
                if (event.key.code == sf::Keyboard::R) {
                    doLine = !doLine;
                    buttonList[2].checkColor();
                }
                else if (event.key.code == sf::Keyboard::G) {
                    robot.scan.doGaussian = !robot.scan.doGaussian;
                    buttonList[1].checkColor();

                }
                else if (event.key.code == sf::Keyboard::F) {
                    world.drawWorld = !world.drawWorld;
                    buttonList[0].checkColor();

                }
                
            default:
                break;
            }
                
        }


        
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
        {
            // w key is pressed: move our character
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
        for (sf::CircleShape cs : od->endPoints) {
            window.draw(cs);
        }
        if (doLine) {
            for (sf::VertexArray vertArr : od->scanLines) {
                window.draw(vertArr);
            }
        }
        for (int i = 0; i < buttonList.size();i++) {
            window.draw(buttonList[i].rect);
            window.draw(buttonList[i].text);
        }
       
        window.draw(robot.circle);
        window.draw(robot.dirLine);
        window.display();
    
        free(obs);
    }

    return 0;
}