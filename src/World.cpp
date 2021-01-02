#include "World.h"
#include "StandardImports.h"

World::World(){}

World::World(Vector2 size, Vector2 border, sf::Color color){
	this->size = size;
	this->border = border;
	this->color = color;
	this->edges = { {border.x, border.y, size.x - border.x,border.y},{ border.x, border.y, border.x,size.x -  border.y},{size.x - border.x, border.y, size.x -  border.x,size.y - border.y },{ border.x, size.y -  border.y, size.x - border.x,size.y -  border.y } };
	this->drawWorld = false;
	this->worldVerticies = sf::VertexArray();
	this->drawViews = std::vector<DrawView*>();

	circles = std::vector<sf::CircleShape>(); //TopRight,TopLeft,BotLeft,BotRight

	borderRect = sf::RectangleShape(sf::Vector2f(this->size.x - 2 * this->border.x, this->size.y - 2 * this->border.y));
	borderRect.setPosition(sf::Vector2f(this->border.x, this->border.y));
	borderRect.setOutlineThickness(5);
	borderRect.setOutlineColor(sf::Color::Magenta);
	borderRect.setFillColor(sf::Color::Black);

	
	this->fileName = "test.csv";
	this->fileRows = getFileSize(fileName);
	this->worldVerticies.resize(fileRows);
	this->worldVerticies.setPrimitiveType(sf::LineStrip);
	std::cout<< fileRows << std::endl;
	std::cout<< this->worldVerticies.getVertexCount() << std::endl;
	loadWorldVerticies(fileRows);
	std::cout<< worldVerticies[0].position.x << std::endl;
}

void World::addCircle(sf::CircleShape circle) {
	circle.setFillColor(sf::Color::Magenta);

	this->circles.push_back(circle);

}

int World::getFileSize(std::string fileName){
	std::ifstream fin(fileName);
	if(fin.fail()){
		std::cerr << "Error: " << strerror(errno);
	}
	int numRows = 0;
	std::string line;
	while(std::getline(fin,line)){
		numRows++;
	}  
	fin.close();
	return numRows;
}

void World::loadWorldVerticies(int numRows){

	std::ifstream fin(fileName);
	if(fin.fail()){
		std::cerr << "Error: " << strerror(errno);
	}


	int i = 0;
	int j = 0;
	float xPos,yPos;
	std::string line,word;
	std::array<std::string,2> row;
	while(j < numRows){
		row[0] = "";
		row[1] = "";

		std::getline(fin,line);
		std::cout << line << std::endl;
		std::stringstream s(line);

		for(int k=0;k<2;k++){
			getline(s,word,',');
			row[k] = word;
		}

		xPos = std::stof(row[0]);
		yPos = std::stof(row[1]);
		worldVerticies[j].position.y = yPos;
		worldVerticies[j].position.x = xPos;
		j++;
	}
	
}

void World::addDrawView(DrawView *dv)
{
	this->drawViews.push_back(dv);
}

void World::draw()
{
	for (DrawView *dv : drawViews)
	{
		dv->window->setView(*(dv->view));
		dv->window->draw(worldVerticies);
		if(this->drawWorld == true)
		{
			dv->window->draw(this->borderRect);

			for (sf::CircleShape circle : this->circles) {
				dv->window->draw(circle);
			}
		}
	}

}