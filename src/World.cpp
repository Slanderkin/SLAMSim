#include "World.h"
#include "StandardImports.h"


/*
TODO:
Find a better way to implement file name

*/
World::World(Vector2 sizeIn, Vector2 borderIn, sf::Color colorIn):
	drawWorld(false),size(sizeIn),border(borderIn),color(colorIn),worldVerticies(),circles(),drawViews(),fileName("test.csv")
{
	this->fileRows = getFileSize(fileName);
	this->worldVerticies.resize(fileRows);
	this->worldVerticies.setPrimitiveType(sf::LineStrip);
	loadWorldVerticies(fileRows);
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
		
		if(this->drawWorld == true)
		{
			dv->window->draw(worldVerticies);
			for (sf::CircleShape circle : this->circles) {
				dv->window->draw(circle);
			}
		}
	}

}