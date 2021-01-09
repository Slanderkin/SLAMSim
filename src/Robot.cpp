#include "Robot.h"

Robot::Robot(Vector2 centerIn, float headingIn, sf::Color colorIn, Vector2 velocityIn, float maxRangeIn, float radiusIn, World *wIn):
scan(),center(centerIn),heading(headingIn),color(colorIn),velocity(velocityIn),maxRange(maxRangeIn),radius(radiusIn),world(wIn),
drawViews(),drawRays(false),circle(10.f),dirLine(sf::Vector2f(20.f, 2.f)),obs()
{

	circle.setFillColor(color);
	circle.setPosition(this->center.x - radius, this->center.y - radius);
	dirLine.setPosition(this->center.x, this->center.y);
	dirLine.setRotation(heading);
	dirLine.setFillColor(color);
}

/*
TODO:
Make the turning more accurate to the two wheel model


Modles a two wheel robot seperated  by a width of 2*radius
The vector control consists of the distance travelled by each wheel (l,r)
*/
void Robot::move(Vector2 control) {
	float l = control.x;
	float r = control.y;
	float alpha = 0;
	float R = 0;
	float x1 = center.x;
	float y1 = center.y;

	if (r == -l) {
		if (r < l) {
			heading -= 2*(l-r)/this->radius;

		}
		else {
			heading += 2*(r-l)/this->radius;

		}
	}
	else if (r < l) {
		alpha = (l - r) / (radius*2);
		R = r / alpha;
		x1 = center.x + (R + radius) * (sin(heading*M_PI/180)-sin(heading * M_PI / 180-alpha));
		y1 = center.y + (R + radius) * (-cos(heading * M_PI / 180) + cos(heading * M_PI / 180 - alpha));
		heading = (fmod((heading * M_PI / 180 - alpha + M_PI), (2 * M_PI)) - M_PI)*180/M_PI;
	}
	else if (l < r) {
		alpha = (r - l) / (radius * 2);
		R = l / alpha;
		x1 = center.x + (R + radius) * (-sin(heading * M_PI / 180) + sin(heading * M_PI / 180 + alpha));
		y1 = center.y + (R + radius) * (cos(heading * M_PI / 180) - cos(heading * M_PI / 180 + alpha));
		heading = (fmod((heading * M_PI / 180 + alpha + M_PI), (2 * M_PI)) - M_PI)*180/M_PI;

	}
	else {
		x1 = center.x + l * cos(heading * M_PI / 180);
		y1 = center.y + l * sin(heading * M_PI / 180);
	}

	checkBorderCol(*(this->world), Vector2(x1, y1));
	checkCircleCol(*(this->world), Vector2(x1, y1));
	update();

}

/*
void Robot::forward(const World &world) {
	float vX = velocity.x * cos(heading * M_PI / 180);
	float vY = velocity.x * sin(heading * M_PI / 180);

	checkBorderCol(world, Vector2(vX, vY));
	checkCircleCol(world, Vector2(vX, vY));
	update();
	
}
*/

/*
TODO:
Update to accurately reflect the two wheel model given a control input
*/
void Robot::turn(bool isLeft) {
	if (isLeft){
		heading -= velocity.y;
	}
	else {
		heading += velocity.y;
	}
	update();
}

void Robot::update() {

	circle.setPosition(center.x - radius, center.y - radius);
	dirLine.setPosition(center.x, center.y);
	dirLine.setRotation(heading);

	this->obs = this->scan.performScan(this->center, this->radius, this->maxRange, *(this->world));
}

void Robot::checkBorderCol(const World &world, Vector2 newPos) {
	
	float circleRadius = radius; 
	Eigen::Vector2f circleCenter = {newPos.x,newPos.y};
	
	bool didMove = false;

	for(int i =1;i<world.worldVerticies.getVertexCount();i++){
		Eigen::Vector2f P1 = {world.worldVerticies[i-1].position.x,world.worldVerticies[i-1].position.y};  
		Eigen::Vector2f V(world.worldVerticies[i].position.x-P1[0],world.worldVerticies[i].position.y-P1[1]);
	

		float a = V.dot(V);
		float b = 2 * V.dot(P1 - circleCenter);
		float c = P1.dot(P1) + circleCenter.dot(circleCenter) - 2 * P1.dot(circleCenter) - circleRadius*circleRadius;
		float disc = b*b-4*a*c;
		
		if(disc>=0){
			float sqrt_disc = sqrt(disc);
			float t1 = (-b+sqrt_disc)/(2*a);
			float t2 = (-b-sqrt_disc)/(2*a);
			if((0<=t1 && t1 <=1 && 0<=t2 && t2<=1)){
				float t = std::max(0.0f,std::min(1.0f,-b/(2*a)));
				Eigen::Vector2f circToLine = ((P1+t*V)-circleCenter);
				float dist = circleRadius-circToLine.norm();
				float angle = atan2(circToLine[1],circToLine[0]);
				if(dist == 0){
					center = newPos;
				}
				else{
					circToLine.normalize();
					Eigen::Vector2f shift = dist*circToLine;
					center = {newPos.x - shift[0],newPos.y-shift[1]};
				}
				
				didMove = true;
			}
		}
		
	}
	if(!didMove){
		center = newPos;
	}

}

void Robot::checkCircleCol(const World &world, Vector2 newPos) {
	for (int i = 0; i < world.circles.size();i++) {

		float circleRad = world.circles[i].getRadius();
		sf::Vector2f position = world.circles[i].getPosition();
		float* collisionPoint = circlesCollided(newPos.x, newPos.y, position.x + circleRad, position.y + circleRad, radius, circleRad);
		center.x += collisionPoint[0];
		center.y += collisionPoint[1];
		collisionPoint[0] = 0.f;
		collisionPoint[1] = 0.f;
	}
	
}

float* Robot::circlesCollided(float c1x, float c1y, float c2x, float c2y, float c1rad, float c2rad) {
	static float result[2];
	float distanceBetweenCirclesSquared = (c2x - c1x) * (c2x - c1x) + (c2y - c1y) * (c2y - c1y);
	if (distanceBetweenCirclesSquared < (c1rad + c2rad) * (c1rad + c2rad)) {
		float angle = atan2f(c2y - c1y, c2x - c1x);
		float distBetweenCircles = sqrt(distanceBetweenCirclesSquared);
		float distToMove = c1rad + c2rad - distBetweenCircles;
		result[0] = -cos(angle) * distToMove;
		result[1] = -sin(angle) * distToMove;
	}
	return result;
}

void Robot::attachObs(Scan::Observation *o)
{
	this->obs = o;
}

// DRAW HANDLING
void Robot::addDrawView(DrawView *dv)
{
	this->drawViews.push_back(dv);
}

void Robot::draw()
{
	Timer timer("Robot draw");
	float circ_size = 2;
    sf::VertexArray line(sf::Lines, 2);
    sf::CircleShape end_circ(circ_size);
    sf::Vector2f circ_offset(circ_size, circ_size);
    end_circ.setFillColor(sf::Color::Green);
    sf::Vector2f end_pos;
	Vector2 robot_center = this->center;

	for (DrawView *dv : drawViews)
	{
		dv->window->setView(*(dv->view));
		for (int i = 0; i < this->obs->theta.size(); i++)
		{
			end_pos = sf::Vector2f(robot_center.x + this->obs->distance[i] * cos(this->obs->theta[i]), robot_center.y + this->obs->distance[i] * sin(obs->theta[i]));
			end_circ.setPosition(end_pos - circ_offset);
			dv->window->draw(end_circ);
			if(this->drawRays)
			{
				line[0].position = sf::Vector2f(robot_center.x, robot_center.y);
				line[1].position = end_pos;
				dv->window->draw(line);
			}
		}
		for (int i =0;i<this->scan.cylinders.size();i++){
			sf::CircleShape toDraw(7);
			toDraw.setPosition(sf::Vector2f(this->scan.cylinders[i](1,0)-7,this->scan.cylinders[i](1,1)-7));
			dv->window->draw(toDraw);
		}

		dv->window->draw(this->circle);
		dv->window->draw(this->dirLine);
	}
}