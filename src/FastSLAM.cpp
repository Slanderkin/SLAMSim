#include "FastSLAM.h"

//
FastSLAM::FastSLAM(float robotWidth, Eigen::Vector2f controlFactors, Eigen::Vector2f measurementStddev, float minimumLikelihood, std::vector<Particle> initialParticles) {
	this->robotWidth = robotWidth;
	this->controlFactors = controlFactors;
	this->measurementStddev = measurementStddev;
	this->minimumLikelihood = minimumLikelihood;
	this->particles = initialParticles;
	this->circle = sf::CircleShape(8);
	this->circle.setPosition(-20,-20);
	this->circle.setOrigin(8,8);
	this->circle.setFillColor(sf::Color::Yellow);
	this->dirLine = sf::RectangleShape(sf::Vector2f(20.f, 2.f));
	this->dirLine.setFillColor(sf::Color::Yellow);
	this->errorEllipse = sf::CircleShape(8);
	this->errorEllipse.setFillColor(sf::Color::Transparent);
	this->errorEllipse.setOutlineThickness(2);


}

/*
Summary:
	Predicts the future location of the particle given a control input. This function
	adds random gaussian noise to the control input to simulate possible drifts in 
	the robot's position.
Params:
	control - first index holds the left control, while the second holds the right
Returns:
	None
*/
void FastSLAM::predict(const Eigen::Vector2f& control) {
	float l0 = control(0);
	float r0 = control(1);
	float lStd = sqrt((controlFactors(0) * l0) * (controlFactors(0) * l0) + (controlFactors(1) * (l0 - r0)) * (controlFactors(1) * (l0 - r0)));
	float rStd = sqrt((controlFactors(0) * r0) * (controlFactors(0) * r0) + (controlFactors(1) * (l0 - r0)) * (controlFactors(1) * (l0 - r0)));
	std::mt19937 generatorL(std::random_device{}());
	std::mt19937 generatorR(std::random_device{}());
	std::normal_distribution<double> distl(l0,lStd);
	std::normal_distribution<double> distr(r0, rStd);
	float l=0;
	float r=0;

	for (int i = 0;i < particles.size();i++) {
		l=l0;
		r=r0;
		if(l == -r){
			float delta = distl(generatorL);
			l = delta;
			r = -delta;
		}
		else{
		l = distl(generatorL);
		r = distr(generatorR);
		}
		particles[i].move(Eigen::Vector2f(l, r));
	}
}

/*
Summary:
	Samples all of the particles currently active in a psuedo-random way,
	highly weighted particles (those that more accurately match the measurements)
	are more likely to survive, but none are guaranteed. This sampling process makes
	up the next particles that will be held.
Params:
	weights - a list of weights corresponding to each particle
Returns:
	A vector of Particles, these are the particles that "survive" the resampling.
*/
std::vector<Particle> FastSLAM::resample(const std::vector<float>& weights) {
	std::vector<Particle> toRet;
	toRet.reserve(particles.size());
	float maxWeight = -1;


	for(int i =0; i < weights.size(); i++){
		if(weights[i] > maxWeight){
			maxWeight = weights[i];
		}
	}
	int index = rand() % particles.size();
	float offset = 0;


	for (int i = 0;i < particles.size();i++) {
		offset += (float(rand()) / float((RAND_MAX)) * 2 * maxWeight);
		while (offset > weights[index]) {
			
			offset -= weights[index];
			index = (index + 1) % weights.size();

		}

		toRet.push_back(particles[index]);
		
	}

	return toRet;
}

/*
Summary:
	Finds the weights of all particles, then resamples them
Params:
	cylinders - A list of all clinders found via the actual robot's scan
Returns:
	None
*/
void FastSLAM::correct(const std::vector<Eigen::Matrix2f>& cylinders) {
	std::vector<float> weights = updateComputeWeights(cylinders);
	particles = resample(weights);
	
}

/*
Summary:
	Gets the mean position and heading of the Particles, used to represent the
	particles' guess at the robot's position and heading
Params:
	A reference vector of all of the particles
Returns:
	A 3 item vector containing the mean X,mean Y, and mean heading (0 if there are no particles)
*/
Eigen::Vector3f FastSLAM::getMean(const std::vector<Particle>& particles){
	float meanx =0;	float meany =0;
	float headx =0; float heady =0;
	float n = particles.size();
	for(int i=0; i<n; i++){
		meanx+= particles[i].position[0];
		meany+= particles[i].position[1];
		headx+= cos(particles[i].heading*M_PI/180);
		heady+= sin(particles[i].heading*M_PI/180);
	}
	if(n >0){
		return Eigen::Vector3f(meanx/n,meany/n,atan2(heady,headx));
	}
	else{
		return Eigen::Vector3f(0,0,0);
	}
}

/*
TODO: Error Arc for heading err
Summary:
	Computes the scale for the major and main axes of the error ellipse of the particle projection
	as well as the angle for the error ellipse.
Params:
	particles - The list of particles
	mean - The mean position and heading of the particles
Returns:
	A 3 item vector containing the mean X,mean Y, and mean heading (0 if there are no particles)
*/
Eigen::Vector4f FastSLAM::ellipseVar(const std::vector<Particle>& particles,const Eigen::Vector3f& mean){
	float n = particles.size();
	if (n < 2){
		return Eigen::Vector4f(0,0,0,0);
	}
	float cx = mean[0];float cy=mean[1];float ch = mean[2];
	float sxx=0;float sxy = 0;float syy = 0;
	float varHeading = 0; float dh =0;
	for(int i=0;i<n;i++){
		float dx = particles[i].position[0]-cx;
		float dy = particles[i].position[1]-cy;
		sxx += dx*dx;
		sxy += dx*dy;
		syy += dy*dy;
		dh = fmod(((particles[i].heading -ch)*(M_PI/180) + M_PI ),2*M_PI)-M_PI;
		varHeading += dh*dh;
	}
	varHeading = varHeading/(n-1);
	Eigen::Matrix2f covXY;
	covXY << sxx,sxy,
		sxy,syy;
	covXY = covXY/(n-1);

	Eigen::EigenSolver<Eigen::Matrix2f> es;
	es.compute(covXY,true);
	float ellipseAngle =atan2(es.eigenvectors().col(0)[1].real(),es.eigenvectors().col(0)[0].real());
	return Eigen::Vector4f(ellipseAngle,sqrt(abs(es.eigenvalues()[0].real())),sqrt(abs(es.eigenvalues()[1].real())),sqrt(varHeading) );
}

void FastSLAM::addDrawView(DrawView *dv){
	this->drawViews.push_back(dv);
}

void FastSLAM::draw(){
	Timer timer("FastSLAM draw");
	Eigen::Vector3f mean = getMean(particles);
	circle.setPosition(mean[0],mean[1]);
	circle.setRotation(mean[2]*180/M_PI);
	dirLine.setPosition(mean[0],mean[1]);
	dirLine.setRotation(mean[2]*180/M_PI);
	Eigen::Vector4f ellipseMisc = ellipseVar(particles,mean);
	errorEllipse.setOrigin(errorEllipse.getRadius(),errorEllipse.getRadius());
	errorEllipse.setScale(ellipseMisc[1],ellipseMisc[2]);
	errorEllipse.setPosition(mean[0],mean[1]);
	errorEllipse.setRotation(ellipseMisc[0]*180/M_PI);

	int index = getIndOfMin(particles,mean);


	for (DrawView *dv: drawViews){
		dv->window->setView(*(dv->view));
		dv->window->draw(circle);
		dv->window->draw(dirLine);
		dv->window->draw(errorEllipse);
		for(int i =0;i<particles.size();i++){
			dv->window->draw(particles[i].marker);
		}
		if(index >=0){
			for(int i =0;i<particles[index].landMarkLocations.size();i++){
			sf::CircleShape circ(10);
			circ.setFillColor(sf::Color::Yellow);
			circ.setOrigin(10,10);
			circ.setPosition(particles[index].landMarkLocations[i][0],particles[index].landMarkLocations[i][1]);
			dv->window->draw(circ);
			if(particles[index].landMarkCov[i](0,0) < 500/10 && particles[index].landMarkCov[i](1,1) < 500/10){
				sf::CircleShape errEllipse(10);
				errEllipse.setFillColor(sf::Color::Transparent);
				errEllipse.setOutlineThickness(1);
				errEllipse.setOrigin(10,10);
				errEllipse.setPosition(particles[index].landMarkLocations[i][0],particles[index].landMarkLocations[i][1]);
				errEllipse.setScale(particles[index].landMarkCov[i](0,0),particles[index].landMarkCov[i](1,1));
				dv->window->draw(errEllipse);
			}
			
			}
		}
	}
}

/*
Summary:
	Gets the index of the particle that is closest to the mean
Params:
	particles - The list of particles
	mean - The mean position and heading of the particles
Returns:
	the index of the particle clostest to the mean
*/
int FastSLAM::getIndOfMin(const std::vector<Particle>& particles,const Eigen::Vector3f& mean){
	int index = -1;
	Eigen::Vector2f min;
	float minNorm;
	for(int i =0;i<particles.size();i++){
		if(i ==0){
			min = particles[i].position;
			index = 0;
			minNorm = particles[i].position.norm();
			
		}
		else{
			float newNorm = (Eigen::Vector2f(mean[0],mean[1])-particles[i].position).norm();
			if (newNorm < minNorm){
				minNorm = newNorm;
				index = i;
				min = particles[i].position;
			}
			
		}
	}
	return index;
}




//===========Multithreading code===========//
/*
Summary:
	This is the function run asynchronously, its manages updating the weight of an individual particle
Params:
	index - this particle's index
	minimumLikelihood - the minimum acceptable weighted level of accuracy in the measurement
	particle - The particle to be updated
	cylinders - A list of all clinders found via the actual robot's scan
	Qt_cov - the covariance matrix
Returns:
	The weight of the particle and it's index in a 2 long vector
*/
static std::vector<float> handleParticle(int index,float minimumLikelihood, Particle* particle,const std::vector<Eigen::Matrix2f>* cylinders, const Eigen::Matrix2f Qt_cov){

	particle->decrementVisibleLandmarkCounters();
	float numLandmarks = particle->landMarkLocations.size();
	float weight = 1;
	std::vector<float> toRet = {0,0};
	for (int j = 0;j < cylinders->size();j++) {
			
			Eigen::Matrix2f currCylinder = cylinders->at(j);
			weight *= particle->update_particle(numLandmarks, minimumLikelihood, Eigen::Vector2f(currCylinder(0,0),currCylinder(0,1)),Qt_cov);
		}

	particle->removeBadLandmarks();
	toRet[0] = (weight);
	toRet[1] = (index);
	return toRet;
}

/*
Summary:
	This loops through all particles and asynchronously updates their weights
Params:
	cylinders - A list of all clinders found via the actual robot's scan
Returns:
	A vector of the updated weights for each particle
*/
std::vector<float> FastSLAM::updateComputeWeights(const std::vector<Eigen::Matrix2f>& cylinders) {

	std::vector<std::future<std::vector<float>>> futures;
	futures.reserve(particles.size());

	Eigen::Matrix2f Qt_cov;
	Qt_cov << measurementStddev(0) * measurementStddev(0), 0,
		0, measurementStddev(1)* measurementStddev(1);
	std::vector<float> toRet(particles.size(),0.0f);
	float numLandmarks = 0;
	int i=0;

	for (int i = 0; i < particles.size();i++) {
		futures.push_back(std::async(std::launch::async, handleParticle,i,minimumLikelihood, &particles[i],&cylinders,Qt_cov));
	}
	
	for(int i=0; i<particles.size();i++){
		std::vector<float> tempVec = futures[i].get();
		toRet[tempVec[1]] = tempVec[0];
	}

	return toRet;

}
