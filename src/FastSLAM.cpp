#include "FastSLAM.h"


FastSLAM::FastSLAM(float robotWidth, Eigen::Vector2f controlFactors, Eigen::Vector2f measurementStddev, float minimumLikelihood, std::vector<Particle> initialParticles) :
dist(mean, stddev),
generator(std::random_device{}())
 {
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


void FastSLAM::predict(const Eigen::Vector2f& control) {
	Timer timer("Predict");
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
std::vector<float> FastSLAM::updateComputeWeights(const std::vector<Eigen::Matrix2f>& cylinders) {
	Timer timer("update compute");
	Eigen::Matrix2f Qt_cov;
	Qt_cov << measurementStddev(0) * measurementStddev(0), 0,
		0, measurementStddev(1)* measurementStddev(1);
	std::vector<float> toRet(particles.size(),0.0f);
	float numLandmarks = 0;
	float weight;
	for (int i = 0; i < particles.size();i++) {
		particles[i].decrementVisibleLandmarkCounters();
		numLandmarks = particles[i].landMarkLocations.size();
		weight = 1;
		
		for (int j = 0;j < cylinders.size();j++) {
			
			Eigen::Matrix2f currCylinder = cylinders[j];
			weight *= particles[i].update_particle(numLandmarks, minimumLikelihood, Eigen::Vector2f(currCylinder(0,0),currCylinder(0,1)),Qt_cov);
		}
		toRet[i] = weight;
		particles[i].removeBadLandmarks();
		
		
	}
	return toRet;
}*/


std::vector<Particle> FastSLAM::resample(const std::vector<float>& weights) {
	std::vector<Particle> toRet;
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

void FastSLAM::correct(const std::vector<Eigen::Matrix2f>& cylinders) {
	Timer timer("correct");
	std::vector<float> weights = updateComputeWeights(cylinders);
	particles = resample(weights);
	
}

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




//Multithreading code

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


std::vector<float> FastSLAM::updateComputeWeights(const std::vector<Eigen::Matrix2f>& cylinders) {
	Timer timer("update compute");

	std::vector<std::future<std::vector<float>>> futures;

	Eigen::Matrix2f Qt_cov;
	Qt_cov << measurementStddev(0) * measurementStddev(0), 0,
		0, measurementStddev(1)* measurementStddev(1);
	std::vector<float> toRet(particles.size(),0.0f);
	float numLandmarks = 0;
	float weight;
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
