#pragma once
#ifndef FASTSLAM_H
#define FASTSLAM_H
#include "StandardImports.h"
#include "Particle.h"
class FastSLAM {


public:
	FastSLAM(float robotWidth, Eigen::Vector2f controlFactors, Eigen::Vector2f measurementStddev, float minimumLikelihood, std::vector<Particle> initialParticles);

	//==========Various Public Members==========//
	float robotWidth;
	Eigen::Vector2f controlFactors; //Motion,Turn (Models innaccuracies in real world movement and turning)
	Eigen::Vector2f measurementStddev; //Dist,angle
	float minimumLikelihood;
	std::vector<Particle> particles;
	

	//==========Draw Related Setup==========//
	void addDrawView(DrawView *dv);
	void draw();
	sf::CircleShape circle;
	sf::RenderWindow *window;
	
	sf::CircleShape errorEllipse;
	sf::RectangleShape dirLine;
	std::vector<DrawView*> drawViews;

	//==========Computational Functions==========//
	void predict(const Eigen::Vector2f& control);
	std::vector<float> updateComputeWeights(const std::vector<Eigen::Matrix2f>& cylinders);
	std::vector<Particle> resample(const std::vector<float>& weights);
	void correct(const std::vector<Eigen::Matrix2f>& cylinders);
	Eigen::Vector3f getMean(const std::vector<Particle>& particles);
	Eigen::Vector4f ellipseVar(const std::vector<Particle>& particles,const Eigen::Vector3f& mean);
	int getIndOfMin(const std::vector<Particle>& particles,const Eigen::Vector3f& mean);

};




#endif // !FASTSLAM_H
