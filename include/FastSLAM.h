#pragma once
#ifndef FASTSLAM_H
#define FASTSLAM_H
#include "StandardImports.h"
#include "Particle.h"
class FastSLAM {


public:
	FastSLAM(float robotWidth, Eigen::Vector2f controlFactors, Eigen::Vector2f measurementStddev, float minimumLikelihood, std::vector<Particle> initialParticles);
	float robotWidth;
	Eigen::Vector2f controlFactors; //Motion,Turn
	Eigen::Vector2f measurementStddev; //Dist,angle
	float minimumLikelihood;
	std::vector<Particle> particles;
	sf::CircleShape circle;
	sf::CircleShape errorEllipse;
	sf::RectangleShape dirLine;

	double mean = 0.0;
	double stddev = 0.001;
	std::mt19937 generator;
	std::normal_distribution<double> dist;

	void addDrawView(DrawView *dv);
	void draw();
	sf::RenderWindow *window;
	std::vector<DrawView*> drawViews;

	void predict(Eigen::Vector2f control);
	std::vector<float> updateComputeWeights(std::vector<Eigen::Matrix2f> cylinders);
	std::vector<Particle> resample(std::vector<float> weights);
	void correct(std::vector<Eigen::Matrix2f> cylinders);
	Eigen::Vector3f getMean(std::vector<Particle> particles);
	Eigen::Vector4f ellipseVar(std::vector<Particle> particles, Eigen::Vector3f mean);


};




#endif // !FASTSLAM_H
