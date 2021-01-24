#pragma once
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "StandardImports.h"
#include "World.h"

class PathPlanner{

private:

    bool startSet;
    bool endSet;
    bool circleCol(float cx, float cy, int x, int y,float rad);
    //A* Mat

public:

    Eigen::Vector2i startPoint;
    Eigen::Vector2i endPoint;

    PathPlanner();
    PathPlanner(Eigen::Vector2i startPointIn,Eigen::Vector2i endPointIn);

    bool isReady();

    void calculatePath();

    void rasterWorld(World& world);

};

#endif