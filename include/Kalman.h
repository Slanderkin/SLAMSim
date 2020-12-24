#pragma once
#include "StandardImports.h"

class Kalman
{
public:
    Eigen::Matrix<float, Dynamic, Dynamic> F;
    Eigen::Matrix<float, Dynamic, Dynamic> G;
    
    Kalman();
    void propagate();
    void update();


private:

};