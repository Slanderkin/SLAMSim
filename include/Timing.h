#pragma once
#ifndef TIMING_H
#define TIMING_H
#include "StandardImports.h"

class Timer{

public:
    std::chrono::high_resolution_clock::time_point start,end;
    std::chrono::duration<float> duration;
    std::string name;
    Timer(std::string timerName){
        start = std::chrono::high_resolution_clock::now();
        name = timerName;
    }

    ~Timer(){
        end = std::chrono::high_resolution_clock::now();
        duration = (end-start);

        float ms = duration.count() * 1000.0f;
        std::cout << name << " took " << ms << "ms " << std::endl;
    }

};


#endif