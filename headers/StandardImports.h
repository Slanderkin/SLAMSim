#ifndef STANDARDIMPORTS_H
#define STANDARDIMPORTS_H
#define _USE_MATH_DEFINES
#include <vector>
#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <random>
#include <string>
#include <array>


struct Vector2
{
    float x;
    float y;
    Vector2()
    {
        x = 0;
        y = 0;
    }
    Vector2(float xx, float yy)
    {
        x = xx;
        y = yy;
    }
};

Vector2 operator+(Vector2 a, Vector2 b);
Vector2 operator-(Vector2 a, Vector2 b);
Vector2 operator*(float c, Vector2 v);
Vector2 operator/(Vector2 v, float c);

#endif // !STANDARDIMPORTS_H