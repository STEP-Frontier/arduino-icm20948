#pragma once

struct Vector {
    float x;
    float y;
    float z;
    
    Vector() : x(0), y(0), z(0) {}
    Vector(float x, float y, float z) : x(x), y(y), z(z) {}
};
