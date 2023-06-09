#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H

#include "Object.hpp"
#include "Material.hpp"
class Object;

struct Intersection
{
    bool happened;
    Vector3f coords;
    Vector3f normal;
    Vector3f emit;
    double distance;
    Object* obj;
    Material* m;

    Intersection()
    {
        happened = false;
        coords= Vector3f(0.0f, 0.0f, 0.0f);
        normal= Vector3f(0.0f, 0.0f, 0.0f);
        emit= Vector3f(0.0f, 0.0f, 0.0f);
        distance= std::numeric_limits<double>::max();
        obj = nullptr;
        m = nullptr;
    }
};
#endif //RAYTRACING_INTERSECTION_H
