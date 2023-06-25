#pragma once
#ifndef RAYTRACING_OBJECT_H
#define RAYTRACING_OBJECT_H

#include "Ray.hpp"
#include "Intersection.hpp"
#include "Bounds3.hpp"

class Object
{
public:
    Object() {}
    virtual ~Object() {}

    virtual bool intersectOrNot(const Ray& ray) = 0;
    virtual Intersection getIntersection(const Ray& ray) = 0;

    virtual void Sample(Intersection& pos, float& pdf) = 0;

    virtual Bounds3 getBounds()=0;
    virtual bool hasEmit() = 0;
    virtual float getArea()=0;
};

#endif //RAYTRACING_OBJECT_H
