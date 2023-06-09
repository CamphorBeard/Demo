#pragma once

#include <vector>
#include <Eigen>
#include "Object.hpp"
#include "Intersection.hpp"
#include "Ray.hpp"
//#include "BVH.hpp"

using Eigen::Vector3f;

class Scene
{
public:
    unsigned width = 1280;
    unsigned height = 960;
    double fov = 40;
    Vector3f backgroundColor = Vector3f(0.235294f, 0.67451f, 0.843137f);
    int maxDepth = 1;

    std::vector<Object* > objects;
    //BVHAccel* bvh;

    float RussianRoulette = 0.8f;

    Scene(int w, int h) : width(w), height(h){}

    void Add(Object *object) { objects.push_back(object); }

    const std::vector<Object*>& get_objects() const { return objects; }

    //void buildBVH();

    //Intersection intersectBVH(const Ray& ray) const;

    void sampleLight(Intersection& pos, float& pdf) const;

    Intersection getIntersection(const Ray& ray) const;

    Vector3f castRay(const Ray &ray, int depth) const;
};