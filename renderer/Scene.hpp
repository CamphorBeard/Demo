#pragma once

#include <vector>
#include <Eigen>
#include "Object.hpp"
#include "Intersection.hpp"
#include "Ray.hpp"
#include "Triangle.hpp"
//#include "BVH.hpp"

using Eigen::Vector3f;

struct boundingBox
{
    Vector3f pointMin{ INFINITY,INFINITY,INFINITY };
    Vector3f pointMax{ 0.0,0.0,0.0 };

    void updateValue()
    {
        pointMin = Vector3f(INFINITY, INFINITY, INFINITY);
        pointMax = Vector3f(0.0, 0.0, 0.0);
    }
};

class Scene
{
public:
    unsigned screenWidth = 1280;
    unsigned screenHeight = 960;
    double fov = 40.0 * M_PI / 180.0;
    Vector3f backgroundColor = Vector3f(0.235294f, 0.67451f, 0.843137f);

    std::vector<Object* > objects;
    //BVHAccel* bvh;

    float RussianRoulette = 0.8f;

    float boxSize = 550.0;  //cornellBox boxSize*boxSize
    float ratioObjectBox = 0.5;  //object boundingBox longest length/box length

    Scene(int w, int h) : screenWidth(w), screenHeight(h){}

    void Add(Object *object) { objects.push_back(object); }
    void addLight(MeshTriangle& light);
    void addCornellBox(MeshTriangle& box);
    void addObjectInBox(MeshTriangle& object, boundingBox& bbx);

    const std::vector<Object*>& get_objects() const { return objects; }

    //void buildBVH();

    //Intersection intersectBVH(const Ray& ray) const;

    void sampleLight(Intersection& pos, float& pdf) const;

    Intersection getIntersection(const Ray& ray) const;

    Vector3f castRay(const Ray &ray) const;
};