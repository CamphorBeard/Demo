#pragma once

#include <vector>
#include <Eigen>
#include "Triangle.hpp"
#include "Intersection.hpp"
#include "Ray.hpp"

using Eigen::Vector3f;

class Scene
{
public:
    unsigned screenWidth = 1280;
    unsigned screenHeight = 960;
    
    std::vector<MeshTriangle* > meshTris;  //save scene aka object in box

    float boxSize = 550.0;  //cornellBox boxSize * boxSize
    float ratioObjectBox = 0.7;  //object AABB longest length / boxSize
    float rotateAngle = 0.0;

    //cornellBox center at coordinates'origin,camera set on z aies look at -z
    double fov = 40.0 * M_PI / 180.0;
    float eyeToBoxFront = (boxSize / 2.0) / tan(fov / 2.0);  //box full of screen
    float eyeToScreen = (screenHeight / 2.0) / tan(fov / 2.0);
    Vector3f eyePosition = Vector3f(0, 0, boxSize / 2.0 + eyeToBoxFront);
    
    float RussianRoulette = 0.8f;

    Scene(int w, int h) : screenWidth(w), screenHeight(h){}

    void addLight(MeshTriangle& light);
    void addCornellBox(MeshTriangle& box);
    void addObjectInBox(MeshTriangle& object);

    void scale(MeshTriangle& meshTri, float nx, float ny, float nz);
    void rotate(MeshTriangle& meshTri, float angle);  //rotate around y axis
    void translate(MeshTriangle& meshTri, float tx, float ty, float tz);

    void viewTransform(MeshTriangle& meshTri);
    void projectTransform(MeshTriangle& meshTri);

    void sampleLight(Intersection& pos, float& pdf) const;
    Intersection getIntersection(const Ray& ray) const;
    Vector3f pathTracing(const Ray &ray) const;
};