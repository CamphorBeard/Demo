#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <vector>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"

struct BVHBuildNode
{
    Bounds3 bounds;
    BVHBuildNode* left;
    BVHBuildNode* right;
    Object* object;

    BVHBuildNode()
    {
        bounds = Bounds3();
        left = nullptr;
        right = nullptr;
        object = nullptr;
    }
};

class BVHAccel 
{
public:
    BVHAccel(std::vector<Object*> p);
    ~BVHAccel();
    
    Intersection Intersect(const Ray &ray) const;
    
private:
    std::vector<Object*> primitives;
    BVHBuildNode* root;

    BVHBuildNode* recursiveBuild(std::vector<Object*>objects);
    Intersection getBVHIntersection(BVHBuildNode* node, const Ray& ray)const;
};
#endif //RAYTRACING_BVH_H
