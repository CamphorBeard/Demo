#pragma once

#include <cassert>
#include <array>
#include <random>
#include <Eigen>
#include "Object.hpp"
//#include "OBJ_Loader.hpp"  //error LNK2005 LNK1169
#include "Material.hpp"
#include "global.hpp"
//#include "BVH.hpp"

using Eigen::Vector3f;

class Triangle : public Object
{
public:
    Vector3f v0, v1, v2;  //triangle's vertices
    Vector3f e1, e2;      //2 edges v1-v0, v2-v0
    Vector3f normal;
    float area;
    Material* m;

    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material* _m = nullptr): v0(_v0), v1(_v1), v2(_v2), m(_m)
    {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = (e1.cross(e2)).normalized();  //whatif vector e1.cross(e2) inside obj
        area = (e1.cross(e2)).norm() * 0.5f;
    }

    bool intersectOrNot(const Ray& ray) override 
    { 
        bool rayIntersectTriangle = false;
        //ray intersects triangle's plane at time t
        float t = -((ray.origin - v0).dot(normal)) / ((ray.direction).dot(normal));
        if (t > 0)
        {
            Vector3f p = ray(t);
            if (isPInTriangle(p))
                rayIntersectTriangle = true;
        }
        return rayIntersectTriangle;
    }

    Intersection getIntersection(const Ray& ray) override
    {
        Intersection intersection;
        float t = -((ray.origin - v0).dot(normal)) / ((ray.direction).dot(normal));
        if (t > 0)
        {
            Vector3f p = ray(t);
            if (isPInTriangle(p))
            {
                intersection.happened = true;
                intersection.coords = p;
                intersection.normal = normal;
                intersection.emit = m->getEmission();
                intersection.distance = (p - ray.origin).norm();
                intersection.obj = this;
                intersection.m = m;
            }
        }
        return intersection;
    }

    //sample on object's area uniformly,using for sample light
    void Sample(Intersection &pos, float &pdf)
    {
        Vector3f barycentricCoord{ 0,0,0 };
        barycentricCoord.x() = get_random_float(1);
        barycentricCoord.y() = get_random_float(1 - barycentricCoord.x());
        barycentricCoord.z() = 1 - barycentricCoord.x() - barycentricCoord.y();
        pos.happened = true;
        pos.coords = barycentricCoord.x() * v0 + barycentricCoord.y() * v1 + barycentricCoord.z() * v2;
        pos.normal = normal;
        pos.emit = m->getEmission();
        pos.distance = std::numeric_limits<double>::max();
        pos.obj = this;
        pos.m = m;
        pdf = 1.0f / area;
    }

    //Bounds3 getBounds() override { return Union(Bounds3(v0, v1), v2); }
    bool hasEmit() { return m->hasEmission(); }
    float getArea() { return area; }

    bool isPInTriangle(const Vector3f& p)
    {
        bool PInTriangle = false;
        Vector3f a = (v1 - v0).cross(p - v0);
        Vector3f b = (v2 - v1).cross(p - v1);
        Vector3f c = (v0 - v2).cross(p - v2);
        float i = a.dot(normal);
        float j = b.dot(normal);
        float k = c.dot(normal);
        if ((i <= 0 && j <= 0 && k <= 0) || (i >= 0 && j >= 0 && k >= 0))
            PInTriangle = true;
        return PInTriangle;
    }
};

struct AxisAlignedBoundingBox
{
    Vector3f pointMin{ INFINITY,INFINITY,INFINITY };
    Vector3f pointMax{ 0.0,0.0,0.0 };

    void clearAABB()
    {
        pointMin = Vector3f{ INFINITY,INFINITY,INFINITY };
        pointMax = Vector3f{ 0.0,0.0,0.0 };
    }

    void updateAABB(const Vector3f& v)
    {
        pointMin = Vector3f(std::min(pointMin.x(), v.x()), std::min(pointMin.y(), v.y()), std::min(pointMin.z(), v.z()));
        pointMax = Vector3f(std::max(pointMax.x(), v.x()), std::max(pointMax.y(), v.y()), std::max(pointMax.z(), v.z()));
    }

    void updateAABB(const Triangle& tri)
    {
        float xMin = std::min(std::min(tri.v0.x(), tri.v1.x()), tri.v2.x());
        float yMin = std::min(std::min(tri.v0.y(), tri.v1.y()), tri.v2.y());
        float zMin = std::min(std::min(tri.v0.z(), tri.v1.z()), tri.v2.z());
        float xMax = std::max(std::max(tri.v0.x(), tri.v1.x()), tri.v2.x());
        float yMax = std::max(std::max(tri.v0.y(), tri.v1.y()), tri.v2.y());
        float zMax = std::max(std::max(tri.v0.z(), tri.v1.z()), tri.v2.z());

        pointMin = Vector3f(std::min(pointMin.x(), xMin), std::min(pointMin.y(), yMin), std::min(pointMin.z(), zMin));
        pointMax = Vector3f(std::max(pointMax.x(), xMax), std::max(pointMax.y(), yMax), std::max(pointMax.z(), zMax));
    }
};

class MeshTriangle : public Object
{
public:
    uint32_t numTriangles = 0;
    std::vector<Triangle> triangles;
    float area = 0;
    Material* m;
    AxisAlignedBoundingBox AABB;
    bool isObject = false;

    //Bounds3 bounding_box;
    //BVHAccel* bvh;

    MeshTriangle(){}

    MeshTriangle(const std::vector<Vector3f>& verts, const unsigned& numTris, Material* mt)
    {
        m = mt;
        numTriangles = numTris;

        for(int i=0;i<numTris*3;i=i+3)
        {
            triangles.emplace_back(Triangle(verts[i], verts[i + 1], verts[i + 2], mt));
            AABB.updateAABB(Triangle(verts[i], verts[i + 1], verts[i + 2], mt));
        }
        
        for (auto& tri : triangles)
        {
            area += tri.area;
        }
    }

    bool intersectOrNot(const Ray& ray) override
    {
        //if (bounding_box.intersectOrNot(ray))  //ray intersect mesh triangle's bounding-box
        {
            for (unsigned k = 0; k < triangles.size(); k++)
            {
                if (triangles[k].intersectOrNot(ray))
                    return true;
            }
        }
        return false;
    }

    Intersection getIntersection(const Ray& ray) override
    {
        Intersection intersection;
        //if (bounding_box.intersectOrNot(ray))  //ray intersect mesh triangle's bounding-box
        {
            double distance = INFINITY;
            for (unsigned k = 0; k < triangles.size(); k++)
            {
                Intersection intersectionTemp = triangles[k].getIntersection(ray);  //ray intersect triangle
                if (intersectionTemp.happened==true&&intersectionTemp.distance < distance)  //intersection's distance get closer
                {
                    intersection = intersectionTemp;
                    distance = intersection.distance;
                    intersection.obj = this;
                }
            }
        }
        return intersection;
    }

    //sample on object's area uniformly,using for sample light
    void Sample(Intersection &pos, float &pdf)
    {
        //bvh->Sample(pos, pdf);
        //pos.emit = m->getEmission();

        //get a random triangle index,sample on this triangle
        unsigned index = get_random_float(numTriangles);
        triangles[index].Sample(pos, pdf);
        pos.obj = this;
        pdf = 1.0f / area;
    }

    //Bounds3 getBounds() { return Bounds3(); }
    bool hasEmit() { return m->hasEmission(); }
    float getArea() { return area; }
};