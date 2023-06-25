#pragma once

#include <array>
#include <Eigen>
#include "Object.hpp"
#include "Material.hpp"
#include "global.hpp"
#include "Bounds3.hpp"
#include "BVH.hpp"

using Eigen::Vector3f;

class Triangle : public Object
{
public:
    Vector3f v0, v1, v2;
    Vector3f normal;
    Material* m;
    float area;

    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material* material = nullptr): v0(_v0), v1(_v1), v2(_v2), m(material)
    {
        Vector3f edgesCrossProduct = (v1 - v0).cross(v2 - v0);
        normal = edgesCrossProduct.normalized();
        area = edgesCrossProduct.norm() * 0.5f;
    }

    bool intersectOrNot(const Ray& ray) override 
    { 
        float t = -((ray.origin - v0).dot(normal)) / ((ray.direction).dot(normal));
        if (t > 0)
        {
            Vector3f p = ray.origin + t * ray.direction;
            if (isPInTriangle(p))
                return true;
        }
        return false;
    }

    Intersection getIntersection(const Ray& ray) override
    {
        Intersection intersection;
        float t = -((ray.origin - v0).dot(normal)) / ((ray.direction).dot(normal));
        if (t > 0)
        {
            Vector3f p = ray.origin + t * ray.direction;
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
        pos.obj = this;
        pos.m = m;
        pdf = 1.0f / area;
    }

    bool isPInTriangle(const Vector3f& p)
    {
        Vector3f a = (v1 - v0).cross(p - v0);
        Vector3f b = (v2 - v1).cross(p - v1);
        Vector3f c = (v0 - v2).cross(p - v2);
        float i = a.dot(normal);
        float j = b.dot(normal);
        float k = c.dot(normal);
        return ((i <= 0 && j <= 0 && k <= 0) || (i >= 0 && j >= 0 && k >= 0));
    }

    Bounds3 getBounds() override { return Union(Bounds3(v0, v1), v2); }
    bool hasEmit() { return m->hasEmission(); }
    float getArea() { return area; }
};

class MeshTriangle : public Object
{
public:
    uint32_t numTriangles = 0;
    std::vector<Triangle> triangles;
    Material* m;
    float area = 0;
    bool isCornellBox = true;

    Bounds3 AABB;  //axis aligned bounding box
    BVHAccel* BVH;  //bounding volume hierarchy

    MeshTriangle(){}

    MeshTriangle(const std::vector<Vector3f>& verts, const unsigned& numTris, Material* material)
    {
        m = material;
        numTriangles = numTris;

        for (int i = 0; i < numTris * 3; i = i + 3)
        {
            Triangle triTemp{ verts[i], verts[i + 1], verts[i + 2], material };
            triangles.emplace_back(triTemp);
            area = area + triTemp.area;
        }
    }

    void buildBVH()
    {
        if (!isCornellBox)
        {
            AABB.clear();
            std::vector<Object*> ptrs;
            for (auto& tri : triangles)
            {
                AABB.update(tri.v0);
                AABB.update(tri.v1);
                AABB.update(tri.v2);
                ptrs.push_back(&tri);
            }
            BVH = new BVHAccel(ptrs);
        }
    }

    bool intersectOrNot(const Ray& ray) override
    {
        if (!isCornellBox)
        {
            Vector3f invDir{ 1.0f / ray.direction.x(),1.0f / ray.direction.y(), 1.0f / ray.direction.z() };
            std::array<int, 3> dirIsNeg{ ray.direction.x() > 0,ray.direction.y() > 0 ,ray.direction.z() > 0 };
            if (AABB.intersectOrNot(ray, invDir, dirIsNeg))
                for (unsigned k = 0; k < triangles.size(); k++)
                {
                    if (triangles[k].intersectOrNot(ray))
                        return true;
                }
        }
        else
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
        if (!isCornellBox)
        {
            intersection = BVH->Intersect(ray);
            if (intersection.happened == true)
                intersection.obj = this;
        }
        else
        {
            double distance = INFINITY;
            for (unsigned k = 0; k < triangles.size(); k++)
            {
                Intersection intersectionTemp = triangles[k].getIntersection(ray);
                if (intersectionTemp.happened == true && intersectionTemp.distance < distance)
                {
                    intersection = intersectionTemp;
                    distance = intersection.distance;
                    intersection.obj = this;
                }
            }
        }
        return intersection;
    }

    void Sample(Intersection &pos, float &pdf)
    {
        unsigned index = get_random_float(numTriangles);
        triangles[index].Sample(pos, pdf);
        pos.obj = this;
        pdf = 1.0f / area;
    }

    Bounds3 getBounds() { return Bounds3(); }
    bool hasEmit() { return m->hasEmission(); }
    float getArea() { return area; }
};
