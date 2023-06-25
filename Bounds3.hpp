#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include <array>
#include <Eigen>
#include "Ray.hpp"

using Eigen::Vector3f;

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box

    Bounds3()
    {
        pMin = Vector3f{ INFINITY,INFINITY,INFINITY };
        pMax = Vector3f{ -INFINITY,-INFINITY,-INFINITY };
    }

    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}

    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()), std::min(p1.z(), p2.z()));
        pMax = Vector3f(std::max(p1.x(), p2.x()), std::max(p1.y(), p2.y()), std::max(p1.z(), p2.z()));
    }

    inline void clear()
    {
        pMin = Vector3f{ INFINITY,INFINITY,INFINITY };
        pMax = Vector3f{ -INFINITY,-INFINITY,-INFINITY };
    }

    inline void update(const Vector3f& p)
    {
        pMin = Vector3f(std::min(pMin.x(), p.x()), std::min(pMin.y(), p.y()), std::min(pMin.z(), p.z()));
        pMax = Vector3f(std::max(pMax.x(), p.x()), std::max(pMax.y(), p.y()), std::max(pMax.z(), p.z()));
    }

    Vector3f Diagonal() const { return pMax - pMin; }  //get bounding-box lengthes in 3 axies

    int maxExtent() const  //get bounding-box's longest side's index
    {
        Vector3f d = Diagonal();
        if (d.x() > d.y() && d.x() > d.z())
            return 0;
        else if (d.y() > d.z())
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const  //get bounding-box surface S
    {
        Vector3f d = Diagonal();
        return 2 * (d.x() * d.y() + d.x() * d.z() + d.y() * d.z());
    }

    Vector3f Centroid() { return 0.5 * (pMin + pMax); }  //get bounding-box's center

    Bounds3 Intersect(const Bounds3& b)  //get two bounding-boxes intersecting box
    {
        return Bounds3(Vector3f(fmax(pMin.x(), b.pMin.x()), fmax(pMin.y(), b.pMin.y()), fmax(pMin.z(), b.pMin.z())),
                       Vector3f(fmin(pMax.x(), b.pMax.x()), fmin(pMax.y(), b.pMax.y()), (pMax.z(), b.pMax.z())));
    }

    Vector3f Offset(const Vector3f& p) const  //get p's bounding-box coordinates
    {
        Vector3f o = p - pMin;
        if (pMax.x() > pMin.x())
            o.x() /= pMax.x() - pMin.x();
        if (pMax.y() > pMin.y())
            o.y() /= pMax.y() - pMin.y();
        if (pMax.z() > pMin.z())
            o.z() /= pMax.z() - pMin.z();
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)  //figure if two bounding-boxes get intersecting part
    {
        bool x = (b1.pMax.x() >= b2.pMin.x()) && (b1.pMin.x() <= b2.pMax.x());
        bool y = (b1.pMax.y() >= b2.pMin.y()) && (b1.pMin.y() <= b2.pMax.y());
        bool z = (b1.pMax.z() >= b2.pMin.z()) && (b1.pMin.z() <= b2.pMax.z());
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)  //figure if a point inside bounding-box
    {
        return (p.x() >= b.pMin.x() && p.x() <= b.pMax.x() && p.y() >= b.pMin.y() &&
                p.y() <= b.pMax.y() && p.z() >= b.pMin.z() && p.z() <= b.pMax.z());
    }

    inline const Vector3f& operator[](int i) const { return (i == 0) ? pMin : pMax; }  //get bounding-box's point with index

    inline bool intersectOrNot(const Ray& ray, const Vector3f& invDir,const std::array<int, 3>& dirisNeg) const;
};

inline bool Bounds3::intersectOrNot(const Ray& ray, const Vector3f& invDir,const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects
    float x_tMin = (pMin.x() - ray.origin.x()) * invDir.x();
    float x_tMax = (pMax.x() - ray.origin.x()) * invDir.x();

    float y_tMin = (pMin.y() - ray.origin.y()) * invDir.y();
    float y_tMax = (pMax.y() - ray.origin.y()) * invDir.y();

    float z_tMin = (pMin.z() - ray.origin.z()) * invDir.z();
    float z_tMax = (pMax.z() - ray.origin.z()) * invDir.z();

    if (!dirIsNeg[0])
    {
        float t = x_tMin;
        x_tMin = x_tMax;
        x_tMax = t;
    }
    if (!dirIsNeg[1])
    {
        float t = y_tMin;
        y_tMin = y_tMax;
        y_tMax = t;
    }
    if (!dirIsNeg[2])
    {
        float t = z_tMin;
        z_tMin = z_tMax;
        z_tMax = t;
    }

    float t_enter = std::max(x_tMin, std::max(y_tMin, z_tMin));
    float t_exit = std::min(x_tMax, std::min(y_tMax, z_tMax));
    if (t_exit >= 0 && t_enter < t_exit)
    {
        return true;
    }
    return false;
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)  //get two bounding-boxes's bounding-box
{
    Bounds3 ret;
    ret.pMin = Vector3f(std::min(b1.pMin.x(),b2.pMin.x()), std::min(b1.pMin.y(), b2.pMin.y()), std::min(b1.pMin.z(), b2.pMin.z()));
    ret.pMax = Vector3f(std::max(b1.pMax.x(), b2.pMax.x()), std::max(b1.pMax.y(), b2.pMax.y()), std::max(b1.pMax.z(), b2.pMax.z()));
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)  //get bounding-box of a point and a bounding-box
{
    Bounds3 ret;
    ret.pMin = Vector3f(std::min(b.pMin.x(), p.x()), std::min(b.pMin.y(), p.y()), std::min(b.pMin.z(), p.z()));
    ret.pMax = Vector3f(std::max(b.pMax.x(), p.x()), std::max(b.pMax.y(), p.y()), std::max(b.pMax.z(), p.z()));
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
