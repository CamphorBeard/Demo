#ifndef RAYTRACING_RAY_H
#define RAYTRACING_RAY_H

#include <Eigen>
#include <iostream>

using Eigen::Vector3f;

struct Ray
{
    Vector3f origin;
    Vector3f direction;
    double t;  //transportation time,
    double t_min, t_max;

    Ray(const Vector3f& ori, const Vector3f& dir, const double _t = 0.0): origin(ori), direction(dir),t(_t)
    {
        t_min = 0.0;
        t_max = std::numeric_limits<double>::max();
    }

    Vector3f operator()(double t) const { return origin + direction * t; }

    friend std::ostream &operator<<(std::ostream& os, const Ray& r)
    {
        os<<"[origin:="<<r.origin<<", direction="<<r.direction<<", time="<< r.t<<"]\n";
        return os;
    }
};
#endif //RAYTRACING_RAY_H
