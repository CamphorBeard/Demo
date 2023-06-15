#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "global.hpp"
#include <Eigen>

using Eigen::Vector3f;

class Material
{
public:
    Vector3f m_emission;  //self-emission
    Vector3f Kd; //K diffuse

    inline Material(Vector3f e = Vector3f(0.0f, 0.0f, 0.0f)) :m_emission(e) {  }
    inline Vector3f getEmission(){ return m_emission; }
    inline bool hasEmission()
    {
        if (m_emission.norm() > 0.00001)
            return true;
        else
            return false;
    }

    //sample a direction on intersection's hemisphere by PDF(uniform)
    inline Vector3f sample(const Vector3f& wo, const Vector3f& N)
    {
        //sample a direction in normal coordinate
        Vector3f direction{ 0,0,0 };
        direction.y() = get_random_float(1);  //return random float in range[0,1]
        float randomAngle = 2 * M_PI * (get_random_float(1));
        float rVerticalY = std::sqrt(1.0f - direction.y() * direction.y());
        direction.x() = rVerticalY * std::cos(randomAngle);
        direction.z() = rVerticalY * std::sin(randomAngle);
        direction = direction.normalized();

        //normal coordinate'basic vectors in world coordinate
        Vector3f nSystemY = N.normalized();
        Vector3f nSystemZ = (wo.cross(N)).normalized();  //using wo get a certain coordinate system
        Vector3f nSystemX = (nSystemY.cross(nSystemZ)).normalized();

        //turn normal coordinate into world coordinate,processing like view transform
        Eigen::Matrix3f transformM;
        transformM << nSystemX.x(),nSystemY.x(),nSystemZ.x(),
                      nSystemX.y(),nSystemY.y(),nSystemZ.y(),
                      nSystemX.z(),nSystemY.z(),nSystemZ.z();
        direction = (transformM * direction).normalized();
        return direction;
    }

    // given a ray, calculate the PDF of this ray
    inline float pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N)
    {
        if (wi.dot(N) > 0.0f)
            return 0.5f / M_PI;  //sum solid angle of hemisphere get 2PI
        else
            return 0.0f;
    }

    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N)
    {
        if (N.dot(wi) > 0.0f)
            return Kd / M_PI;  //energy conservation
        else
            return Vector3f(0.0f, 0.0f, 0.0f);
    }
};

#endif //RAYTRACING_MATERIAL_H
