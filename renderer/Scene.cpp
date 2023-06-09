#include "Scene.hpp"
#include <iostream>

//void Scene::buildBVH() 
//{
//    printf(" - Generating BVH...\n\n");
//    //this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
//}

//Intersection Scene::intersectBVH(const Ray &ray) const
//{
//    //return this->bvh->Intersect(ray);
//    return Intersection();
//}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    //float emit_area_sum = 0;
    //for (uint32_t k = 0; k < objects.size(); ++k)
    //{
    //    if (objects[k]->hasEmit())
    //        emit_area_sum += objects[k]->getArea();
    //}
    //float p = get_random_float(1) * emit_area_sum;
    //emit_area_sum = 0;
    //for (uint32_t k = 0; k < objects.size(); ++k)
    //{
    //    if (objects[k]->hasEmit())
    //    {
    //        emit_area_sum += objects[k]->getArea();
    //        if (p <= emit_area_sum)
    //        {
    //            objects[k]->Sample(pos, pdf);
    //            break;
    //        }
    //    }
    //}

    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            objects[k]->Sample(pos, pdf);
            break;
        }
    }
}

Intersection Scene::getIntersection(const Ray& ray) const
{
    Intersection intersection;
    double distance = INFINITY;
    for (const auto& object : objects)
    {
        Intersection intersectionTemp = object->getIntersection(ray);
        if (intersectionTemp.happened==true&&intersectionTemp.distance < distance)
        {
            intersection = intersectionTemp;
            distance = intersection.distance;
        }
    }
    return intersection;
}

Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f hitColor{ 0,0,0 };
    if (depth > this->maxDepth)
        return hitColor;

    Intersection intersection = getIntersection(ray);
    if (intersection.happened)  //eyeRay get intersection with scene
    {
        //apart direct and indirect illimination, calculate direct illumination for all intersections to lower noise
        Vector3f LOut = intersection.emit;

        //uniformly sample a intersection on light area
        Intersection sampleIntersection;
        float PDFSampleLight;
        sampleLight(sampleIntersection, PDFSampleLight);

        //generate sampleRay,if it could hit light then calculate intersection's direct illumination
        Vector3f distance = sampleIntersection.coords - intersection.coords;
        float distance2 = (distance.norm()) * (distance.norm());
        Vector3f sampleDirection = distance.normalized();
        Ray sampleRay(intersection.coords + 0.1 * sampleDirection, sampleDirection);

        //sampleRay hit light at sampleIntersection meaning not been covered
        Intersection tempIntersection = getIntersection(sampleRay);
        if (tempIntersection.happened && tempIntersection.m->hasEmission())
        {
            //sample on light Monte Carlo rendering equation
            Vector3f wIn = sampleDirection;  //for calculation convenience,make inside radiance's direction outside
            Vector3f wOut = -(ray.direction);
            Vector3f N = intersection.normal;
            Vector3f BRDF = intersection.m->eval(wIn, wOut, N);
            Vector3f LIn = sampleIntersection.emit;
            float PDF = PDFSampleLight;

            LOut.x() += BRDF.x() * LIn.x() * (N.dot(wIn)) * ((-wIn).dot(sampleIntersection.normal)) / distance2 / PDF;
            LOut.y() += BRDF.y() * LIn.y() * (N.dot(wIn)) * ((-wIn).dot(sampleIntersection.normal)) / distance2 / PDF;
            LOut.z() += BRDF.z() * LIn.z() * (N.dot(wIn)) * ((-wIn).dot(sampleIntersection.normal)) / distance2 / PDF;
        }

        //using RussianRoulette to judge if generate secRay to calculate indirect illumination
        if (get_random_float(1) < RussianRoulette)
        {
            Vector3f wOut = -(ray.direction);
            Vector3f N = intersection.normal;

            //generate secRay,if it hit a not-light object then calculate intersection's indirect illumination
            Ray secRay(intersection.coords, (intersection.m->sample(wOut, N)).normalized());
            Intersection secIntersection = getIntersection(secRay);
            if (secIntersection.happened && !secIntersection.m->hasEmission())
            {
                Vector3f wIn = secRay.direction;
                Vector3f BRDF = intersection.m->eval(wIn, wOut, N);
                Vector3f LIn = castRay(secRay, 0);
                float PDF = intersection.m->pdf(wIn, wOut, N);

                LOut.x() += BRDF.x() * LIn.x() * (N.dot(wIn)) / PDF / RussianRoulette;
                LOut.y() += BRDF.y() * LIn.y() * (N.dot(wIn)) / PDF / RussianRoulette;
                LOut.z() += BRDF.z() * LIn.z() * (N.dot(wIn)) / PDF / RussianRoulette;
            }
        }

        hitColor = LOut;
    }
    return hitColor;
}