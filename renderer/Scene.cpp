#include "Scene.hpp"
#include <iostream>

void scale(MeshTriangle& meshTri, float nx, float ny, float nz)
{
    Eigen::Matrix4f scaleMatrix;
    scaleMatrix <<
        nx, 0, 0, 0,
        0, ny, 0, 0,
        0, 0, nz, 0,
        0, 0, 0, 1;

    meshTri.area = 0;  //scale would change triangle's area
    for(unsigned i=0;i<meshTri.numTriangles;i++)
    {
        Vector3f v0, v1, v2;
        v0 = vec4ToVec3(scaleMatrix * vec3ToVec4(meshTri.triangles[i].v0));
        v1 = vec4ToVec3(scaleMatrix * vec3ToVec4(meshTri.triangles[i].v1));
        v2 = vec4ToVec3(scaleMatrix * vec3ToVec4(meshTri.triangles[i].v2));
        Triangle triTemp(v0, v1, v2, meshTri.m);
        meshTri.triangles[i]= triTemp;
        meshTri.area += triTemp.area;
    }
}

void rotate(MeshTriangle& meshTri, float angle)
{
    angle = angle * M_PI / 180.f;
    Eigen::Matrix4f rotateMatrix;
    rotateMatrix <<
        cos(angle), 0, sin(angle), 0,
        0, 1, 0, 0,
        -sin(angle), 0, cos(angle), 0,
        0, 0, 0, 1;

    for (unsigned i = 0; i < meshTri.numTriangles; i++)
    {
        Vector3f v0, v1, v2;
        v0 = vec4ToVec3(rotateMatrix * vec3ToVec4(meshTri.triangles[i].v0));
        v1 = vec4ToVec3(rotateMatrix * vec3ToVec4(meshTri.triangles[i].v1));
        v2 = vec4ToVec3(rotateMatrix * vec3ToVec4(meshTri.triangles[i].v2));
        Triangle triTemp(v0, v1, v2, meshTri.m);
        meshTri.triangles[i] = triTemp;
    }
}

void translate(MeshTriangle& meshTri, float tx, float ty, float tz)
{
    Eigen::Matrix4f translateMatrix;
    translateMatrix <<
        1, 0, 0, tx,
        0, 1, 0, ty,
        0, 0, 1, tz,
        0, 0, 0, 1;

    for (unsigned i = 0; i < meshTri.numTriangles; i++)
    {
        Vector3f v0, v1, v2;
        v0 = vec4ToVec3(translateMatrix * vec3ToVec4(meshTri.triangles[i].v0));
        v1 = vec4ToVec3(translateMatrix * vec3ToVec4(meshTri.triangles[i].v1));
        v2 = vec4ToVec3(translateMatrix * vec3ToVec4(meshTri.triangles[i].v2));
        Triangle triTemp(v0, v1, v2, meshTri.m);
        meshTri.triangles[i] = triTemp;
    }
}

void Scene::addLight(MeshTriangle& light)
{
    scale(light, boxSize / 2.0 / 5.0, 1, boxSize / 2.0 / 5.0);
    translate(light, 0, boxSize / 2.0 - 2.0, 0);
    objects.push_back(&light);
}

void Scene::addCornellBox(MeshTriangle& box)
{
    scale(box, boxSize / 2.0, boxSize / 2.0, boxSize / 2.0);
    objects.push_back(&box);
}

void Scene::addObjectInBox(MeshTriangle& object, boundingBox& bbx)
{
    //move object to origin where box setted
    translate(object, -(bbx.pointMin.x() + bbx.pointMax.x()) / 2,
                      -(bbx.pointMin.y() + bbx.pointMax.y()) / 2,
                      -(bbx.pointMin.z() + bbx.pointMax.z()) / 2);

    //scale object to suitable size
    float longestLength = std::max(std::max(bbx.pointMax.x() - bbx.pointMin.x(), bbx.pointMax.y() - bbx.pointMin.y()),
                                   bbx.pointMax.z() - bbx.pointMin.z());
    float n = (1.0 / (longestLength / 2.0)) * (boxSize / 2.0) * ratioObjectBox;
    scale(object, n, n, n);

    //move object to box floor
    float distanceToFloor = (boxSize / 2.0) - ((bbx.pointMax.y() - bbx.pointMin.y()) / 2.0) * n;
    translate(object, 0, -distanceToFloor, 0);
    objects.push_back(&object);
}

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

Vector3f Scene::castRay(const Ray &ray) const
{
    Vector3f LOut{ 0.0, 0.0, 0.0 };

    Intersection intersection = getIntersection(ray);
    if (intersection.happened)  //eyeRay gets intersection with scene
    {
        if(intersection.m->hasEmission())  //ray intersects light directly
        {
            LOut = intersection.emit;
            return LOut;
        }
        
        //apart direct and indirect illimination, calculate direct illumination for all intersections to lower noise
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

            float cosSita = N.dot(wIn);
            float cosSitaLight = (-wIn).dot(sampleIntersection.normal);

            LOut.x() += BRDF.x() * LIn.x() * cosSita * cosSitaLight / distance2 / PDF;
            LOut.y() += BRDF.y() * LIn.y() * cosSita * cosSitaLight / distance2 / PDF;
            LOut.z() += BRDF.z() * LIn.z() * cosSita * cosSitaLight / distance2 / PDF;
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
                Vector3f LIn = castRay(secRay);
                float PDF = intersection.m->pdf(wIn, wOut, N);

                LOut.x() += BRDF.x() * LIn.x() * (N.dot(wIn)) / PDF / RussianRoulette;
                LOut.y() += BRDF.y() * LIn.y() * (N.dot(wIn)) / PDF / RussianRoulette;
                LOut.z() += BRDF.z() * LIn.z() * (N.dot(wIn)) / PDF / RussianRoulette;
            }
        }
    }
    return LOut;
}