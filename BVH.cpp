#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p) : primitives(std::move(p))
{
    time_t start, stop;
    time(&start);

    if (primitives.empty())
        return;
    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);
    printf("\nBVH Generation complete: \nTime Taken: %i hours, %i minutes, %i seconds\n\n", hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    if (objects.size() == 1)  //just one triangle
    {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2)  //two triangles
    {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else  //more than two triangles
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)  //get bounding-box of all triangles'bounding-boxes'center
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());

        int dim = centroidBounds.maxExtent();  //get bounding-box's longest side's index
        switch (dim) 
        {
        case 0:  //x axis side get the longest length
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                { return f1->getBounds().Centroid().x() < f2->getBounds().Centroid().x(); });
            break;
        case 1:  //y axis side get the longest length
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                { return f1->getBounds().Centroid().y() < f2->getBounds().Centroid().y(); });
            break;
        case 2:  //z axis side get the longest length
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) 
                { return f1->getBounds().Centroid().z() < f2->getBounds().Centroid().z(); });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        //assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    } 
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;

    isect = getBVHIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getBVHIntersection(BVHBuildNode* node, const Ray& ray) const
{
    Intersection isect;
    
    Vector3f invDir{ 1.0f / ray.direction.x(),1.0f / ray.direction.y(), 1.0f / ray.direction.z() };
    std::array<int, 3> dirIsNeg{ ray.direction.x() > 0,ray.direction.y() > 0 ,ray.direction.z() > 0 };
    if (node->bounds.intersectOrNot(ray, invDir, dirIsNeg))
    {
        if (node->object == nullptr)
        {
            Intersection isectLeft = getBVHIntersection(node->left, ray);
            Intersection isectRight = getBVHIntersection(node->right, ray);
            isect = isectLeft.distance < isectRight.distance ? isectLeft : isectRight;
        }
        else
            isect = node->object->getIntersection(ray);
    }
    return isect;
}