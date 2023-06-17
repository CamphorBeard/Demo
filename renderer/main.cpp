#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "OBJ_Loader.hpp"
#include <chrono>
#include <Eigen>

using Eigen::Vector3f;

int main(int argc, const char** argv)
{
    Scene scene(350, 350);

    Material* red = new Material(Vector3f(0.0f, 0.0f, 0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(Vector3f(0.0f, 0.0f, 0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(Vector3f(0.0f, 0.0f, 0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material* objectMaterial = new Material(Vector3f(0.0f, 0.0f, 0.0f));
    objectMaterial->Kd = Vector3f(0.35f, 0.39f, 1.0f);
    Material* light = new Material(Vector3f(47.8348f, 38.5664f, 31.0808f));
    light->Kd = Vector3f(0.65f, 0.65f, 0.65f);

    Vector3f v0{ 1,1,1 }, v1{ -1,1,1 }, v2{ -1,1,-1 }, v3{ 1,1,-1 },
             v4{ 1,-1,1 }, v5{ -1,-1,1 }, v6{ -1,-1,-1 }, v7{ 1,-1,-1 };
    //cornellBox
    std::vector<Vector3f> vertsFloor{ v0, v2, v3, v0, v1, v2,    //top
                                      v4, v7, v6, v4, v6, v5,    //bottom
                                      v2, v7, v3, v2, v6, v7 };  //back
    MeshTriangle floor(vertsFloor, 6, white);
    std::vector<Vector3f> vertsLeft{ v1, v5, v2, v6, v2, v5 };
    MeshTriangle left(vertsLeft, 2, red);
    std::vector<Vector3f> vertsRight{ v0, v3, v4, v3, v7, v4 };
    MeshTriangle right(vertsRight, 2, green);
    std::vector<Vector3f> vertsLight{ v0, v2, v3, v0, v1, v2 };
    MeshTriangle meshLight(vertsLight, 2, light);

    scene.addLight(meshLight);
    scene.addCornellBox(left);
    scene.addCornellBox(right);
    scene.addCornellBox(floor);

    MeshTriangle inputObject;  //solve Debug Error: abort() has been called
    if (argc >= 2)  //using command line add object
    {
        objl::Loader Loader;
        Loader.LoadFile(std::string(argv[1]));  //E:/models/cornellbox/shortbox.obj
        std::vector<Vector3f> vertsObject;
        unsigned triNumber = 0;
        for (auto mesh : Loader.LoadedMeshes)
        {
            triNumber += mesh.Vertices.size() / 3;  //only fit for triangle mesh
            for (int i = 0; i < mesh.Vertices.size(); i++)
                vertsObject.push_back(Vector3f(mesh.Vertices[i].Position.X, mesh.Vertices[i].Position.Y, mesh.Vertices[i].Position.Z));
        }
        inputObject = MeshTriangle(vertsObject, triNumber, objectMaterial);
        scene.addObjectInBox(inputObject);
    }

    Renderer renderer;
    renderer.rasterizationRender(scene);

    //auto start = std::chrono::system_clock::now();
    //renderer.pathTracingRender(scene);
    //auto stop = std::chrono::system_clock::now();
    //std::cout << "Render complete: \n";
    //std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    //std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    //std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}