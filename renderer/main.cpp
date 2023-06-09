#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include <chrono>
#include <Eigen>

using Eigen::Vector3f;

int main(int argc, const char** argv)
{
    Scene scene(100, 100);  //350, 350  784, 784

    Material* red = new Material(Vector3f(0.0f, 0.0f, 0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(Vector3f(0.0f, 0.0f, 0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(Vector3f(0.0f, 0.0f, 0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material* light = new Material(Vector3f(47.8348f, 38.5664f, 31.0808f));
    light->Kd = Vector3f(0.65f, 0.65f, 0.65f);

    //cornellbox scene
    Vector3f vertsFloor[18] = { {552.8,0.0,0.0}, {0.0,0.0,0.0}, {0.0,0.0,559.2},
                                {0.0,0.0,559.2}, {549.6,0.0,559.2}, {552.8,0.0,0.0},
                                {556.0,548.8,0.0}, {556.0,548.8,559.2}, {0.0,548.8,559.2},
                                {0.0,548.8,559.2}, {0.0,548.8,0.0}, {556.0,548.8,0.0},
                                {549.6,0.0,559.2}, {0.0,0.0,559.2}, {0.0,548.8,559.2},
                                {0.0,548.8,559.2}, {556.0,548.8,559.2}, {549.6,0.0,559.2}, };
    MeshTriangle floor(vertsFloor, 6, white);
    Vector3f vertsLeft[6] = { {552.8,0.0,0.0}, {549.6,0.0,559.2}, {556.0,548.8,559.2},
                              {552.8,0.0,0.0}, {556.0,548.8,559.2}, {556.0,548.8,0.0} };
    MeshTriangle left(vertsLeft, 2, red);
    Vector3f vertsRight[6] = { {0.0,0.0,559.2}, {0.0,  0.0,0.0}, {0.0,548.8,  0.0},
                               {0.0,0.0,559.2}, {0.0,548.8,0.0}, {0.0,548.8,559.2} };
    MeshTriangle right(vertsRight, 2, green);
    Vector3f vertsLight[6] = { {343.0,548.7,227.0}, {343.0,548.7,332.0}, {213.0,548.7,332.0},
                               {343.0,548.7,227.0}, {213.0,548.7,332.0}, {213.0,548.7,227.0} };
    MeshTriangle light_(vertsLight, 2, light);

    MeshTriangle objFile(0);  //solve Debug Error: abort() has been called
    if (argc >= 2)  //using command line add object
    {
        objFile.addValue(MeshTriangle(std::string(argv[1]), white));  //E:/models/cornellbox/shortbox.obj
        scene.Add(&objFile);
    }
    
    scene.Add(&light_);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&floor);
    //scene.buildBVH();

    Renderer r;
    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();
    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}