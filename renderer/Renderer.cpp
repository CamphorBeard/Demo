#define _CRT_SECURE_NO_WARNINGS
#include <fstream>
#include <Eigen>
#include <opencv2/opencv.hpp>
#include "Renderer.hpp"

using Eigen::Vector3f;

inline unsigned getPixelIndex(const unsigned& i, const unsigned& j, const unsigned& width) { return j * width + i; }

const float EPSILON = 0.00001;

float crossProduct(float ux, float uy, float vx, float vy)
{
    return(ux * vy - uy * vx);
}

bool insideTriangle(float x, float y, const Triangle& v)
{
    float f1 = 0, f2 = 0, f3 = 0;
    f1 = crossProduct(v.v1.x() - v.v0.x(), v.v1.y() - v.v0.y(), x - v.v0.x(), y - v.v0.y());
    f2 = crossProduct(v.v2.x() - v.v1.x(), v.v2.y() - v.v1.y(), x - v.v1.x(), y - v.v1.y());
    f3 = crossProduct(v.v0.x() - v.v2.x(), v.v0.y() - v.v2.y(), x - v.v2.x(), y - v.v2.y());
    return((f1 >= 0 && f2 >= 0 && f3 >= 0) || (f1 <= 0 && f2 <= 0 && f3 <= 0));
}

void Renderer::rasterizationRender(Scene& scene)
{
    std::vector<Vector3f> frameBuffer(scene.screenWidth * scene.screenHeight);
    for (unsigned i = 0; i < frameBuffer.size(); i++)
        frameBuffer[i] = Vector3f(0.0, 0.0, 0.0);
    std::vector<float> depthBuffer(scene.screenWidth * scene.screenHeight);
    for (unsigned i = 0; i < frameBuffer.size(); i++)
        depthBuffer[i] = INFINITY;
    
    float eyeToBoxFront = (scene.boxSize / 2.0) / tan(scene.fov / 2.0);
    Vector3f eyePosition(0, 0, scene.boxSize / 2.0 + eyeToBoxFront);

    //iterate all projected meshtriangles
    for (MeshTriangle meshTri : scene.meshTris)
    {
        scene.viewTransform(meshTri, eyePosition);
        scene.projectTransform(meshTri);
        //for one projected meshtriangle,get all triangles belong to it
        for (Triangle& tri : meshTri.triangles)
        {
            //Viewport transformation
            tri.v0 = Vector3f(scene.screenWidth / 2.0 * (tri.v0.x() + 1.0), scene.screenHeight / 2.0 * (tri.v0.y() + 1.0), tri.v0.z());
            tri.v1 = Vector3f(scene.screenWidth / 2.0 * (tri.v1.x() + 1.0), scene.screenHeight / 2.0 * (tri.v1.y() + 1.0), tri.v1.z());
            tri.v2 = Vector3f(scene.screenWidth / 2.0 * (tri.v2.x() + 1.0), scene.screenHeight / 2.0 * (tri.v2.y() + 1.0), tri.v2.z());
            
            std::vector <int> rangeX{ 0,0 };
            std::vector <int> rangeY{ 0,0 };
            rangeX[0] = std::min(std::min(tri.v0.x(), tri.v1.x()), tri.v2.x());
            rangeX[1] = std::max(std::max(tri.v0.x(), tri.v1.x()), tri.v2.x());
            rangeY[0] = std::min(std::min(tri.v0.y(), tri.v1.y()), tri.v2.y());
            rangeY[1] = std::max(std::max(tri.v0.y(), tri.v1.y()), tri.v2.y());

            for (unsigned i = rangeX[0]; i <= rangeX[1]; i++)
            {
                for (unsigned j = rangeY[0]; j <= rangeY[1]; j++)
                {
                    if(insideTriangle(i+0.5, j+0.5, tri))
                    {
                        unsigned index = j * scene.screenWidth + i;
                        frameBuffer[index] = 255.0 * tri.m->Kd;
                    }
                }
            }
        }
    }

    int key = 0;
    while (key != 27)  //esc
    {
        cv::Mat image(scene.screenWidth, scene.screenHeight, CV_32FC3, frameBuffer.data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);
    }
}

void Renderer::pathTracingRender(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.screenWidth * scene.screenHeight);

    //scene cornellBox center at coordinates origin,camera set on z aies look at -z
    float eyeToBoxFront = (scene.boxSize / 2.0) / tan(scene.fov / 2.0);
    Vector3f eyePosition(0, 0, scene.boxSize / 2.0 + eyeToBoxFront);
    float eyeToScreen = (scene.screenHeight / 2.0) / tan(scene.fov / 2.0);
    
    int spp = 1;  // change the spp value to change sample ammount
    std::cout << "SPP: " << spp << "\n";
    int m = 0;
    for (uint32_t j = 0; j < scene.screenHeight; ++j)
    {
        for (uint32_t i = 0; i < scene.screenWidth; ++i)
        {
            // generate primary ray direction,get different diffuse directions at same intersection
            float x = eyePosition.x() - scene.screenWidth / 2.0 + i + 0.5;
            float y = eyePosition.y() + scene.screenHeight / 2.0 - j - 0.5;
            float z = eyePosition.z() - eyeToScreen;

            Vector3f ijPosition{ x,y,z };  //screen pixel(i,j)'s coordinates
            Vector3f dir = (ijPosition - eyePosition).normalized();
            
            framebuffer[m] = Vector3f(0.0f, 0.0f, 0.0f);
            for (int k = 0; k < spp; k++)
                framebuffer[m] += scene.pathTracing(Ray(eyePosition, dir)) / spp;  //average each sample's radiance
            m++;
        }
        UpdateProgress(j / (float)scene.screenHeight);
    }
    UpdateProgress(1.f);

    for (unsigned i = 0; i < framebuffer.size(); i++)
        framebuffer[i] = framebuffer[i] * 255.0f;
    int key = 0;
    std::string filename = "output.ppm";
    while (key != 27)  //esc
    {
        cv::Mat image(scene.screenWidth, scene.screenHeight, CV_32FC3, framebuffer.data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);
    }
}
