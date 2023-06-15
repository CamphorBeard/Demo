#define _CRT_SECURE_NO_WARNINGS
#include <Eigen>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "Renderer.hpp"

using Eigen::Vector3f;

float crossProduct(float ux, float uy, float vx, float vy) { return(ux * vy - uy * vx); }

bool insideTriangle(float x, float y, const Triangle& tri)
{
    float f1 = crossProduct(tri.v1.x() - tri.v0.x(), tri.v1.y() - tri.v0.y(), x - tri.v0.x(), y - tri.v0.y());
    float f2 = crossProduct(tri.v2.x() - tri.v1.x(), tri.v2.y() - tri.v1.y(), x - tri.v1.x(), y - tri.v1.y());
    float f3 = crossProduct(tri.v0.x() - tri.v2.x(), tri.v0.y() - tri.v2.y(), x - tri.v2.x(), y - tri.v2.y());
    return((f1 >= 0 && f2 >= 0 && f3 >= 0) || (f1 <= 0 && f2 <= 0 && f3 <= 0));
}

Vector3f getBarycentricCoordinates(float x, float y, const Triangle& tri)
{
    float f1 = (x * (tri.v1.y() - tri.v2.y()) + (tri.v2.x() - tri.v1.x()) * y + tri.v1.x() * tri.v2.y() - tri.v2.x() * tri.v1.y()) / (tri.v0.x() * (tri.v1.y() - tri.v2.y()) + (tri.v2.x() - tri.v1.x()) * tri.v0.y() + tri.v1.x() * tri.v2.y() - tri.v2.x() * tri.v1.y());
    float f2 = (x * (tri.v2.y() - tri.v0.y()) + (tri.v0.x() - tri.v2.x()) * y + tri.v2.x() * tri.v0.y() - tri.v0.x() * tri.v2.y()) / (tri.v1.x() * (tri.v2.y() - tri.v0.y()) + (tri.v0.x() - tri.v2.x()) * tri.v1.y() + tri.v2.x() * tri.v0.y() - tri.v0.x() * tri.v2.y());
    float f3 = 1 - f1 - f2;
    return Vector3f(f1, f2, f3);
}

void Renderer::rasterizationRender(Scene& scene)
{
    std::vector<Vector3f> frameBuffer(scene.screenWidth * scene.screenHeight);
    std::vector<float> depthBuffer(scene.screenWidth * scene.screenHeight);
    
    int key = 0;
    while (key != 27)  //esc
    {
        for (unsigned i = 0; i < frameBuffer.size(); i++)
            frameBuffer[i] = Vector3f(0.0, 0.0, 0.0);
        for (unsigned i = 0; i < frameBuffer.size(); i++)
            depthBuffer[i] = INFINITY;
        
        for (MeshTriangle* meshTri : scene.meshTris)
        {
            MeshTriangle meshTriTemp = *meshTri;
            if (meshTriTemp.isObject == true)
            {
                scene.rotate(meshTriTemp, scene.rotateAngle);
            }
            
            scene.viewTransform(meshTriTemp);
            scene.projectTransform(meshTriTemp);
            
            for (Triangle& tri : meshTriTemp.triangles)
            {
                //Viewport transformation
                tri.v0 = Vector3f(scene.screenWidth / 2.0 * (tri.v0.x() + 1.0), scene.screenHeight / 2.0 * (tri.v0.y() + 1.0), tri.v0.z());
                tri.v1 = Vector3f(scene.screenWidth / 2.0 * (tri.v1.x() + 1.0), scene.screenHeight / 2.0 * (tri.v1.y() + 1.0), tri.v1.z());
                tri.v2 = Vector3f(scene.screenWidth / 2.0 * (tri.v2.x() + 1.0), scene.screenHeight / 2.0 * (tri.v2.y() + 1.0), tri.v2.z());

                //perspective projected triangle's aies alian bounding box
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
                        if (insideTriangle(i + 0.5, j + 0.5, tri))
                        {
                            Vector3f baryCoord = getBarycentricCoordinates(i + 0.5, j + 0.5, tri);

                            float w_reciprocal = 1.0 / (baryCoord[0] + baryCoord[1] + baryCoord[2]);
                            float z_interpolated = baryCoord[0] * tri.v0.z() + baryCoord[1] * tri.v1.z() + baryCoord[2] * tri.v2.z();
                            z_interpolated *= w_reciprocal;
                            
                            unsigned index = j * scene.screenWidth + i;
                            //if (depthBuffer[index] > z_interpolated)
                            //{
                            //    depthBuffer[index] = z_interpolated;
                                frameBuffer[index] = 255.0 * tri.m->Kd;
                            //}
                        }
                    }
                }
            }
        }

        cv::Mat image(scene.screenWidth, scene.screenHeight, CV_32FC3, frameBuffer.data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);
        if (key == 'a')
            scene.rotateAngle -= 10;
        else if (key == 'd')
            scene.rotateAngle += 10;
    }
}

void Renderer::pathTracingRender(Scene& scene)
{
    for (MeshTriangle* meshTri : scene.meshTris)
    {
        if (meshTri->isObject == true)
            scene.rotate(*meshTri, scene.rotateAngle);
    }
    
    std::vector<Vector3f> framebuffer(scene.screenWidth * scene.screenHeight);

    int spp = 1;  // change the spp value to change sample ammount
    //std::cout << "SPP: " << spp << "\n";
    int m = 0;
    for (uint32_t j = 0; j < scene.screenHeight; ++j)
    {
        for (uint32_t i = 0; i < scene.screenWidth; ++i)
        {
            // generate primary ray direction,get different diffuse directions at same intersection
            float x = scene.eyePosition.x() - scene.screenWidth / 2.0 + i + 0.5;
            float y = scene.eyePosition.y() + scene.screenHeight / 2.0 - j - 0.5;
            float z = scene.eyePosition.z() - scene.eyeToScreen;

            Vector3f ijPosition{ x,y,z };  //screen pixel(i,j)'s coordinates
            Vector3f dir = (ijPosition - scene.eyePosition).normalized();
            
            framebuffer[m] = Vector3f(0.0f, 0.0f, 0.0f);
            for (int k = 0; k < spp; k++)
                framebuffer[m] += scene.pathTracing(Ray(scene.eyePosition, dir)) / spp;  //average each sample's radiance
            m++;
        }
        UpdateProgress(j / (float)scene.screenHeight);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("fileOutput.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.screenWidth, scene.screenHeight);
    for (auto i = 0; i < scene.screenHeight * scene.screenWidth; ++i)
    {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x()), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y()), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z()), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);

    for (unsigned i = 0; i < framebuffer.size(); i++)
        framebuffer[i] = framebuffer[i] * 255.0f;
    int key = 0;
    std::string filename = "cvOutput.ppm";
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
