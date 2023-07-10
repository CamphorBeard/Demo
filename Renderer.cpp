#define _CRT_SECURE_NO_WARNINGS
#include <Eigen>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <fstream>
#include <ctime>
#include "Renderer.hpp"

using Eigen::Vector3f;

inline float crossProductValue(Eigen::Vector2f u, Eigen::Vector2f v) { return (u.x() * v.y() - u.y() * v.x()); }

bool insideTriangle(float x, float y, const Triangle& tri)
{
    Eigen::Vector2f p{ x,y };
    Eigen::Vector2f v0{ tri.v0.x(),tri.v0.y() }, v1{ tri.v1.x(),tri.v1.y() }, v2{ tri.v2.x(),tri.v2.y() };
    float f0 = crossProductValue(p - v0, v1 - v0);
    float f1 = crossProductValue(p - v1, v2 - v1);
    float f2 = crossProductValue(p - v2, v0 - v2);
    return((f0 >= 0 && f1 >= 0 && f2 >= 0) || (f0 <= 0 && f1 <= 0 && f2 <= 0));
}

Vector3f getBarycentricCoordinates(float x, float y, const Triangle& tri)
{
    Eigen::Vector2f p{ x,y };
    Eigen::Vector2f v0{ tri.v0.x(),tri.v0.y() }, v1{ tri.v1.x(),tri.v1.y() }, v2{ tri.v2.x(),tri.v2.y() };
    float alpha = (0.5 * crossProductValue(p - v1, v2 - v1)) / (0.5 * crossProductValue(v1 - v0, v2 - v0));
    float beta = (0.5 * crossProductValue(p - v2, v0 - v2)) / (0.5 * crossProductValue(v1 - v0, v2 - v0));
    float gamma = (0.5 * crossProductValue(p - v0, v1 - v0)) / (0.5 * crossProductValue(v1 - v0, v2 - v0));
    return Vector3f(alpha, beta, gamma);
}

Vector3f shading(const Vector3f& lightPosition, float lightIntensity, const Vector3f& position, const Vector3f& normal, const Vector3f& kd)
{
    Vector3f PToLight = lightPosition - position;
    float distance2 = PToLight.dot(PToLight);
    PToLight = PToLight.normalized();

    Vector3f Diffuse = kd * (lightIntensity / distance2) * std::max(0.0f, normal.dot(PToLight));
    Vector3f Ambient = kd * 0.3f;

    return (Diffuse + Ambient) * 255.f;
}

void Renderer::rasterizationRender(Scene& scene)
{
    std::vector<Vector3f> frameBuffer(scene.screenWidth * scene.screenHeight);
    std::vector<float> depthBuffer(scene.screenWidth * scene.screenHeight);
    for (unsigned i = 0; i < depthBuffer.size(); i++)
        depthBuffer[i] = INFINITY;

    std::vector<Vector3f> background(scene.screenWidth * scene.screenHeight);
    for (MeshTriangle* meshTri : scene.meshTris)
    {
        if (meshTri->isCornellBox)
        {
            MeshTriangle meshTriTemp = *meshTri;

            if (meshTriTemp.hasEmit()) 
                scene.translate(meshTriTemp, 0, -0.5, 0);  //avoid z-fighting
            
            scene.viewTransform(meshTriTemp);
            scene.projectTransform(meshTriTemp);

            //Viewport transformation
            scene.translate(meshTriTemp, 1, 1, 0);
            scene.scale(meshTriTemp, scene.screenWidth / 2.0, scene.screenHeight / 2.0, 1);

            for (unsigned triIndex = 0; triIndex < meshTriTemp.numTriangles; triIndex++)
            {
                Triangle projectedTri = meshTriTemp.triangles[triIndex];
                Triangle originalTri = meshTri->triangles[triIndex];

                //perspective projected triangle's axis aligned bounding box
                std::vector <int> rangeX{ 0,0 };
                std::vector <int> rangeY{ 0,0 };
                rangeX[0] = std::min(std::min(projectedTri.v0.x(), projectedTri.v1.x()), projectedTri.v2.x());
                rangeX[1] = std::max(std::max(projectedTri.v0.x(), projectedTri.v1.x()), projectedTri.v2.x());
                rangeY[0] = std::min(std::min(projectedTri.v0.y(), projectedTri.v1.y()), projectedTri.v2.y());
                rangeY[1] = std::max(std::max(projectedTri.v0.y(), projectedTri.v1.y()), projectedTri.v2.y());

                for (unsigned i = rangeX[0]; i <= rangeX[1]; i++)
                {
                    for (unsigned j = rangeY[0]; j <= rangeY[1]; j++)
                    {
                        if (insideTriangle(i + 0.5, j + 0.5, projectedTri))
                        {
                            Vector3f baryCoord = getBarycentricCoordinates(i + 0.5, j + 0.5, projectedTri);

                            float zInterpolation = 1 / (baryCoord.x() / projectedTri.v0.z() +
                                                        baryCoord.y() / projectedTri.v1.z() +
                                                        baryCoord.z() / projectedTri.v2.z());

                            unsigned index = j * scene.screenWidth + i;
                            if (depthBuffer[index] > zInterpolation)
                            {
                                depthBuffer[index] = zInterpolation;

                                Vector3f lightPosition{ 0, scene.boxSize / 2.0f, 0 };
                                float lightIntensity = scene.boxSize * 100000.0f / 550.0f;
                                Vector3f ijPosition = (baryCoord[0] * originalTri.v0 / projectedTri.v0.z() +
                                                       baryCoord[1] * originalTri.v1 / projectedTri.v1.z() +
                                                       baryCoord[2] * originalTri.v2 / projectedTri.v2.z()) / (1 / zInterpolation);
                                Vector3f normal = originalTri.normal;
                                Vector3f kd = originalTri.m->Kd;

                                background[index] = shading(lightPosition, lightIntensity, ijPosition, normal, kd);
                            }
                        }
                    }
                }
            }
        }
    }

    float angle = 0.0f;
    int key = 0;
    while (key != 27)  //esc
    {
        //for (unsigned i = 0; i < frameBuffer.size(); i++)
        //    frameBuffer[i] = background[i];
        frameBuffer = background;
        for (unsigned i = 0; i < depthBuffer.size(); i++)
            depthBuffer[i] = INFINITY;
        
        for (MeshTriangle* meshTri : scene.meshTris)
        {
            if (!meshTri->isCornellBox)
            {
                MeshTriangle meshTriTemp = *meshTri;
                scene.rotate(meshTriTemp, angle);
                scene.viewTransform(meshTriTemp);
                scene.projectTransform(meshTriTemp);

                //Viewport transformation
                scene.translate(meshTriTemp, 1, 1, 0);
                scene.scale(meshTriTemp, scene.screenWidth / 2.0, scene.screenHeight / 2.0, 1);

                for (unsigned triIndex = 0; triIndex < meshTriTemp.numTriangles; triIndex++)
                {
                    Triangle projectedTri = meshTriTemp.triangles[triIndex];
                    Triangle originalTri = meshTri->triangles[triIndex];

                    Vector3f lightPosition{ 0, scene.boxSize / 2.0f, 0 };
                    float lightIntensity = scene.boxSize * 100000.0f / 550.0f;
                    Vector3f color = shading(lightPosition, lightIntensity, originalTri.v0, originalTri.normal, originalTri.m->Kd);

                    //perspective projected triangle's axis aligned bounding box
                    std::vector <int> rangeX{ 0,0 };
                    std::vector <int> rangeY{ 0,0 };
                    rangeX[0] = std::min(std::min(projectedTri.v0.x(), projectedTri.v1.x()), projectedTri.v2.x());
                    rangeX[1] = std::max(std::max(projectedTri.v0.x(), projectedTri.v1.x()), projectedTri.v2.x());
                    rangeY[0] = std::min(std::min(projectedTri.v0.y(), projectedTri.v1.y()), projectedTri.v2.y());
                    rangeY[1] = std::max(std::max(projectedTri.v0.y(), projectedTri.v1.y()), projectedTri.v2.y());

                    for (unsigned i = rangeX[0]; i <= rangeX[1]; i++)
                    {
                        for (unsigned j = rangeY[0]; j <= rangeY[1]; j++)
                        {
                            if (insideTriangle(i + 0.5, j + 0.5, projectedTri))
                            {
                                Vector3f baryCoord = getBarycentricCoordinates(i + 0.5, j + 0.5, projectedTri);

                                float zInterpolation = 1 / (baryCoord.x() / projectedTri.v0.z() +
                                                            baryCoord.y() / projectedTri.v1.z() +
                                                            baryCoord.z() / projectedTri.v2.z());

                                unsigned index = j * scene.screenWidth + i;
                                if (depthBuffer[index] > zInterpolation)
                                {
                                    depthBuffer[index] = zInterpolation;
                                    frameBuffer[index] = color;
                                }
                            }
                        }
                    }
                }
            }
        }

        cv::Mat image(scene.screenWidth, scene.screenHeight, CV_32FC3, frameBuffer.data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::namedWindow("image", 0);
        cv::resizeWindow("image", cv::Size(scene.screenWidth * 458.0f / 800.0f, scene.screenHeight * 458.0f / 800.0f));
        cv::imshow("image", image);
        key = cv::waitKey(10);
        if (key == 'a')
            angle -= 10;
        else if (key == 'd')
            angle += 10;
    }
    scene.rotateAngle = angle;
}

void Renderer::pathTracingRender(Scene& scene)
{
    for (MeshTriangle* meshTri : scene.meshTris)
    {
        if (!meshTri->isCornellBox)
        {
            scene.rotate(*meshTri, scene.rotateAngle);
            meshTri->buildBVH();
        }
    }
    
    time_t start, stop;
    time(&start);

    std::vector<Vector3f> framebuffer(scene.screenWidth * scene.screenHeight);

    //int spp = 6;  // change the spp value to change sample ammount
    std::cout << "SPP: " << spp << "\n";

    //multiThreads
    const unsigned threadsNum = 20;
    int rows = scene.screenHeight / threadsNum;
    std::thread threads[threadsNum];
    
    unsigned count = 0;
    std::mutex mtx;
    auto threadFunc = [&](unsigned jMin, unsigned jMax)
    {
        for (uint32_t j = jMin; j < jMax; j++)
        {
            for (uint32_t i = 0; i < scene.screenWidth; i++)
            {
                // generate primary ray direction,get different diffuse directions at same intersection
                float x = scene.eyePosition.x() - scene.screenWidth / 2.0 + i + 0.5;
                float y = scene.eyePosition.y() + scene.screenHeight / 2.0 - j - 0.5;
                float z = scene.eyePosition.z() - scene.eyeToScreen;

                Vector3f ijPosition{ x,y,z };  //screen pixel(i,j)'s coordinates
                Vector3f dir = (ijPosition - scene.eyePosition).normalized();

                unsigned index = j * scene.screenWidth + i;
                framebuffer[index] = Vector3f(0.0f, 0.0f, 0.0f);
                for (int k = 0; k < spp; k++)
                    framebuffer[index] += scene.pathTracing(Ray(scene.eyePosition, dir)) / spp;  //average each sample's radiance
            }
            mtx.lock();
            count++;
            UpdateProgress((float)count / (float)(rows * threadsNum));
            mtx.unlock();
        }
    };
    
    for (unsigned k = 0; k < threadsNum; k++) 
    {
        threads[k] = std::thread(threadFunc, k * rows, (k + 1) * rows);
    }
    for (unsigned k = 0; k < threadsNum; k++) 
    {
        threads[k].join();
    }

    ////without multiThread
    ////int m = 0;
    //for (uint32_t j = 0; j < scene.screenHeight; ++j)
    //{
    //    for (uint32_t i = 0; i < scene.screenWidth; ++i)
    //    {
    //        // generate primary ray direction,get different diffuse directions at same intersection
    //        float x = scene.eyePosition.x() - scene.screenWidth / 2.0 + i + 0.5;
    //        float y = scene.eyePosition.y() + scene.screenHeight / 2.0 - j - 0.5;
    //        float z = scene.eyePosition.z() - scene.eyeToScreen;
    //
    //        Vector3f ijPosition{ x,y,z };  //screen pixel(i,j)'s coordinates
    //        Vector3f dir = (ijPosition - scene.eyePosition).normalized();
    //        
    //        unsigned index = j * scene.screenWidth + i;
    //
    //        framebuffer[index] = Vector3f(0.0f, 0.0f, 0.0f);  //?
    //        for (int k = 0; k < spp; k++)
    //            framebuffer[index] += scene.pathTracing(Ray(scene.eyePosition, dir)) / spp;  //average each sample's radiance
    //        //m++;
    //    }
    //    UpdateProgress(j / (float)scene.screenHeight);
    //}

    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("output.ppm", "wb");
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

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);
    printf("\nRender complete: \nTime Taken: %i hours, %i minutes, %i seconds\n\n", hrs, mins, secs);

    int key = 0;
    while (key != 27)  //esc
    {
        cv::Mat image = cv::imread("output.ppm");
        cv::namedWindow("image", 0);
        cv::resizeWindow("image", cv::Size(scene.screenWidth * 458.0f / 800.0f, scene.screenHeight * 458.0f / 800.0f));
        cv::imshow("image", image);
        key = cv::waitKey(10);
    }
}
