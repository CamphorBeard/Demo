#define _CRT_SECURE_NO_WARNINGS
#include <fstream>
#include <Eigen>
#include "Renderer.hpp"

using Eigen::Vector3f;

inline unsigned getPixelIndex(const unsigned& i, const unsigned& j, const unsigned& width) { return j * width + i; }

const float EPSILON = 0.00001;

void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.screenWidth * scene.screenHeight);

    //scene cornellBox center at coordinates origin,camera set on z aies look at -z
    float eyeToBoxFront = (scene.boxSize / 2.0) / tan(scene.fov / 2.0);
    Vector3f eyePosition(0, 0, scene.boxSize / 2.0 + eyeToBoxFront);
    float eyeToScreen = (scene.screenHeight / 2.0) / tan(scene.fov / 2.0);
    
    int spp = 6;  // change the spp value to change sample ammount
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
                framebuffer[m] += scene.castRay(Ray(eyePosition, dir)) / spp;  //average each sample's radiance
            m++;
        }
        UpdateProgress(j / (float)scene.screenHeight);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
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
}
