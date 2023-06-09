#define _CRT_SECURE_NO_WARNINGS
#include <fstream>
#include <Eigen>
#include "Renderer.hpp"

using Eigen::Vector3f;

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

inline unsigned getPixelIndex(const unsigned& i, const unsigned& j, const unsigned& width) { return j * width + i; }

const float EPSILON = 0.00001;

void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    Vector3f eye_pos(278, 273, -800);  //look at ray tracing direction
    float scale = tan(deg2rad(scene.fov * 0.5));
    float distance = (scene.height / 2.0) / scale;
    
    int spp = 6;  // change the spp value to change sample ammount
    std::cout << "SPP: " << spp << "\n";
    int m = 0;
    for (uint32_t j = 0; j < scene.height; ++j)
    {
        for (uint32_t i = 0; i < scene.width; ++i)
        {
            // generate primary ray direction,get different diffuse directions at same intersection
            float x = eye_pos.x() + scene.width / 2.0 - i - 0.5;
            float y = eye_pos.y() + scene.height / 2.0 - j - 0.5;
            float z = eye_pos.z() + distance;
            Vector3f ijposition{ x,y,z };
            Vector3f dir = (ijposition - eye_pos).normalized();
            
            framebuffer[m] = Vector3f(0.0f, 0.0f, 0.0f);
            for (int k = 0; k < spp; k++)
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  //average each sample's radiance
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i)
    {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x()), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y()), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z()), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
