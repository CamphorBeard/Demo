#pragma once

#include "Scene.hpp"

class Renderer
{
public:
    unsigned spp = 6;  //simple per pixel

    void rasterizationRender(Scene& scene);
    void pathTracingRender(Scene& scene);
};
