#pragma once

#include "Scene.hpp"

class Renderer
{
public:
    void rasterizationRender(Scene& scene);

    void pathTracingRender(Scene& scene);
};
