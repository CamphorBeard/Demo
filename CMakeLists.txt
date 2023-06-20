cmake_minimum_required(VERSION 3.10)
project(Renderer)

set(OpenCV_DIR D:/Library/opencv/build)
find_package(OpenCV REQUIRED)

include_directories(D:/Library/eigen-3.4.0/Eigen)

set(CMAKE_CXX_STANDARD 17)

add_executable(Renderer global.hpp Intersection.hpp main.cpp Material.hpp OBJ_Loader.hpp
                         Object.hpp Ray.hpp Renderer.cpp Renderer.hpp Scene.cpp Scene.hpp Triangle.hpp)
target_link_libraries(Renderer ${OpenCV_LIBRARIES})