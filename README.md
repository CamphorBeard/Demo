# Renderer
这是一个简单的基于GAMES101作业框架的渲染程序。在耗时较长的路径追踪前可在光栅化渲染场景中对物体位置或方向进行调整。

此程序依赖Eigen及OpenCV库，如果想使用CMakelists.txt进行编译还需要改变其中对应的配置路径。建议使用命令行运行程序，在程序调用后加所选物体文件路径及逐像素采样光线数，类似"./renderer E:/model/cube.obj 30"，若不添加模型文件则渲染空的康奈尔盒子，默认逐像素采样光线数为6。在弹出窗口中即可看到物体在康奈尔盒子中的光栅化渲染图像（目前未加投影），使用键盘A键与D键旋转物体，方向合适后按Esc开始路径追踪渲染，进度条结束后即可看到渲染结果。

This is a simple render program based on GAMES101 homework frame. Try to combine rasterization with path tracing so that the object could be adjusted to a suitable size or location in a crude scene before long-waiting rendering output.

Before build this program, you have to configure Eigen and OpenCV correctly then change configuration path in CMakelists.txt.Using command line to run with object file path and sample per pixel such as "./renderer E:/model/cube.obj 30", a pop-up window would show the object in the Cornellbox scene(without shadow now). Tap A or D to rotate object then tap Esc to path tracing rendering the whole scene, it will be showen after progressing bar finished.
