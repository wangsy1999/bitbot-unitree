/**
 * @file depth_camera.cpp
 * @brief MuJoCo 深度缓冲读取、米制转换和策略输入预览。
 */
#include "depth_camera.hpp"
#include "policy.hpp"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace g1_parkour
{
    DepthCamera::DepthCamera(const mjModel* model)
    {
        this->CameraId__ = mj_name2id(model, mjOBJ_CAMERA, "parkour_depth");
        if (this->CameraId__ < 0)
        {
            throw std::runtime_error("Missing parkour_depth camera");
        }

        // Bitbot Simulate 已在当前 UI 线程初始化 GLFW。
        auto* previous_context = glfwGetCurrentContext();
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
        glfwWindowHint(GLFW_FOCUSED, GLFW_FALSE);
        glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
        this->Window__ = glfwCreateWindow(640, 360,
            "Parkour depth | 32x18 | 0-2.5m | near: white, far: black", nullptr, nullptr);
        glfwDefaultWindowHints();
        if (!this->Window__)
        {
            throw std::runtime_error("Cannot create Parkour depth OpenGL context");
        }

        glfwSetWindowUserPointer(this->Window__, this);
        glfwSetWindowCloseCallback(this->Window__, [](GLFWwindow* window)
        {
            auto* camera = static_cast<DepthCamera*>(glfwGetWindowUserPointer(window));
            camera->PreviewOpen__.store(false);
            glfwSetWindowShouldClose(window, GLFW_FALSE);
            glfwHideWindow(window); // 仅关闭预览，保留策略采集所需的上下文。
        });

        glfwMakeContextCurrent(this->Window__);
        glfwSwapInterval(0);
        mjr_defaultContext(&this->RenderContext__);
        mjr_makeContext(model, &this->RenderContext__, mjFONTSCALE_100);
        mjr_resizeOffscreen(MujocoDepthLayout::Width, MujocoDepthLayout::Height, &this->RenderContext__);
        glGenTextures(1, &this->PreviewTexture__);
        glBindTexture(GL_TEXTURE_2D, this->PreviewTexture__);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        mjv_defaultScene(&this->Scene__);
        mjv_makeScene(model, &this->Scene__, std::max(2000, model->ngeom * 2 + 100));
        mjv_defaultOption(&this->RenderOption__);
        // 与源适配器一致：只渲染碰撞 group 0，排除视觉副本和 site。
        for (int i = 0; i < mjNGROUP; ++i)
        {
            this->RenderOption__.geomgroup[i] = i == 0;
            this->RenderOption__.sitegroup[i] = 0;
        }
        glfwMakeContextCurrent(previous_context); // 释放后才可在物理线程使用。
    }

    DepthCamera::~DepthCamera()
    {
        if (!this->Window__)
        {
            return;
        }
        auto* previous_context = glfwGetCurrentContext();
        glfwMakeContextCurrent(this->Window__);
        glDeleteTextures(1, &this->PreviewTexture__);
        mjr_freeContext(&this->RenderContext__);
        mjv_freeScene(&this->Scene__);
        glfwMakeContextCurrent(previous_context == this->Window__ ? nullptr : previous_context);
        glfwDestroyWindow(this->Window__);
    }

    DepthCamera::RawDepthImage DepthCamera::Capture(const mjModel* model, mjData* data)
    {
        auto* previous_context = glfwGetCurrentContext();
        glfwMakeContextCurrent(this->Window__);
        mjr_setBuffer(mjFB_OFFSCREEN, &this->RenderContext__);

        mjvCamera camera;
        mjv_defaultCamera(&camera);
        camera.type = mjCAMERA_FIXED;
        camera.fixedcamid = this->CameraId__;
        mjv_updateScene(model, data, &this->RenderOption__, nullptr, &camera, mjCAT_ALL, &this->Scene__);
        const mjrRect viewport{0, 0, MujocoDepthLayout::Width, MujocoDepthLayout::Height};
        mjr_render(viewport, &this->Scene__, &this->RenderContext__);
        mjr_readPixels(nullptr, this->DepthBuffer__.data(), viewport, &this->RenderContext__);
        mjr_finish();
        mjr_restoreBuffer(&this->RenderContext__);
        glfwMakeContextCurrent(previous_context);

        // MuJoCo 2.3.7：非线性 OpenGL 深度，首行位于底部。
        // 转为光轴 Z 米制距离，并翻转为首行位于顶部。
        const double near_plane = model->vis.map.znear * model->stat.extent;
        const double far_plane = model->vis.map.zfar * model->stat.extent;
        RawDepthImage depth;
        for (int y = 0; y < MujocoDepthLayout::Height; ++y)
        {
            for (int x = 0; x < MujocoDepthLayout::Width; ++x)
            {
                const float value = this->DepthBuffer__[(MujocoDepthLayout::Height - 1 - y) * MujocoDepthLayout::Width + x];
                depth[y * MujocoDepthLayout::Width + x] = value >= 1.f ? std::numeric_limits<float>::infinity()
                    : static_cast<float>(near_plane / (1.0 - value * (1.0 - near_plane / far_plane)));
            }
        }
        if (this->PreviewOpen__.load())
        {
            ShowPreview(depth);
        }
        return depth;
    }

    void DepthCamera::ShowPreview(const RawDepthImage& depth)
    {
        // 复用网络输入的裁剪、滤波和归一化，近处亮、远处暗。
        const auto image = Policy::Preprocess(depth);
        std::array<unsigned char, ModelLayout::DepthPixels * 3> rgb;
        for (size_t i = 0; i < image.size(); ++i)
        {
            const auto gray = static_cast<unsigned char>(std::lround((1.f - image[i]) * 255));
            rgb[3 * i] = rgb[3 * i + 1] = rgb[3 * i + 2] = gray;
        }

        auto* previous_context = glfwGetCurrentContext();
        glfwMakeContextCurrent(this->Window__);
        mjr_setBuffer(mjFB_WINDOW, &this->RenderContext__);
        int width, height;
        glfwGetFramebufferSize(this->Window__, &width, &height);
        glViewport(0, 0, width, height);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glDisable(GL_SCISSOR_TEST);
        glDisable(GL_CULL_FACE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glMatrixMode(GL_TEXTURE);
        glLoadIdentity();
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, this->PreviewTexture__);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ModelLayout::DepthWidth, ModelLayout::DepthHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());

        // 图像首行在顶部，与 ONNX 输入方向一致。
        glBegin(GL_QUADS);
        glTexCoord2f(0, 0);
        glVertex2f(-1, 1);
        glTexCoord2f(1, 0);
        glVertex2f(1, 1);
        glTexCoord2f(1, 1);
        glVertex2f(1, -1);
        glTexCoord2f(0, 1);
        glVertex2f(-1, -1);
        glEnd();
        glDisable(GL_TEXTURE_2D);
        glfwSwapBuffers(this->Window__);
        glfwMakeContextCurrent(previous_context);
    }
}
