// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 RealSense, Inc. All Rights Reserved.
// Minimal window and text helpers extracted from librealsense examples/example.hpp.

#pragma once

#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#include "stb_easy_font.h"

#include <functional>
#include <stdexcept>
#include <vector>

inline void draw_text(int x, int y, const char* text)
{
    std::vector<char> buffer(60000);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 16, &buffer[0]);
    glDrawArrays(GL_QUADS, 0, 4 * stb_easy_font_print((float)x, (float)(y - 7), (char*)text, nullptr, &buffer[0], (int)buffer.size()));
    glDisableClientState(GL_VERTEX_ARRAY);
}

class window
{
public:
    std::function<void(bool)> on_left_mouse = [](bool) {};
    std::function<void(double, double)> on_mouse_scroll = [](double, double) {};
    std::function<void(double, double)> on_mouse_move = [](double, double) {};
    std::function<void(int)> on_key_release = [](int) {};

    window(int width, int height, const char* title)
        : _width(width), _height(height)
    {
        glfwInit();
        win = glfwCreateWindow(width, height, title, nullptr, nullptr);
        if (!win)
            throw std::runtime_error("Could not open an OpenGL window");
        glfwMakeContextCurrent(win);
        glfwSetWindowUserPointer(win, this);
        glfwSetMouseButtonCallback(win, [](GLFWwindow* w, int button, int action, int) {
            auto s = (window*)glfwGetWindowUserPointer(w);
            if (button == 0)
                s->on_left_mouse(action == GLFW_PRESS);
        });
        glfwSetScrollCallback(win, [](GLFWwindow* w, double xoffset, double yoffset) {
            ((window*)glfwGetWindowUserPointer(w))->on_mouse_scroll(xoffset, yoffset);
        });
        glfwSetCursorPosCallback(win, [](GLFWwindow* w, double x, double y) {
            ((window*)glfwGetWindowUserPointer(w))->on_mouse_move(x, y);
        });
        glfwSetKeyCallback(win, [](GLFWwindow* w, int key, int, int action, int) {
            if (action == GLFW_RELEASE)
                ((window*)glfwGetWindowUserPointer(w))->on_key_release(key);
        });
    }

    ~window()
    {
        glfwDestroyWindow(win);
        glfwTerminate();
    }

    float width() const { return (float)_width; }
    float height() const { return (float)_height; }

    operator bool()
    {
        glPopMatrix();
        glfwSwapBuffers(win);
        bool open = !glfwWindowShouldClose(win);
        glfwPollEvents();
        glfwGetFramebufferSize(win, &_width, &_height);
        glClear(GL_COLOR_BUFFER_BIT);
        glViewport(0, 0, _width, _height);
        glPushMatrix();
        glfwGetWindowSize(win, &_width, &_height);
        glOrtho(0, _width, _height, 0, -1, +1);
        return open;
    }

private:
    GLFWwindow* win;
    int _width;
    int _height;
};
