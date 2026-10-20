// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include "window.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// Camera 0 is the origin. Camera i repeats this step:
// p0 = Ry(yaw) * Rx(pitch) * Rz(roll) * pi + t
struct rig_pose
{
    float tx, ty, tz, yaw, pitch, roll;
    rig_pose() : tx(0.47f), ty(0.f), tz(-0.03f), yaw(21.f), pitch(-7.f), roll(-8.f) {}
};

struct view_state
{
    float yaw, pitch, distance;
    bool dragging;
    bool show_options;
    double last_x, last_y;
    view_state() : yaw(16.f), pitch(-6.f), distance(2.6f), dragging(false), show_options(false), last_x(0), last_y(0) {}
};

struct camera_source
{
    std::string serial;
    std::string name;
    rs2::pipeline pipe;
    rs2::pointcloud pc;
    rs2::points points;
    rs2::decimation_filter decimate;
    rs2::spatial_filter spatial;
    rs2::hole_filling_filter hole;
    int grid_w;
    int grid_h;
    bool have_points;

    camera_source(rs2::context& ctx, const std::string& sn, const std::string& device_name)
        : serial(sn), name(device_name), pipe(ctx), grid_w(0), grid_h(0), have_points(false)
    {
        decimate.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.f);
        if (spatial.supports(RS2_OPTION_FILTER_MAGNITUDE))
            spatial.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.f);
        if (spatial.supports(RS2_OPTION_FILTER_SMOOTH_ALPHA))
            spatial.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.6f);
        if (spatial.supports(RS2_OPTION_FILTER_SMOOTH_DELTA))
            spatial.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 24.f);
        if (hole.supports(RS2_OPTION_HOLES_FILL))
            hole.set_option(RS2_OPTION_HOLES_FILL, 2.f);
    }
};

static void print_usage()
{
    std::cout
        << "rs-multicam-mesh [--tx m] [--ty m] [--tz m] [--yaw deg] [--pitch deg] [--roll deg]\n"
        << "                 [--range m] [--decimation 1-8]\n"
        << "\n"
        << "Shaded mesh of whatever is inside --range (default 2.5 m), both cameras.\n"
        << "The view turns on its own. Drag to take over, scroll to zoom.\n"
        << "H shows the align controls. - / = change the range.\n"
        << std::endl;
}

static bool take_arg(int argc, char* argv[], int& i, float& dst)
{
    if (i + 1 >= argc)
        throw std::runtime_error(std::string("missing value for ") + argv[i]);
    dst = std::stof(argv[++i]);
    return true;
}

static rig_pose parse_args(int argc, char* argv[], int& decimation, float& max_range)
{
    rig_pose pose;
    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h")
        {
            print_usage();
            std::exit(EXIT_SUCCESS);
        }
        else if (arg == "--tx") take_arg(argc, argv, i, pose.tx);
        else if (arg == "--ty") take_arg(argc, argv, i, pose.ty);
        else if (arg == "--tz") take_arg(argc, argv, i, pose.tz);
        else if (arg == "--yaw") take_arg(argc, argv, i, pose.yaw);
        else if (arg == "--pitch") take_arg(argc, argv, i, pose.pitch);
        else if (arg == "--roll") take_arg(argc, argv, i, pose.roll);
        else if (arg == "--range") take_arg(argc, argv, i, max_range);
        else if (arg == "--decimation")
        {
            float value = 2.f;
            take_arg(argc, argv, i, value);
            decimation = std::max(1, std::min(8, (int)std::lround(value)));
        }
        else
            throw std::runtime_error("unknown argument: " + arg + " (try --help)");
    }
    return pose;
}

static rs2::video_stream_profile choose_depth_profile(const rs2::device& dev)
{
    rs2::video_stream_profile best;
    int best_score = -1;
    for (auto&& sensor : dev.query_sensors())
    {
        for (auto&& profile : sensor.get_stream_profiles())
        {
            if (profile.stream_type() != RS2_STREAM_DEPTH || profile.format() != RS2_FORMAT_Z16)
                continue;
            auto video = profile.as<rs2::video_stream_profile>();
            int w = video.width();
            int h = video.height();
            int fps = video.fps();
            int score = fps;
            if (w == 640 && h == 480 && fps == 30) score += 100000;
            else if (w == 640 && h == 480) score += 80000;
            else if (w * h <= 640 * 480) score += 40000 - std::abs(w * h - 640 * 480) / 64;
            else score += 10000 - w * h / 1024;
            if (score > best_score)
            {
                best_score = score;
                best = video;
            }
        }
    }
    if (!best)
        throw std::runtime_error("no Z16 depth profile on " + std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)));
    return best;
}

static void start_depth(camera_source& cam, const rs2::device& dev, int decimation)
{
    auto depth = choose_depth_profile(dev);
    cam.decimate.set_option(RS2_OPTION_FILTER_MAGNITUDE, float(decimation));
    rs2::config cfg;
    cfg.enable_device(cam.serial);
    cfg.enable_stream(RS2_STREAM_DEPTH, depth.stream_index(), depth.width(), depth.height(), RS2_FORMAT_Z16, depth.fps());
    cam.pipe.start(cfg);
    std::cout << cam.name << "  " << cam.serial
              << "  depth " << depth.width() << "x" << depth.height() << " @ " << depth.fps()
              << std::endl;
}

struct xform
{
    float r[9];
    float t[3];
};

static xform make_xform(const rig_pose& pose, int index)
{
    const float d2r = 0.01745329251f;
    float roll = pose.roll * d2r * float(index);
    float pitch = pose.pitch * d2r * float(index);
    float yaw = pose.yaw * d2r * float(index);
    float cz = std::cos(roll), sz = std::sin(roll);
    float cx = std::cos(pitch), sx = std::sin(pitch);
    float cy = std::cos(yaw), sy = std::sin(yaw);
    xform xf;
    xf.r[0] = cy * cz + sy * sx * sz;
    xf.r[1] = -cy * sz + sy * sx * cz;
    xf.r[2] = sy * cx;
    xf.r[3] = cx * sz;
    xf.r[4] = cx * cz;
    xf.r[5] = -sx;
    xf.r[6] = -sy * cz + cy * sx * sz;
    xf.r[7] = sy * sz + cy * sx * cz;
    xf.r[8] = cy * cx;
    xf.t[0] = pose.tx * float(index);
    xf.t[1] = pose.ty * float(index);
    xf.t[2] = pose.tz * float(index);
    return xf;
}

static void apply_r(const xform& xf, float x, float y, float z, float out[3])
{
    out[0] = xf.r[0] * x + xf.r[1] * y + xf.r[2] * z;
    out[1] = xf.r[3] * x + xf.r[4] * y + xf.r[5] * z;
    out[2] = xf.r[6] * x + xf.r[7] * y + xf.r[8] * z;
}

static bool usable(const rs2::vertex& v, float max_range)
{
    return v.z >= 0.25f && v.z <= max_range;
}

static bool close_enough(const rs2::vertex& a, const rs2::vertex& b)
{
    float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    if (std::fabs(dz) > 0.07f)
        return false;
    return dx * dx + dy * dy + dz * dz < 0.12f * 0.12f;
}

static void face_normal(const rs2::vertex& a, const rs2::vertex& b, const rs2::vertex& c, float n[3])
{
    float ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    float vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
    n[0] = uy * vz - uz * vy;
    n[1] = uz * vx - ux * vz;
    n[2] = ux * vy - uy * vx;
    if (n[2] > 0.f)
    {
        n[0] = -n[0];
        n[1] = -n[1];
        n[2] = -n[2];
    }
}

static void accumulate(std::vector<float>& acc, int idx, const float n[3])
{
    acc[idx * 3] += n[0];
    acc[idx * 3 + 1] += n[1];
    acc[idx * 3 + 2] += n[2];
}

static void shade_vertex(const xform& xf, const rs2::vertex& v, const float ncam[3], float* dst)
{
    float p[3], n[3];
    apply_r(xf, v.x, v.y, v.z, p);
    p[0] += xf.t[0];
    p[1] += xf.t[1];
    p[2] += xf.t[2];
    apply_r(xf, ncam[0], ncam[1], ncam[2], n);
    float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (len > 1e-6f)
    {
        n[0] /= len;
        n[1] /= len;
        n[2] /= len;
    }
    const float l1[3] = { 0.220f, -0.719f, -0.659f };
    const float l2[3] = { -0.822f, -0.224f, 0.523f };
    float d1 = std::fabs(n[0] * l1[0] + n[1] * l1[1] + n[2] * l1[2]);
    float d2 = std::fabs(n[0] * l2[0] + n[1] * l2[1] + n[2] * l2[2]);
    float shade = std::min(1.f, 0.20f + 0.62f * d1 + 0.30f * d2);
    dst[0] = p[0];
    dst[1] = p[1];
    dst[2] = p[2];
    dst[3] = 0.76f * shade;
    dst[4] = 0.78f * shade;
    dst[5] = 0.82f * shade;
}

static void append_mesh(const camera_source& cam, int camera_index, const rig_pose& pose, float max_range, std::vector<float>& mesh)
{
    if (!cam.have_points || cam.grid_w < 2 || cam.grid_h < 2)
        return;
    xform xf = make_xform(pose, camera_index);
    const rs2::vertex* vertices = cam.points.get_vertices();
    int w = cam.grid_w;
    int h = cam.grid_h;
    if ((int)cam.points.size() != w * h)
        return;

    std::vector<float> acc((size_t)w * h * 3, 0.f);
    std::vector<unsigned char> tri_a;
    std::vector<unsigned char> tri_b;
    tri_a.reserve((size_t)(w - 1) * (h - 1));
    tri_b.reserve(tri_a.capacity());

    for (int y = 0; y < h - 1; ++y)
    {
        for (int x = 0; x < w - 1; ++x)
        {
            int i00 = y * w + x;
            int i10 = i00 + 1;
            int i01 = i00 + w;
            int i11 = i01 + 1;
            const rs2::vertex& v00 = vertices[i00];
            const rs2::vertex& v10 = vertices[i10];
            const rs2::vertex& v01 = vertices[i01];
            const rs2::vertex& v11 = vertices[i11];
            bool a = usable(v00, max_range) && usable(v10, max_range) && usable(v01, max_range)
                && close_enough(v00, v10) && close_enough(v00, v01) && close_enough(v10, v01);
            bool b = usable(v10, max_range) && usable(v01, max_range) && usable(v11, max_range)
                && close_enough(v10, v11) && close_enough(v01, v11) && close_enough(v10, v01);
            tri_a.push_back(a ? 1 : 0);
            tri_b.push_back(b ? 1 : 0);
            float n[3];
            if (a)
            {
                face_normal(v00, v10, v01, n);
                accumulate(acc, i00, n);
                accumulate(acc, i10, n);
                accumulate(acc, i01, n);
            }
            if (b)
            {
                face_normal(v10, v11, v01, n);
                accumulate(acc, i10, n);
                accumulate(acc, i11, n);
                accumulate(acc, i01, n);
            }
        }
    }

    size_t quads = tri_a.size();
    for (size_t q = 0; q < quads; ++q)
    {
        int x = (int)(q % (size_t)(w - 1));
        int y = (int)(q / (size_t)(w - 1));
        int i00 = y * w + x;
        int i10 = i00 + 1;
        int i01 = i00 + w;
        int i11 = i01 + 1;
        int ids_a[3] = { i00, i10, i01 };
        int ids_b[3] = { i10, i11, i01 };
        if (tri_a[q])
        {
            for (int k = 0; k < 3; ++k)
            {
                float dst[6];
                shade_vertex(xf, vertices[ids_a[k]], &acc[ids_a[k] * 3], dst);
                mesh.insert(mesh.end(), dst, dst + 6);
            }
        }
        if (tri_b[q])
        {
            for (int k = 0; k < 3; ++k)
            {
                float dst[6];
                shade_vertex(xf, vertices[ids_b[k]], &acc[ids_b[k] * 3], dst);
                mesh.insert(mesh.end(), dst, dst + 6);
            }
        }
    }
}

static void draw_mesh(float width, float height, const view_state& view, const std::vector<float>& mesh)
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glClearColor(0.015f, 0.016f, 0.02f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluPerspective(38.0, width / height, 0.05, 20.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    const float d2r = 0.01745329251f;
    float yaw = view.yaw * d2r;
    float pitch = view.pitch * d2r;
    float target_y = 0.05f;
    float target_z = 1.35f;
    float dist = view.distance;
    float cp = std::cos(pitch);
    float eye_x = std::sin(yaw) * cp * dist;
    float eye_y = target_y + std::sin(pitch) * dist;
    float eye_z = target_z - std::cos(yaw) * cp * dist;
    gluLookAt(eye_x, eye_y, eye_z, 0.f, target_y, target_z, 0.f, -1.f, 0.f);

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    GLfloat fog_color[4] = { 0.015f, 0.016f, 0.02f, 1.f };
    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogf(GL_FOG_START, dist * 0.55f);
    glFogf(GL_FOG_END, dist + target_z + 0.8f);
    glFogfv(GL_FOG_COLOR, fog_color);

    if (!mesh.empty())
    {
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glVertexPointer(3, GL_FLOAT, 6 * sizeof(float), &mesh[0]);
        glColorPointer(3, GL_FLOAT, 6 * sizeof(float), &mesh[3]);
        glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(mesh.size() / 6));
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopAttrib();
}

static void draw_options(float width, float height, const rig_pose& pose, float max_range, int decimation)
{
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_FOG);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, width, height, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor3f(0.85f, 0.85f, 0.85f);
    std::ostringstream line;
    line.setf(std::ios::fixed);
    line.precision(3);
    line << "range " << max_range << " m   tx " << pose.tx << "  tz " << pose.tz
         << "  yaw " << pose.yaw << "  pitch " << pose.pitch << "  roll " << pose.roll
         << "   decimation " << decimation;
    draw_text(16, 24, line.str().c_str());
    draw_text(16, 44, "A/D tx   W/S tz   I/K ty   Q/E yaw   T/G pitch   Z/X roll   -/= range   [ ] detail   H hide");
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

static bool adjust_pose(int key, rig_pose& pose, float& max_range, int& decimation, std::vector<camera_source>& cameras)
{
    const float step_m = 0.01f;
    const float step_deg = 1.f;
    if (key == GLFW_KEY_A) pose.tx -= step_m;
    else if (key == GLFW_KEY_D) pose.tx += step_m;
    else if (key == GLFW_KEY_W) pose.tz -= step_m;
    else if (key == GLFW_KEY_S) pose.tz += step_m;
    else if (key == GLFW_KEY_I) pose.ty -= step_m;
    else if (key == GLFW_KEY_K) pose.ty += step_m;
    else if (key == GLFW_KEY_Q) pose.yaw -= step_deg;
    else if (key == GLFW_KEY_E) pose.yaw += step_deg;
    else if (key == GLFW_KEY_T) pose.pitch -= step_deg;
    else if (key == GLFW_KEY_G) pose.pitch += step_deg;
    else if (key == GLFW_KEY_Z) pose.roll -= step_deg;
    else if (key == GLFW_KEY_X) pose.roll += step_deg;
    else if (key == GLFW_KEY_MINUS) max_range = std::max(0.5f, max_range - 0.1f);
    else if (key == GLFW_KEY_EQUAL) max_range = std::min(6.f, max_range + 0.1f);
    else if (key == GLFW_KEY_0)
        pose = rig_pose();
    else if (key == GLFW_KEY_LEFT_BRACKET || key == GLFW_KEY_RIGHT_BRACKET)
    {
        decimation += (key == GLFW_KEY_RIGHT_BRACKET) ? 1 : -1;
        decimation = std::max(1, std::min(8, decimation));
        for (size_t i = 0; i < cameras.size(); ++i)
            cameras[i].decimate.set_option(RS2_OPTION_FILTER_MAGNITUDE, float(decimation));
    }
    else
        return false;

    std::cout.setf(std::ios::fixed);
    std::cout.precision(3);
    std::cout << "tx=" << pose.tx << " ty=" << pose.ty << " tz=" << pose.tz
              << " yaw=" << pose.yaw << " pitch=" << pose.pitch << " roll=" << pose.roll
              << " range=" << max_range << " decimation=" << decimation << std::endl;
    return true;
}

int main(int argc, char* argv[]) try
{
    int decimation = 2;
    float max_range = 2.5f;
    rig_pose pose = parse_args(argc, argv, decimation, max_range);
    print_usage();

    rs2::context ctx;
    std::vector<std::pair<std::string, std::string> > found;
    for (auto&& dev : ctx.query_devices())
    {
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::string name = dev.supports(RS2_CAMERA_INFO_NAME) ? dev.get_info(RS2_CAMERA_INFO_NAME) : "RealSense";
        found.push_back(std::make_pair(serial, name));
    }
    std::sort(found.begin(), found.end());
    if (found.size() < 2)
        throw std::runtime_error("need two depth cameras, found " + std::to_string(found.size()));

    std::vector<camera_source> cameras;
    cameras.reserve(found.size());
    for (size_t i = 0; i < found.size(); ++i)
    {
        if (i)
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        cameras.emplace_back(ctx, found[i].first, found[i].second);
        rs2::device dev;
        for (auto&& candidate : ctx.query_devices())
        {
            if (candidate.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == found[i].first)
                dev = candidate;
        }
        start_depth(cameras.back(), dev, decimation);
    }

    window app(1280, 720, "Dual Depth");
    view_state view;
    std::vector<float> mesh;
    mesh.reserve(400000);

    app.on_left_mouse = [&](bool pressed) { view.dragging = pressed; };
    app.on_mouse_scroll = [&](double, double yoffset)
    {
        view.distance -= (float)yoffset * 0.18f;
        view.distance = std::max(0.7f, std::min(view.distance, 7.f));
    };
    app.on_mouse_move = [&](double x, double y)
    {
        if (view.dragging)
        {
            view.yaw -= (float)(x - view.last_x) * 0.35f;
            view.pitch += (float)(y - view.last_y) * 0.25f;
            view.pitch = std::max(-35.f, std::min(view.pitch, 35.f));
        }
        view.last_x = x;
        view.last_y = y;
    };
    app.on_key_release = [&](int key)
    {
        if (key == GLFW_KEY_H)
            view.show_options = !view.show_options;
        else if (key == GLFW_KEY_SPACE)
        {
            view.yaw = 16.f;
            view.pitch = -6.f;
            view.distance = 2.6f;
        }
        else if (view.show_options)
            adjust_pose(key, pose, max_range, decimation, cameras);
        else if (key == GLFW_KEY_MINUS || key == GLFW_KEY_EQUAL)
            adjust_pose(key, pose, max_range, decimation, cameras);
    };

    auto last = std::chrono::steady_clock::now();
    while (app)
    {
        bool updated = false;
        for (size_t i = 0; i < cameras.size(); ++i)
        {
            rs2::frameset frames;
            if (!cameras[i].pipe.poll_for_frames(&frames))
                continue;
            rs2::depth_frame depth = frames.get_depth_frame();
            if (!depth)
                continue;
            rs2::frame frame = depth;
            frame = cameras[i].decimate.process(frame);
            frame = cameras[i].spatial.process(frame);
            frame = cameras[i].hole.process(frame);
            rs2::video_frame video = frame.as<rs2::video_frame>();
            cameras[i].grid_w = video.get_width();
            cameras[i].grid_h = video.get_height();
            cameras[i].points = cameras[i].pc.calculate(frame);
            cameras[i].have_points = true;
            updated = true;
        }
        if (updated)
        {
            mesh.clear();
            for (size_t i = 0; i < cameras.size(); ++i)
                append_mesh(cameras[i], (int)i, pose, max_range, mesh);
        }

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last).count();
        last = now;
        if (dt > 0.1f)
            dt = 0.1f;
        if (!view.dragging && !view.show_options)
            view.yaw += 10.f * dt;

        draw_mesh(app.width(), app.height(), view, mesh);
        if (view.show_options)
            draw_options(app.width(), app.height(), pose, max_range, decimation);
    }
    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
