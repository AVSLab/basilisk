/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

/*
This is a simple utility that takes a MuJoCo model file and a series of 'qpos' generated during a
simulation and visualizes the result. The utility is useful for debugging and understanding the
simulation results.

This is designed as a standalone utility, compiled separately from Basilisk.
This utility is not necessary for regular Basilisk operation, even if using MuJoCo
*/

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "CLI11.hpp"

// MuJoCo data structures
mjModel* m = NULL; // MuJoCo model
mjData* d = NULL;  // MuJoCo data
mjVFS vfs;         // MuJoCo Virtual File System
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouseButton(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouseMove(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

std::vector<std::vector<double>> readNumericFile(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        exit(1);
    }

    std::vector<std::vector<double>> data;
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<double> row;
        double value;

        while (iss >> value) {
            row.push_back(value);
        }

        data.push_back(row);
    }

    file.close();
    return data;
}

bool endsWith(std::string_view str, std::string_view suffix)
{
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void interpolateQpos(const mjModel* m,
                     double* qpos,
                     double t,
                     const std::vector<double>& left,
                     const std::vector<double>& right)
{
    auto tMin = left.at(0) / 1e9;
    auto tMax = right.at(0) / 1e9;
    auto scaledT = (t - tMin) / (tMax - tMin);

    auto lerpAt = [&](size_t qposIndex) {
        qpos[qposIndex] =
            left.at(qposIndex + 1) * (1 - scaledT) + right.at(qposIndex + 1) * scaledT;
    };

    auto slerpAt = [&left, &right, scaledT, qpos](size_t qposIndex) {
        const double* q1 = left.data() + qposIndex + 1;
        const double* q2 = right.data() + qposIndex + 1;
        double* result = qpos + qposIndex;

        double cosHalfTheta = 0;
        for (size_t i = 0; i < 4; i++)
            cosHalfTheta += q1[i] * q2[i];

        auto inverse = cosHalfTheta >= 0 ? 1 : -1;
        cosHalfTheta *= inverse;

        // if q1=q2 or q1=-q2 then theta = 0 and we can return q1
        if (cosHalfTheta >= 1.0) {
            std::copy_n(q1, 4, result);
            return;
        }

        double halfTheta = acos(cosHalfTheta);
        double sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta);

        double ratioA = sin((1 - scaledT) * halfTheta) / sinHalfTheta;
        double ratioB = sin(scaledT * halfTheta) / sinHalfTheta;

        // If theta = 180 degrees then result is not fully defined
        // We could rotate around any axis normal to q1 or q2
        if (std::abs(sinHalfTheta) < 1e-4) {
            ratioA = 0.5;
            ratioB = 0.5;
        }

        for (size_t i = 0; i < 4; i++)
            result[i] = q1[i] * ratioA + inverse * q2[i] * ratioB;
    };

    for (int j = 0; j < m->njnt; j++) {
        // get addresses in qpos
        int padr = m->jnt_qposadr[j];

        switch ((mjtJoint)m->jnt_type[j]) {
        case mjJNT_FREE:
            // position interpolate
            for (int i = 0; i < 3; i++) {
                lerpAt(padr + i);
            }
            // quaternion interpolate
            slerpAt(padr + 3);
            break;

        case mjJNT_BALL:
            // quaternion interpolate
            slerpAt(padr);
            break;

        case mjJNT_HINGE:
        case mjJNT_SLIDE:
            // scalar interpolation
            lerpAt(padr);
        }
    }
}

struct cli {
    std::string modelFileName;
    std::string stateFileName;
    double speedUp = 1;
    std::string trackingBodyName;
    std::vector<std::string> extraFiles;

    static cli parse(int argc, char** argv)
    {
        cli cli;

        CLI::App app{"'replay' takes a MuJoCo model file and a series 'qpos' generated during a "
                     "simulation and visualizes the result."};
        argv = app.ensure_utf8(argv);

        app.add_option("-m,--model", cli.modelFileName, "Path to MuJoCo model file.");

        app.add_option("-s,--state",
                       cli.stateFileName,
                       "Path to file that contains the state evolution in time.");

        app.add_option(
            "--speed",
            cli.speedUp,
            "Factor with which to speed up simulation replay. =1 is real time, =2 is double "
            "speed, =0.2 is five times slower, etc.");

        app.add_option("--track",
                       cli.trackingBodyName,
                       "Name of the body to track during visualization, by default, the first free "
                       "body in the simulation. If 'none', camera is moved freely by user.");

        app.add_option("--file",
                       cli.extraFiles,
                       "Path to extra file to expose to MuJoCo, for example for a mesh.");

        try {
            app.parse(argc, argv);
        } catch (const CLI::ParseError& e) {
            std::exit(app.exit(e));
        }

        if (cli.speedUp <= 0) {
            cli.speedUp = std::numeric_limits<double>::max();
        }

        return cli;
    }
};

// main function
int main(int argc, char** argv)
{
    auto cli = cli::parse(argc, argv);

    mj_defaultVFS(&vfs);

    for (auto&& file : cli.extraFiles) {
        switch (mj_addFileVFS(&vfs, "", file.c_str())) {
        case 0:
            break; // success
        case 1:
            std::cerr << "Error loading file " + file + ": VFS memory is full.";
            return 1;
        case 2:
            std::cerr << "Error loading file " + file + ": file name used multiple times.";
            return 1;
        case -1:
            std::cerr << "Error loading file " + file + ": internal error.";
            return 1;
        default:
            std::cerr << "Error loading file " + file + ".";
            return 1; // should never happen
        };
    }

    // load and compile model
    char error[1000] = "Could not load binary model";
    if (endsWith(cli.modelFileName, ".mjb")) {
        m = mj_loadModel(cli.modelFileName.c_str(), &vfs);
    } else {
        m = mj_loadXML(cli.modelFileName.c_str(), &vfs, error, 1000);
    }
    if (!m) {
        std::cerr << "Load model error: " << error;
        return 1;
    }

    int bodyId = 1;
    if (cli.trackingBodyName == "none") {
        bodyId = 0;
    } else if (std::size(cli.trackingBodyName) > 0) {
        bodyId = mj_name2id(m, mjOBJ_BODY, cli.trackingBodyName.c_str());

        if (bodyId < 0) {
            std::cerr << "No body found with name: " << cli.trackingBodyName;
            return 1;
        }
    }

    // load states
    auto qpos = readNumericFile(cli.stateFileName);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW";
        return 1;
    }

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // Make cam tracking
    if (bodyId > 0) {
        cam.type = mjCAMERA_TRACKING;
        cam.trackbodyid = bodyId;
        cam.fixedcamid = -1;
    }

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouseMove);
    glfwSetMouseButtonCallback(window, mouseButton);
    glfwSetScrollCallback(window, scroll);

    double t0 = qpos.at(0).at(0) / 1e9;
    auto start = std::chrono::steady_clock::now();

    size_t i = 0;

    auto secondsAtWhichToRender = t0;

    // run main loop, target real-time simulation and 60 fps rendering
    while (!glfwWindowShouldClose(window)) {

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        if (i < qpos.size()) {
            auto nanosSinceStart = std::chrono::steady_clock::now() - start;
            auto effectiveSecondsSinceStart = (cli.speedUp / 1e9) * nanosSinceStart.count();

            secondsAtWhichToRender = t0 + effectiveSecondsSinceStart;

            // Advance i until qpos.at(i).at(0) < secondsAtWhichToRender < qpos.at(i+1).at(0)
            // or i = qpos.size() - 1
            while (i + 1 < qpos.size() && (qpos.at(i + 1).at(0) / 1e9) <= secondsAtWhichToRender) {
                i++;
            }

            if (i == qpos.size() - 1) {
                std::copy_n(qpos.at(i).data() + 1, m->nq, d->qpos);
            } else {
                interpolateQpos(m, d->qpos, secondsAtWhichToRender, qpos.at(i), qpos.at(i + 1));
            }

            mj_forward(m, d);
        }

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        std::ostringstream stamp;
        stamp << "Time = " << std::fixed << std::setprecision(3) << secondsAtWhichToRender << " s";
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, stamp.str().c_str(), NULL, &con);

        mjr_overlay(mjFONT_NORMAL,
                    mjGRID_BOTTOMRIGHT,
                    viewport,
                    "BASILISK\nAVS LAB\nCU BOULDER",
                    NULL,
                    &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deleteVFS(&vfs);

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 0;
}
