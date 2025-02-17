import os

from conan import ConanFile
from conan.tools.files import load
from conan.tools.cmake import CMake, cmake_layout

class MujocoReplayConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"

    def requirements(self):
        # mujocoUtils, utilities, src, top-level, libs, mujoco, version.txt
        version_file = os.path.join(self.recipe_folder, "..", "..", "..", "libs", "mujoco", "version.txt")
        mujoco_version = load(self, version_file).strip()

        self.requires("glfw/3.4")
        self.requires(f"mujoco/{mujoco_version}")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def layout(self):
        cmake_layout(self)
