from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.build import check_min_cppstd, build_jobs
from conan.tools.apple import is_apple_os
from conan.tools.microsoft import is_msvc

import os
import sys
import subprocess
import shutil

from pathlib import Path
from contextlib import contextmanager

# Import Basilisk utilities necessary to build itself
from src.utilities import makeDraftModule
from src.architecture.messaging.msgAutoSource import GenCMessages

# XXX: this statement is needed to enable Windows to print ANSI codes in the Terminal
# see https://stackoverflow.com/questions/287871/how-to-print-colored-text-in-terminal-in-python/3332860#3332860
os.system("")

@contextmanager
def chdir(dest):
    """
    Simple chdir helper that automatically returns to the current working directory.
    In Python 3.11+, this is built-in as contextlib.chdir().
    """
    orig_dir = os.getcwd()
    os.chdir(Path(dest).as_posix())
    yield
    os.chdir(orig_dir)

def read_version():
    # Read the version file.
    this_dir = Path(__file__).parent.resolve()
    with (this_dir/'docs/source/bskVersion.txt').open('r') as f:
        version = f.read().strip()
    return version


#==============================================================================
# Basilisk build recipe
# -----------------------------------------------------------------------------
# NOTE: This recipe is used only as a helper for downloading C++ dependencies
# and generating some extra code for Basilisk.
# It is only intended for use with `conan build .`. It is *NOT* intended to
# actually create a Conan package or to be exported!
#==============================================================================

required_conan_version = ">=1.60 <2.0 || >=2.0.5"

class BasiliskRecipe(ConanFile):
    name     = "Basilisk"
    homepage = "http://hanspeterschaub.info/basilisk"
    version  = read_version()
    license  = "ISC"

    # Requirements
    requires = [
        "eigen/3.4.0",
    ]

    # Binary configuration
    package_type = "shared-library"  # TODO: Confirm this is correct.
    settings = "os", "compiler", "build_type", "arch"
    options = {
        # define BSK module option list
        "opNav": [True, False],
        "vizInterface": [True, False],
        "pathToExternalModules": [None, "ANY"],
        "sourceFolder": ["ANY"],
        "buildFolder": ["ANY"],
        "clean": [True, False],
    }
    default_options = {
        "opNav": False,
        "vizInterface": True,
        "pathToExternalModules": None,
        "sourceFolder": "src",
        "buildFolder": "dist3",
        "clean": False,
    }

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "setup.py", "CMakeLists.txt", "src/*", "include/*"

    @property 
    def _is_legacy_one_profile(self): 
        # Returns True if user is running a Conan 1 profile
        return not hasattr(self, "settings_build") 

    def validate(self):
        check_min_cppstd(self, "11")

    def configure(self):
        if self._is_legacy_one_profile:
            # XXX: Backwards-compatibility settings for Conan<2.

            if self.settings.os == "Linux" and self.settings.compiler.libcxx != "libstdc++11":
                # Force the user's profile to use the new CXX11 ABI.
                # NOTE: For convenience only (Conan already provides clear
                # instructions to do this).
                conan_args = ["conan", "profile", "update", "settings.compiler.libcxx=libstdc++11", "default"]
                self.output.warning(f"Updating default Conan profile to use CXX11 ABI.")
                self.output.warning(f"{' '.join(conan_args)}")
                subprocess.check_output(conan_args)

            # Provide some additional generators based on OS.
            if is_apple_os(self):
                self.output.warning("Macos detected! Adding Xcode generator.")
                self.generators = "XcodeDeps", "XcodeToolchain"

            elif is_msvc(self):
                self.output.warning("Windows detected! Adding Visual Studio (MSBuild) generator.")
                self.generators = "MSBuildDeps", "MSBuildToolchain"

        if self.settings.build_type == "Debug":
            self.output.warning("Build type is set to Debug. Performance will be significantly lower.")

        self.externalModulesPath = self.options.get_safe("pathToExternalModules")
        if self.externalModulesPath:
            self.externalModulesPath = Path(str(self.externalModulesPath))
            assert self.externalModulesPath.is_dir(), \
                f"Expected pathToExternalModules to exist as a directory! Got {self.externalModulesPath}"
            self.output.info(f"Building external modules from: {self.externalModulesPath}")

        # Install additional opencv methods
        if self.options.get_safe("opNav"):
            self.options['opencv'].contrib = True
            # Raise an issue to conan-center to fix this bug. Using workaround to disable freetype for windows
            # Issue link: https://github.com/conan-community/community/issues/341
            #TODO Remove this once they fix this issue.
            # TODO: Confirm if still needed.
            if is_msvc(self):
                self.options['opencv'].freetype = False

        if is_msvc(self):
            self.options["*"].shared = True

        # Other dependency options
        self.options['zeromq'].encryption = False # Basilisk does not use data streaming encryption.
        self.options['opencv'].with_ffmpeg = False  # video frame encoding lib
        self.options['opencv'].gapi = False  # graph manipulations framework
        self.options['opencv'].with_tiff = False  # generate image in TIFF format
        self.options['opencv'].with_openexr = False  # generate image in EXR format
        self.options['opencv'].with_quirc = False  # QR code lib
        self.options['opencv'].with_webp = False  # raster graphics file format for web

        if self.options.get_safe("clean"):
            # clean the distribution folder to start fresh (kept here for backwards compatibility)
            # TODO: Fix the CMake build system so we can do proper incremental
            # builds and also avoid having to clean the build directory.
            this_dir = Path(__file__).parent.resolve()
            distPath = this_dir/"dist3"
            shutil.rmtree(distPath, ignore_errors=True)

    def build_requirements(self):
        # Protobuf is also required as a tool (in order for CMake to find the
        # Conan-installed `protoc` compiler).
        # See https://github.com/conan-io/conan-center-index/issues/21737
        # and https://github.com/conan-io/conan-center-index/pull/22244#issuecomment-1910770387
        if not self._is_legacy_one_profile: 
            self.tool_requires("protobuf/<host_version>") 
            # self.tool_requires("cmake/[>=3.14]")  # TODO: As of Feb 2024, cmake recipe doesn't set PATH correctly in Conan 2+.

    def requirements(self):
        if self.options.get_safe("opNav"):
            self.requires("opencv/4.1.2#b610ad323f67adc1b51e402cb5d68d70")
            # TODO: Confirm if we need to specify the requirements below.
            # self.requires("pcre/8.45")
            # self.requires("zlib/1.2.13")
            # self.requires("xz_utils/5.4.5")

        if self.options.get_safe("vizInterface") or self.options.get_safe("opNav"):
            self.requires("protobuf/3.17.1")
            self.requires("cppzmq/4.5.0")

    def generate(self):
        if self.settings.build_type == "Debug":
            self.output.warning("Build type is set to Debug. Performance will be significantly slower.")

        with chdir(self.generators_folder):
            self.output.info("Auto-Generating Draft Modules...")
            genMod = makeDraftModule.moduleGenerator()
            genMod.cleanBuild = True
            genMod.verbose = False
            makeDraftModule.fillCppInfo(genMod)
            genMod.createCppModule()
            makeDraftModule.fillCInfo(genMod)
            genMod.createCModule()

        # Generate all Basilisk message files
        with chdir(self.source_path/"architecture/messaging/msgAutoSource"):
            self.output.info("Auto-Generating Message files...")
            # TODO: Only regenerate files that changed. (Move these auto-generators into CMake!)
            generateMessages = GenCMessages.GenerateMessages(self.externalModulesPath or "", build_folder=self.build_folder)
            generateMessages.initialize()
            generateMessages.run()

        # -------------------------------------------------------------
        # Run the CMake configuration generators.
        # -------------------------------------------------------------
        deps = CMakeDeps(self)
        deps.set_property("eigen", "cmake_target_name", "Eigen3::Eigen3")   # XXX: Override, original is "Eigen3::Eigen"
        deps.set_property("cppzmq", "cmake_target_name", "cppzmq::cppzmq")  # XXX: Override, original is "cppzmq"
        deps.generate()

        tc = CMakeToolchain(self)
        tc.cache_variables["BUILD_OPNAV"] = bool(self.options.opNav)
        tc.cache_variables["BUILD_VIZINTERFACE"] = bool(self.options.vizInterface)
        if self.options.get_safe("pathToExternalModules"):
            tc.cache_variables["EXTERNAL_MODULES_PATH"] = self.externalModulesPath.resolve().as_posix()
        tc.cache_variables["PYTHON_VERSION"] = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
        # Set the build rpath, since we don't install the targets, so that the
        # shared libraries can find each other using relative paths.
        tc.cache_variables["CMAKE_BUILD_RPATH_USE_ORIGIN"] = True
        # Set the minimum buildable MacOS version.
        tc.cache_variables["CMAKE_OSX_DEPLOYMENT_TARGET"] = "10.13"
        tc.parallel = True

        # Generate!
        tc.generate()

    def layout(self):
        cmake_layout(self, src_folder=str(self.options.sourceFolder), build_folder=str(self.options.buildFolder))

        # XXX: Override the build folder again to keep it consistent between
        # multi- (e.g. Visual Studio) and single-config (e.g. Make) generators.
        # Otherwise, it's too difficult to extract the value of this into the
        # setup.py file programmatically.
        self.folders.build  = str(self.options.buildFolder)

    def build(self):
        cmake = CMake(self)
        cmake.configure()

        if self._is_legacy_one_profile and is_apple_os(self):
            # TODO: Confirm if this is necessary?
            cmake.build(build_tool_args=['-jobs', str(build_jobs(self)), '-parallelizeTargets'])
        else:
            cmake.build()
