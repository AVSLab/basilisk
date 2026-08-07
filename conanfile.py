import argparse
import os
import json
import shutil
import shlex
import subprocess
import sys
from datetime import datetime
from glob import glob
from pathlib import Path
from typing import Callable, Optional

import importlib.metadata

from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.microsoft import is_msvc
from conan.tools.files import copy

sys.path.insert(1, './src/utilities/')
import makeDraftModule

# XXX: this statement is needed to enable Windows to print ANSI codes in the Terminal
# see https://stackoverflow.com/questions/287871/how-to-print-colored-text-in-terminal-in-python/3332860#3332860
os.system("")

# define the print color codes
statusColor = '\033[92m'
failColor = '\033[91m'
warningColor = '\033[93m'
endColor = '\033[0m'


# # define BSK module option list (option name and default value)
bskModuleOptionsBool = {
    "opNav": [[True, False], False],
    "vizInterface": [[True, False], True],
    "mujoco": [[True, False], False],
    # Experimental: build Rust modules under src/ (see
    # src/cmake/bskFindRustModules.cmake and
    # docs/source/Learn/makingModules/rustModules.rst).
    # Requires a Rust/Cargo toolchain on PATH; off by default so most builds/CI
    # jobs never need one.
    "rustModules": [[True, False], False],
    # Enable the opt-in compiler warning policy defined by BSK_STRICT_WARNINGS
    # in src/CMakeLists.txt.
    "strictWarnings": [[True, False], False],
    "buildTesting": [[True, False], True],
    "buildProject": [[True, False], True],
    "recorderPropertyRollback": [[True, False], False],
}
bskModuleOptionsString = {
    "pathToExternalModules": [["ANY"], ""],
    "pyLimitedAPI": [["ANY"], ""],
}
bskModuleOptionsFlag = {
    "clean": [[True, False], False],
}

required_conan_version = ">=2.0.5"

PY_LIMITED_API_PY39  = "0x03090000"  # cp39-abi3
NUMBA_CACHE_SEARCH_DIRS = ("src", "docs/source", "examples")
NUMBA_CACHE_DIR_NAME = "__pycache__"
NUMBA_CACHE_FILE_PATTERNS = ("*.nbc", "*.nbi")


def get_basilisk_numba_model_cache_dir() -> Optional[Path]:
    """Return Basilisk's generated Numba model cache directory."""
    try:
        from platformdirs import user_cache_dir
    except ImportError:
        return None
    return Path(user_cache_dir("basilisk")) / "numba_model"


def clean_numba_cache_artifacts(root: Optional[Path] = None,
                                user_cache_dir: Optional[Path] = None,
                                print_fn: Optional[Callable[[str], None]] = print) -> tuple[int, bool]:
    """Remove Numba disk-cache artifacts that can outlive a clean build."""
    root_path = Path.cwd() if root is None else Path(root)
    removed_cache_files = 0

    for search_dir in NUMBA_CACHE_SEARCH_DIRS:
        search_path = root_path / search_dir
        if not search_path.exists():
            continue
        for cache_dir in search_path.rglob(NUMBA_CACHE_DIR_NAME):
            if not cache_dir.is_dir():
                continue
            for pattern in NUMBA_CACHE_FILE_PATTERNS:
                for cache_file in cache_dir.glob(pattern):
                    try:
                        cache_file.unlink()
                        removed_cache_files += 1
                    except FileNotFoundError:
                        continue

    cache_dir = (
        get_basilisk_numba_model_cache_dir()
        if user_cache_dir is None
        else Path(user_cache_dir)
    )
    cache_dir_existed = False
    removed_user_cache = False
    if cache_dir is not None and cache_dir.exists():
        cache_dir_existed = True
        shutil.rmtree(cache_dir, ignore_errors=True)
        removed_user_cache = not cache_dir.exists()

    if print_fn is not None:
        print_fn(f"Removed {removed_cache_files} in-repo Numba cache file(s).")
        if cache_dir is None:
            print_fn("Basilisk Numba model cache path unavailable; platformdirs is not installed.")
        elif removed_user_cache:
            print_fn(f"Removed Basilisk Numba model cache: {cache_dir}")
        elif cache_dir_existed:
            print_fn(f"Unable to fully remove Basilisk Numba model cache: {cache_dir}")
        else:
            print_fn(f"No Basilisk Numba model cache found at: {cache_dir}")

    return removed_cache_files, removed_user_cache


def clean_rust_target_artifacts(root: Optional[Path] = None,
                                print_fn: Optional[Callable[[str], None]] = print) -> int:
    """Remove Cargo ``target`` directories belonging to crates under ``src``."""
    root_path = Path.cwd() if root is None else Path(root)
    source_path = root_path / "src"
    if not source_path.exists():
        return 0

    manifests = [
        manifest
        for manifest in source_path.rglob("Cargo.toml")
        if "target" not in manifest.relative_to(source_path).parts
    ]
    target_directories = sorted({manifest.parent / "target" for manifest in manifests})
    removed_target_directories = 0
    for target_directory in target_directories:
        if not target_directory.is_dir() or target_directory.is_symlink():
            continue
        shutil.rmtree(target_directory, ignore_errors=True)
        if not target_directory.exists():
            removed_target_directories += 1

    if print_fn is not None:
        directory_label = "directory" if removed_target_directories == 1 else "directories"
        print_fn(f"Removed {removed_target_directories} Cargo target {directory_label}.")

    return removed_target_directories


def should_scan_windows_dll_directory(root: str, build_folder: str) -> bool:
    """Return whether a build directory can contain runtime DLLs.

    Cargo's target tree contains build-time procedural-macro DLLs. They are
    loaded only by ``rustc`` and must not be copied into the Basilisk wheel.
    """
    normalized_root = os.path.normcase(os.path.abspath(root))
    excluded_roots = (
        os.path.normcase(os.path.abspath(os.path.join(build_folder, "Basilisk"))),
        os.path.normcase(os.path.abspath(os.path.join(build_folder, "cargo"))),
    )
    for excluded_root in excluded_roots:
        try:
            if os.path.commonpath([normalized_root, excluded_root]) == excluded_root:
                return False
        except ValueError:
            continue
    return True


def resolve_py_limited_api(opt_value: Optional[str]) -> str:
    """Use explicit --pyLimitedAPI if provided, else cp39."""
    if opt_value:
        return opt_value
    return PY_LIMITED_API_PY39


def read_cached_cmake_generator(build_folder: Path) -> Optional[str]:
    """Return the generator recorded in an existing CMake build directory."""
    cache_path = Path(build_folder) / "CMakeCache.txt"
    try:
        cache_lines = cache_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except FileNotFoundError:
        return None

    cache_key = "CMAKE_GENERATOR:INTERNAL="
    for line in cache_lines:
        if line.startswith(cache_key):
            return line[len(cache_key):].strip() or None
    return None


def select_cmake_generator(
        requested_generator: Optional[str],
        build_folder: Path,
        operating_system: str,
        build_project: bool,
        ninja_finder: Callable[[str], Optional[str]] = shutil.which,
) -> tuple[str, str]:
    """Select a CMake generator without changing an existing build directory."""
    cached_generator = read_cached_cmake_generator(build_folder)
    if requested_generator:
        if cached_generator and requested_generator != cached_generator:
            raise ValueError(
                f"The existing CMake build directory '{build_folder}' uses "
                f"the '{cached_generator}' generator, but '{requested_generator}' "
                "was requested. Clean the build directory before switching generators "
                "(use --clean with conanfile.py or -o \"&:clean=True\" with direct Conan)."
            )
        return requested_generator, "explicitly requested"

    if cached_generator:
        return cached_generator, "reused from the existing build directory"

    if not build_project:
        if operating_system == "Windows":
            return "Visual Studio 17 2022", "default Windows IDE project"
        if operating_system == "Macos":
            return "Xcode", "default macOS IDE project"
    if ninja_finder("ninja"):
        return "Ninja", "ninja executable found"
    if operating_system == "Windows":
        return "Visual Studio 17 2022", "ninja executable not found"
    return "Unix Makefiles", "ninja executable not found"


def create_conan_build_command(
        arguments: argparse.Namespace,
        platform_name: str = os.name,
) -> list[str]:
    """Create the single Conan command used to install dependencies and build Basilisk."""
    command = [
        sys.executable,
        "-m",
        "conans.conan",
        "build",
        ".",
        "--build=missing",
        "-s",
        "build_type=" + str(arguments.buildType),
        "-s",
        "compiler.cppstd=17",
        "-s:b",
        "compiler.cppstd=17",
    ]
    if platform_name != "nt":
        command.extend(["-s", "compiler.cstd=gnu17"])
    if arguments.generator:
        command.extend(["-o", "&:generator=" + str(arguments.generator)])

    argument_values = vars(arguments)
    for option_name in bskModuleOptionsBool:
        command.extend(["-o", f"&:{option_name}={argument_values[option_name]}"])

    for option_name in bskModuleOptionsString:
        option_value = str(argument_values[option_name])
        if not option_value:
            continue
        if option_name == "pathToExternalModules":
            external_path = os.path.abspath(option_value.rstrip(os.path.sep))
            if not os.path.exists(external_path):
                raise ValueError(f"path {option_value} does not exist")
            option_value = external_path
        command.extend(["-o", f"&:{option_name}={option_value}"])

    for option_name in bskModuleOptionsFlag:
        if argument_values[option_name]:
            command.extend(["-o", f"&:{option_name}=True"])

    return command


class BasiliskConan(ConanFile):
    name = "Basilisk"
    homepage = "https://avslab.github.io/basilisk/"
    f = open('docs/source/bskVersion.txt', 'r')
    version = f.read()
    f.close()
    # generators = "CMakeDeps"
    settings = "os", "compiler", "build_type", "arch"
    build_policy = "missing"
    license = "ISC"

    # Requirements
    requires = [
        "eigen/3.4.0",
        "cspice/0067",
        "cfitsio/4.6.3",
    ]
    package_type = "shared-library"
    options = {
        # define BSK module option list
        "sourceFolder": ["ANY"],
        "buildFolder": ["ANY"],
        "generator": ["ANY"],
    }
    default_options = {
        "sourceFolder": "src",
        "buildFolder": "dist3",
        "generator": "",
    }

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "setup.py", "CMakeLists.txt", "src/*", "include/*"

    for opt, value in bskModuleOptionsBool.items():
        options.update({opt: value[0]})
        default_options.update({opt: value[1]})
    for opt, value in bskModuleOptionsString.items():
        options.update({opt: value[0]})
        default_options.update({opt: value[1]})
    for opt, value in bskModuleOptionsFlag.items():
        options.update({opt: value[0]})
        default_options.update({opt: value[1]})

    def build_requirements(self):
        # Protobuf is also required as a tool (in order for CMake to find the
        # Conan-installed `protoc` compiler).
        # See https://github.com/conan-io/conan-center-index/issues/21737
        # and https://github.com/conan-io/conan-center-index/pull/22244#issuecomment-1910770387
        if self.options.get_safe("vizInterface") or self.options.get_safe("opNav"):
            self.tool_requires("protobuf/<host_version>")

    def requirements(self):
        if self.options.get_safe("opNav"):
            self.requires("opencv/4.13.0")

        if self.options.get_safe("vizInterface") or self.options.get_safe("opNav"):
            self.requires("protobuf/3.21.12") # For compatibility with openCV
            self.requires("cppzmq/4.11.0")

        if self.options.get_safe("mujoco"):
            self.requires(f"mujoco/{get_mujoco_version()}")

    def configure(self):
        if self.options.get_safe("clean"):
            # clean the distribution folder to start fresh
            root = os.path.abspath(os.path.curdir)
            distPath = os.path.join(root, "dist3")
            if os.path.exists(distPath):
                shutil.rmtree(distPath, ignore_errors=True)
            clean_numba_cache_artifacts(Path(root))
            clean_rust_target_artifacts(Path(root))
        if self.settings.get_safe("build_type") == "Debug":
            print(warningColor + "Build type is set to Debug. Performance will be significantly lower." + endColor)

        # Install additional opencv methods
        if self.options.get_safe("opNav"):
            self.options['opencv'].contrib = True
            self.options['opencv'].with_ffmpeg = False  # video frame encoding lib
            self.options['opencv'].gapi = False  # graph manipulations framework
            self.options['opencv'].with_tiff = False  # generate image in TIFF format
            self.options['opencv'].with_openexr = False  # generate image in EXR format
            self.options['opencv'].with_quirc = False  # QR code lib
            self.options['opencv'].with_webp = False  # raster graphics file format for web
            if self.settings.get_safe("os") == "Linux":
                self.options['opencv'].with_wayland = False  # desktop display protocol

        # Other dependency options
        if self.options.get_safe("vizInterface") or self.options.get_safe("opNav"):
            if self.settings.get_safe("os") == "Macos":
                # Avoid loading separate protobuf runtimes through OpenCV and vizInterface.
                self.options["protobuf"].shared = True
            self.options['zeromq'].encryption = False # Basilisk does not use data streaming encryption.


    def package_id(self):
        if self.info.settings.compiler == "Visual Studio":
            if "MD" in self.settings.compiler.runtime:
                self.info.settings.compiler.runtime = "MD/MDd"
            else:
                self.info.settings.compiler.runtime = "MT/MTd"

    def layout(self):
        cmake_layout(self,
                     src_folder=str(self.options.get_safe("sourceFolder")),
                     build_folder=str(self.options.get_safe("buildFolder"))
                     )

        # XXX: Override the build folder again to keep it consistent between
        # multi- (e.g. Visual Studio) and single-config (e.g. Make) generators.
        # Otherwise, it's too difficult to extract the value of this into the
        # setup.py file programmatically.
        self.folders.build = str(self.options.get_safe("buildFolder"))

    def generate(self):
        if self.settings.os == "Windows":
            # Ensure dependent DLLs are copied into the Basilisk package
            # directory inside the build folder so they can be discovered by
            # packaging tools (delvewheel) and included in wheels.
            basilisk_dst = os.path.join(self.build_folder, "Basilisk")
            for dep in self.dependencies.values():
                for bindir in dep.cpp_info.bindirs:
                    copy(self, "*.dll", bindir, basilisk_dst)
        if self.settings.os == "Windows":
            for dep in self.dependencies.values():
                for libdir in dep.cpp_info.bindirs:
                    copy(self, "*.dll", libdir, "../Basilisk")

        if self.options.get_safe("pathToExternalModules"):
            print(statusColor + "Including External Folder: " + endColor + str(self.options.pathToExternalModules))

        if self.settings.build_type == "Debug":
            self.output.warning("Build type is set to Debug. Performance will be significantly slower.")

        # -------------------------------------------------------------
        # Run the CMake configuration generators.
        # -------------------------------------------------------------
        deps = CMakeDeps(self)
        deps.set_property("eigen", "cmake_target_name", "Eigen3::Eigen3")   # XXX: Override, original is "Eigen3::Eigen"
        deps.set_property("cppzmq", "cmake_target_name", "cppzmq::cppzmq")  # XXX: Override, original is "cppzmq"
        deps.generate()

        tc = CMakeToolchain(self)
        generatorString, generatorReason = select_cmake_generator(
            requested_generator=str(self.options.get_safe("generator") or ""),
            build_folder=Path(self.build_folder),
            operating_system=str(self.settings.os),
            build_project=bool(self.options.get_safe("buildProject")),
        )
        tc.generator = generatorString
        if self.settings.os == "Windows":
            self.options["*"].shared = True
        print(
            "cmake generator set to: " + statusColor + generatorString + endColor
            + f" ({generatorReason})"
        )

        tc.cache_variables["BUILD_OPNAV"] = bool(self.options.get_safe("opNav"))
        tc.cache_variables["BUILD_VIZINTERFACE"] = bool(self.options.get_safe("vizInterface"))
        tc.cache_variables["BUILD_MUJOCO"] = bool(self.options.get_safe("mujoco"))
        tc.cache_variables["BUILD_RUST_MODULES"] = bool(self.options.get_safe("rustModules"))
        tc.cache_variables["BSK_STRICT_WARNINGS"] = bool(self.options.get_safe("strictWarnings"))
        tc.cache_variables["BUILD_TESTING"] = bool(self.options.get_safe("buildTesting"))
        tc.cache_variables["BSK_CONAN_BUILD_TYPE"] = str(self.settings.build_type)
        tc.cache_variables["BSK_VERSION"] = str(self.version).strip()
        tc.cache_variables["BSK_CONAN_VERSION"] = importlib.metadata.version("conan")
        tc.cache_variables["BSK_CONAN_CXX_STANDARD"] = str(self.settings.get_safe("compiler.cppstd") or "")
        tc.cache_variables["BSK_CONAN_CXX_STANDARD_LIBRARY"] = str(self.settings.get_safe("compiler.libcxx") or "")
        tc.cache_variables["BSK_CONAN_COMPILER_RUNTIME"] = str(self.settings.get_safe("compiler.runtime") or "")
        tc.cache_variables["BSK_CONAN_COMPILER_RUNTIME_TYPE"] = str(
            self.settings.get_safe("compiler.runtime_type") or ""
        )
        tc.cache_variables["Python3_EXECUTABLE"] = Path(sys.executable).as_posix()
        if self.options.get_safe("pathToExternalModules"):
            tc.cache_variables["EXTERNAL_MODULES_PATH"] = Path(str(self.options.pathToExternalModules)).resolve().as_posix()
        tc.cache_variables["PYTHON_VERSION"] = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
        tc.cache_variables["RECORDER_PROPERTY_ROLLBACK"] = "1" if self.options.get_safe("recorderPropertyRollback") else "0"

        # get the header directory for numpy
        import numpy
        tc.cache_variables["NUMPY_INCLUDE_DIR"] = numpy.get_include()

        # Set the build rpath, since we don't install the targets, so that the
        # shared libraries can find each other using relative paths.
        tc.cache_variables["CMAKE_BUILD_RPATH_USE_ORIGIN"] = True
        # Set the minimum buildable MacOS version.
        # tc.cache_variables["CMAKE_OSX_DEPLOYMENT_TARGET"] = "10.13"
        tc.parallel = True

        # Use sccache as a compiler launcher if available (e.g. in CI via CMAKE_C/CXX_COMPILER_LAUNCHER env vars)
        if c_launcher := os.environ.get("CMAKE_C_COMPILER_LAUNCHER"):
            tc.cache_variables["CMAKE_C_COMPILER_LAUNCHER"] = c_launcher
        if cxx_launcher := os.environ.get("CMAKE_CXX_COMPILER_LAUNCHER"):
            tc.cache_variables["CMAKE_CXX_COMPILER_LAUNCHER"] = cxx_launcher

        py_limited = resolve_py_limited_api(self.options.get_safe("pyLimitedAPI"))
        tc.cache_variables["PY_LIMITED_API"] = py_limited
        print(f"{statusColor}PY_LIMITED_API={py_limited}{endColor}")

        # Generate!
        tc.generate()

    def build(self):

        cmake = CMake(self)
        print(statusColor + "Configuring cmake..." + endColor)
        cmake.configure()

        if self.options.get_safe("buildProject"):
            print(statusColor + "\nCompiling Basilisk..." + endColor)
            start = datetime.now()
            cmake.build()
            print("Total Build Time: " + str(datetime.now() - start))
            print(f"{statusColor}The Basilisk build is successful and the scripts are ready to run{endColor}")
            # On Windows, copy project-built DLLs next to the Python extension modules
            # so they are bundled in the wheel and resolvable at runtime without PATH tweaks.
            if self.settings.os == "Windows":
                basilisk_dst_root = os.path.join(self.build_folder, "Basilisk")
                common_srcs = [
                    os.path.join(self.build_folder, "bin"),
                    os.path.join(self.build_folder, "Release"),
                    os.path.join(self.build_folder, "Debug"),
                ]
                for src in common_srcs:
                    if os.path.isdir(src):
                        try:
                            copy(self, "*.dll", src, basilisk_dst_root)
                        except Exception as e:
                            self.output.warning(f"Failed to copy DLLs from {src}: {e}")

                # As a fallback, scan the build tree for any remaining DLLs.
                for root, dirs, files in os.walk(self.build_folder):
                    # Skip the destination and Cargo's build-only proc-macro DLLs.
                    if not should_scan_windows_dll_directory(root, self.build_folder):
                        dirs.clear()
                        continue
                    if any(f.lower().endswith(".dll") for f in files):
                        try:
                            copy(self, "*.dll", root, basilisk_dst_root)
                        except Exception as e:
                            self.output.warning(f"Failed to copy DLLs from {root}: {e}")

                # Rename DLLs to lowercase
                for path in glob(os.path.join(basilisk_dst_root, "*.dll")):
                    base = os.path.basename(path)
                    lower = base.lower()
                    if base != lower:
                        tmp = os.path.join(basilisk_dst_root, f".{lower}.tmp")
                        os.replace(path, tmp)
                        os.replace(tmp, os.path.join(basilisk_dst_root, lower))
        else:
            print(f"{statusColor}Finished configuring the Basilisk project.{endColor}")
            print(
                f"{statusColor}Build it later with `cmake --build {self.build_folder} "
                f"--config {self.settings.build_type} --parallel <number of threads to use>`{endColor}"
            )
        return


def get_mujoco_version():
    with open("./libs/mujoco/version.txt") as f:
        return f.read().strip()


def is_conan_package_available(ref: str):
    """
    Run 'conan list' and return True if package exists in local or remote caches.
    """
    try:
        output = subprocess.check_output(
            [sys.executable, "-m", "conans.conan", "list", ref, "-c", "-f", "json", "-verror"],
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )
        parsed = json.loads(output)
        return any( "error" not in v for v in parsed.values() )
    except subprocess.CalledProcessError:
        return False


def conan_create_mujoco(print_fn: Optional[Callable[[str], None]] = print):
    """
    If the 'mujoco/VERSION' package is not found in any remote or the local cache,
    then the mujoco project (as defined in '/libs/mujoco/conanfile.py') is created
    into the local cache.
    """
    ref = f"mujoco/{get_mujoco_version()}"
    if not is_conan_package_available(ref):
        if print_fn is not None:
            print_fn(f"Package {ref} not found locally, creating it...")
        # Run 'conan create' in the external recipe directory
        subprocess.run([sys.executable, "-m", "conans.conan", "create", ".", "-s" ,"compiler.cppstd=17"], cwd="./libs/mujoco" )
    else:
        if print_fn is not None:
            print_fn(f"Package {ref} already available, skipping creation.")

if __name__ == "__main__":
    # make sure conan is configured to use the libstdc++11 by default
    # XXX: This needs to be run before dispatching to Conan (i.e. outside of the
    # ConanFile object), because it affects the configuration of the first run.
    # (Running it here fixes https://github.com/AVSLab/basilisk/issues/525)
    try:
        subprocess.check_output([sys.executable, "-m", "conans.conan", "profile", "detect", "--exist-ok"])
    except:
        # if profile already exists the above command returns an error.  Just ignore in this
        # case.  We don't want to overwrite an existing profile file
        pass
    print(statusColor + "Checking conan configuration:" + endColor + " Done")

    parser = argparse.ArgumentParser(description="Configure the Basilisk framework.")
    # define the optional arguments
    parser.add_argument("--generator", help="cmake generator")
    parser.add_argument("--buildType", help="build type", default="Release", choices=["Release", "Debug"])
    # parser.add_argument("--clean", help="make a clean distribution folder", action="store_true")
    for opt, value in bskModuleOptionsBool.items():
        parser.add_argument("--" + opt, help="build modules for " + opt + " behavior", default=value[1],
                            type=lambda x: (str(x).lower() == 'true'))
    for opt, value in bskModuleOptionsString.items():
        parser.add_argument("--" + opt, help="using string option for " + opt, default=value[1])
    for opt, value in bskModuleOptionsFlag.items():
        if sys.version_info < (3, 9, 0):
            parser.add_argument("--" + opt, help="using flag option for " + opt, default=value[1], action='store_true')
        else:
            parser.add_argument("--" + opt, help="using flag option for " + opt, default=value[1],
                                action=argparse.BooleanOptionalAction)
    args = parser.parse_args()

    # set the build destination folder
    # buildFolderName = 'dist3/conan'

    # run the auto-module generation script
    # this ensures that this script is up-to-date with the latest BSK code base
    # and that the associated unit test draft file runs
    print(statusColor + "Auto-Generating Draft Modules... " + endColor, end=" ")
    genMod = makeDraftModule.moduleGenerator()
    genMod.cleanBuild = True
    genMod.verbose = False
    makeDraftModule.fillCppInfo(genMod)
    genMod.createCppModule()
    makeDraftModule.fillCInfo(genMod)
    genMod.createCModule()
    print("Done")

    # If we're missing MuJoCo, create the conan package
    if args.mujoco:
        conan_create_mujoco()

    try:
        buildCmd = create_conan_build_command(args)
    except ValueError as error:
        print(f"{failColor}Error: {error}{endColor}")
        sys.exit(1)

    print(statusColor + "Running conan build:" + endColor)
    print(shlex.join(buildCmd))
    subprocess.run(buildCmd, check=True)
