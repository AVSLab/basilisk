import argparse
import os
import platform
import json
import shutil
import subprocess
import sys
from datetime import datetime
from typing import Optional, Callable
from glob import glob

import importlib.metadata
from packaging.requirements import Requirement

from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.microsoft import is_msvc
from conan.tools.files import copy
from pathlib import Path

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
    "buildProject": [[True, False], True],
    "pyPkgCanary": [[True, False], False],
    "recorderPropertyRollback": [[True, False], False],

    # XXX: Set managePipEnvironment to True to keep the old behaviour of
    # managing the `pip` environment directly (upgrading, installing Python
    # packages, etc.). This behaviour is deprecated using the new pip-based
    # installation, and should only be required for users who are still calling
    # this file with `python conanfile.py ...`.
    # TODO: Remove all managePipEnvironment behaviour!
    "managePipEnvironment": [[True, False], True],
}
bskModuleOptionsString = {
    "autoKey": [["", "u", "s","c"], ""],  # TODO: Remove, used only for managePipEnvironment.
    "pathToExternalModules": [["ANY"], ""],
    "pyLimitedAPI": [["ANY"], ""],
}
bskModuleOptionsFlag = {
    "clean": [[True, False], False],
    "allOptPkg": [[True, False], False]  # TODO: Remove, used only for managePipEnvironment.
}


def is_running_virtual_env():
    return sys.prefix != sys.base_prefix

required_conan_version = ">=2.0.5"

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

    def system_requirements(self):
        if not self.options.get_safe("managePipEnvironment"):
            return  # Don't need to manage any pip requirements

        # TODO: Remove everything here, which only runs if we have set
        # managePipEnvironment (i.e. conanfile.py-based build).

        # ensure latest pip is installed
        cmakeCmdString = f'{sys.executable} -m pip install --upgrade pip'
        print(statusColor + "Updating pip:" + endColor)
        print(cmakeCmdString)
        os.system(cmakeCmdString)

        # TODO: Remove this: requirements and optional requirements are
        # installed automatically by add_basilisk_to_sys_path(). Only build
        # system requirements need to be installed here.
        reqPath = '.github/workflows/' if self.options.get_safe("pyPkgCanary") else ''

        reqFile = open(f'{reqPath}requirements.txt', 'r')
        required = reqFile.read().replace("`", "").split('\n')
        reqFile.close()
        pkgList = [x.lower() for x in required]

        reqFile = open(f'{reqPath}requirements_dev.txt', 'r')
        required = reqFile.read().replace("`", "").split('\n')
        reqFile.close()
        pkgList += [x.lower() for x in required]

        checkStr = "Required"
        if self.options.get_safe("allOptPkg"):
            optFile = open(f'{reqPath}requirements_doc.txt', 'r')
            optionalPkgs = optFile.read().replace("`", "").split('\n')
            optFile.close()
            optionalPkgs = [x.lower() for x in optionalPkgs]
            pkgList += optionalPkgs
            checkStr += " and All Optional"

        print("\nChecking " + checkStr + " Python packages:")
        missing_packages = []
        for elem in pkgList:
            if not elem:  # Skip empty or falsy elements
                continue

            try:
                # Parse the requirement (e.g., "numpy<2")
                req = Requirement(elem)
                # Get the installed version of the package
                installed_version = importlib.metadata.version(req.name)

                # Check if the installed version satisfies the requirement
                if req.specifier.contains(installed_version):
                    print("Found: " + statusColor + elem + endColor)
                else:
                    raise Exception(
                        f"Version conflict for {req.name}: {installed_version} does not satisfy {req.specifier}")
            except importlib.metadata.PackageNotFoundError:
                missing_packages.append(elem)
            except Exception as e:
                print(f"Error: {e}")
                missing_packages.append(elem)

        for elem in missing_packages:
            installCmd = [sys.executable, "-m", "pip", "install"]

            if not is_running_virtual_env():
                if self.options.get_safe("autoKey"):
                    choice = self.options.get_safe("autoKey")
                else:
                    choice = input(warningColor + f"Required python package " + elem + " is missing" + endColor +
                                    "\nInstall for user (u), system (s) or cancel(c)? ")
                if choice == 'c':
                    print(warningColor + "Skipping installing " + elem + endColor)
                    continue
                elif choice == 'u':
                    installCmd.append("--user")
            installCmd.append(elem)
            try:
                subprocess.check_call(installCmd)
                print(f"Installed: {statusColor}{elem}{endColor}")
            except subprocess.CalledProcessError:
                print(failColor + f"Was not able to install " + elem + endColor)

    def build_requirements(self):
        # Protobuf is also required as a tool (in order for CMake to find the
        # Conan-installed `protoc` compiler).
        # See https://github.com/conan-io/conan-center-index/issues/21737
        # and https://github.com/conan-io/conan-center-index/pull/22244#issuecomment-1910770387
        if self.options.get_safe("vizInterface") or self.options.get_safe("opNav"):
            self.tool_requires("protobuf/<host_version>")

    def requirements(self):
        if self.options.get_safe("opNav"):
            self.requires("opencv/4.5.5")

        if self.options.get_safe("vizInterface") or self.options.get_safe("opNav"):
            self.requires("protobuf/3.21.12") # For compatibility with openCV
            self.requires("cppzmq/4.5.0")

        if self.options.get_safe("mujoco"):
            self.requires(f"mujoco/{get_mujoco_version()}")

    def configure(self):
        if self.options.get_safe("clean"):
            # clean the distribution folder to start fresh
            root = os.path.abspath(os.path.curdir)
            distPath = os.path.join(root, "dist3")
            if os.path.exists(distPath):
                shutil.rmtree(distPath, ignore_errors=True)
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

        # Other dependency options
        if self.options.get_safe("vizInterface") or self.options.get_safe("opNav"):
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
        generatorString = str(self.options.get_safe("generator"))
        if generatorString == "":
            # Select default generator supplied to cmake based on os
            if self.settings.os == "Macos" and not self.options.get_safe("buildProject"):
                generatorString = "Xcode"
                tc.generator = generatorString
            elif self.settings.os == "Windows":
                generatorString = "Visual Studio 17 2022"
                tc.generator = generatorString
                self.options["*"].shared = True
            else:
                print("Creating a make file for project. ")
                print("Specify your own using the -o generator='Name' flag during conan install")
        else:
            tc.generator = generatorString
            if self.settings.os == "Windows":
                self.options["*"].shared = True
        print("cmake generator set to: " + statusColor + generatorString + endColor)

        tc.cache_variables["BUILD_OPNAV"] = bool(self.options.get_safe("opNav"))
        tc.cache_variables["BUILD_VIZINTERFACE"] = bool(self.options.get_safe("vizInterface"))
        tc.cache_variables["BUILD_MUJOCO"] = bool(self.options.get_safe("mujoco"))
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
        if self.options.get_safe("pyLimitedAPI"):
            tc.cache_variables["PY_LIMITED_API"] = str(self.options.pyLimitedAPI)

        # Generate!
        tc.generate()

    def build(self):

        cmake = CMake(self)
        print(statusColor + "Configuring cmake..." + endColor)
        cmake.configure()

        if self.options.get_safe("managePipEnvironment"):
            # TODO: it's only needed when conanfile.py is handling pip installations.
            self.add_basilisk_to_sys_path()

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
                for root, _dirs, files in os.walk(self.build_folder):
                    # Skip the destination to avoid self-copy
                    if os.path.commonpath([root, basilisk_dst_root]) == basilisk_dst_root:
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
            if self.settings.os != "Linux":
                print(f"{statusColor}Please open project file inside dist3 with IDE "
                      f"and build the project for {self.settings.build_type}{endColor}")
            else:
                print(f"{statusColor}Please go to dist3 folder and run command "
                      f"`make -j <number of threads to use>`{endColor}")
        return

    def add_basilisk_to_sys_path(self):
        print(f"{statusColor}Adding Basilisk module to python{endColor}\n")
        # NOTE: "--no-build-isolation" is used here only to force pip to use the
        # packages installed directly by this Conanfile (using the
        # "managePipEnvironment" option). Otherwise, it is not necessary.
        add_basilisk_module_command = [sys.executable, "-m", "pip", "install", "--no-build-isolation", "-e", "../"]

        if self.options.get_safe("allOptPkg"):
            # Install the optional requirements as well
            add_basilisk_module_command[-1] = ".[optional]"

        if not is_running_virtual_env() and self.options.get_safe("autoKey") != 's':
            add_basilisk_module_command.append("--user")

        process = subprocess.Popen(add_basilisk_module_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, err = process.communicate()
        if process.returncode:
            print("Error %s while running %s" % (err.decode(), add_basilisk_module_command))
            sys.exit(1)
        else:
            print("This resulted in the stdout: \n%s" % output.decode())
            if err.decode() != "":
                print("This resulted in the stderr: \n%s" % err.decode())

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
    parser.add_argument("--mujocoReplay",
                        help="Whether to build the 'replay' utility for visualizing MuJoCo results",
                        default=False,
                        type=lambda x: (str(x).lower() == 'true'),
                        choices=[True, False])
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

    if args.mujocoReplay:
        print(f"{statusColor}Building 'replay' tool, since '--mujocoReplay true' was used")
        try:
            subprocess.check_output([sys.executable, "-m", "conans.conan", "build", ".", "-s" ,"compiler.cppstd=17", "--build=missing"], cwd="./src/utilities/mujocoUtils" )
        except:
            raise RuntimeError("Failed to install MuJoCo replay! See error above.")

    # setup conan install command arguments
    conanInstallList = list()
    conanInstallList.append(f'{sys.executable} -m conans.conan install . --build=missing')
    conanInstallList.append(' -s build_type=' + str(args.buildType))
    conanInstallList.append(' -s compiler.cppstd=17')
    conanBuildOptionsList = list()  # setup list of conan build arguments
    conanBuildOptionsList.append(' -s compiler.cppstd=17')
    if args.generator:
        conanBuildOptionsList.append(' -o "&:generator=' + str(args.generator) + '"')
    for opt, value in bskModuleOptionsBool.items():
        conanBuildOptionsList.append(' -o "&:' + opt + '=' + str(vars(args)[opt]) + '"')
    conanInstallList.append(''.join(conanBuildOptionsList))  # argument get used in both install and build

    # Most of these options go to both conan install and build commands
    for opt, value in bskModuleOptionsString.items():
        if str(vars(args)[opt]):
            if opt == "pathToExternalModules":
                externalPath = os.path.abspath(str(vars(args)[opt]).rstrip(os.path.sep))
                if os.path.exists(externalPath):
                    conanInstallList.append(' -o "&:' + opt + '=' + externalPath + '"')
                    conanBuildOptionsList.append(' -o "&:' + opt + '=' + externalPath + '"')
                else:
                    print(f"{failColor}Error: path {str(vars(args)[opt])} does not exist{endColor}")
                    sys.exit(1)
            else:
                if opt != "autoKey":
                    conanInstallList.append(' -o "&:' + opt + '=' + str(vars(args)[opt]) + '"')
                    conanBuildOptionsList.append(' -o "&:' + opt + '=' + str(vars(args)[opt]) + '"')

    # only used for conan install arguments, not build
    for opt, value in bskModuleOptionsFlag.items():
        if vars(args)[opt]:
            conanInstallList.append(' -o "&:' + opt + '=True"')
    conanInstallString = ''.join(conanInstallList)

    print(statusColor + "Running conan install:" + endColor)
    print(conanInstallString)
    completedProcess = subprocess.run(conanInstallString, shell=True, check=True)

    # run conan build
    buildCmdString = f'{sys.executable} -m conans.conan build . ' + ''.join(conanBuildOptionsList)
    print(statusColor + "Running conan build:" + endColor)
    print(buildCmdString)
    completedProcess = subprocess.run(buildCmdString, shell=True, check=True)
