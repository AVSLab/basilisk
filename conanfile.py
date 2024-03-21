import argparse
import os
import platform
import shutil
import subprocess
import sys
from datetime import datetime

import pkg_resources

sys.path.insert(1, './src/utilities/')
import makeDraftModule

# define the print color codes
statusColor = '\033[92m'
failColor = '\033[91m'
warningColor = '\033[93m'
endColor = '\033[0m'

try:
    from conans import __version__ as conan_version
    if int(conan_version[0]) >= 2:
        print(failColor + "conan version " + conan_version + " is not compatible with Basilisk.")
        print("use version 1.40.1 to 1.xx.0 to work with the conan repo changes." + endColor)
        exit(0)
    from conans.tools import Version
    # check conan version 1.xx
    if conan_version < Version("1.40.1"):
        print(failColor + "conan version " + conan_version + " is not compatible with Basilisk.")
        print("use version 1.40.1+ to work with the conan repo changes from 2021." + endColor)
        exit(0)
    from conans import ConanFile, CMake, tools
except ModuleNotFoundError:
    print("Please make sure you install python conan (version 1.xx, not 2.xx) package\nRun command `pip install conan` "
          "for Windows\nRun command `pip3 install conan` for Linux/MacOS")
    sys.exit(1)

# check that the large spice data files were installed.
filePath = os.getcwd()
filePath = os.path.join(filePath, "supportData/EphemerisData/de430.bsp")
fileStats = os.stat(filePath)
if fileStats.st_size < 1024:
    print(failColor + "GIT CLONING ERROR: Git didn't pull the large data files.  You must install lfs first "
                      "before cloning the repo. "
                      "See step 1 in http://hanspeterschaub.info/basilisk/Install/pullCloneBSK.html"+ endColor)
    exit(0)

# define BSK module option list (option name and default value)
bskModuleOptionsBool = {
    "opNav": False,
    "vizInterface": True,
    "buildProject": True
}
bskModuleOptionsString = {
    "autoKey": "",
    "pathToExternalModules": ""
}
bskModuleOptionsFlag = {
    "clean": False,
    "allOptPkg": False
}

# this statement is needed to enable Windows to print ANSI codes in the Terminal
# see https://stackoverflow.com/questions/287871/how-to-print-colored-text-in-terminal-in-python/3332860#3332860
os.system("")

def is_running_virtual_env():
    return sys.prefix != sys.base_prefix

class BasiliskConan(ConanFile):
    name = "Basilisk"
    homepage = "http://hanspeterschaub.info/basilisk"
    f = open('docs/source/bskVersion.txt', 'r')
    version = f.read()
    f.close()
    generators = "cmake_find_package_multi"
    requires = "eigen/3.3.9"
    settings = "os", "compiler", "build_type", "arch"
    build_policy = "missing"
    license = "ISC"

    options = {"generator": "ANY"}
    default_options = {"generator": ""}

    # ensure latest pip is installed
    if is_running_virtual_env() or platform.system() == "Windows":
        cmakeCmdString = 'python -m pip install --upgrade pip'
    else:
        cmakeCmdString = 'python3 -m pip install --upgrade pip'
    print(statusColor + "Updating pip:" + endColor)
    print(cmakeCmdString)
    os.system(cmakeCmdString)

    for opt, value in bskModuleOptionsBool.items():
        options.update({opt: [True, False]})
        default_options.update({opt: value})
    for opt, value in bskModuleOptionsString.items():
        options.update({opt: "ANY"})
        default_options.update({opt: value})
    for opt, value in bskModuleOptionsFlag.items():
        options.update({opt: [True, False]})
        default_options.update({opt: value})

    # set cmake generator default
    generator = None

    # make sure conan is configured to use the libstdc++11 by default
    if platform.system() != "Darwin":
        try:
            subprocess.check_output(["conan", "profile", "new", "default", "--detect"], stdout=subprocess.DEVNULL)
        except:
            pass

        if platform.system() == "Linux":
            try:
                subprocess.check_output(["conan", "profile", "update",
                                         "settings.compiler.libcxx=libstdc++11", "default"])
                print("\nConfiguring: " + statusColor + "use libstdc++11 by default" + endColor)

            except:
                pass

    print(statusColor + "Checking conan configuration:" + endColor + " Done")

    try:
        # enable this flag for access revised conan modules.
        subprocess.check_output(["conan", "config", "set", "general.revisions_enabled=1"])
    except:
        pass

    def system_requirements(self):
        reqFile = open('docs/source/bskPkgRequired.txt', 'r')
        required = reqFile.read().replace("`", "").split('\n')
        reqFile.close()
        pkgList = [x.lower() for x in required]

        checkStr = "Required"
        if self.options.allOptPkg:
            optFile = open('docs/source/bskPkgOptions.txt', 'r')
            optionalPkgs = optFile.read().replace("`", "").split('\n')
            optFile.close()
            optionalPkgs = [x.lower() for x in optionalPkgs]
            pkgList += optionalPkgs
            checkStr += " and All Optional"

        print("\nChecking " + checkStr + " Python packages:")
        for elem in pkgList:
            try:
                pkg_resources.require(elem)
                print("Found: " + statusColor + elem + endColor)
            except (pkg_resources.DistributionNotFound, pkg_resources.VersionConflict):
                installCmd = [sys.executable, "-m", "pip", "install"]

                if not is_running_virtual_env():
                    if self.options.autoKey:
                        choice = self.options.autoKey
                    else:
                        choice = input(warningColor + "Required python package " + elem + " is missing" + endColor +
                                       "\nInstall for user (u), system (s) or cancel(c)? ")
                    if choice == 'c':
                        print(warningColor + "Skipping installing " + elem + endColor)
                        continue
                    elif choice == 'u':
                        installCmd.append("--user")
                installCmd.append(elem)
                try:
                    subprocess.check_call(installCmd)
                except subprocess.CalledProcessError:
                    print(failColor + "Was not able to install " + elem + endColor)

        # check the version of Python
        print("\nChecking Python version:")
        if not (sys.version_info.major == 3 and sys.version_info.minor >= 8):
            print(warningColor + "Python 3.8 or newer should be used with Basilisk." + endColor)
            print("You are using Python {}.{}.{}".format(sys.version_info.major,
                                                         sys.version_info.minor, sys.version_info.micro))
        else:
            print(statusColor + "Python {}.{}.{}".format(sys.version_info.major,
                                                         sys.version_info.minor, sys.version_info.micro)
                  + " is acceptable for Basilisk" + endColor)

        print("\n")

    def requirements(self):
        if self.options.opNav:
            self.requires.add("pcre/8.45")
            self.requires.add("opencv/4.1.2#b610ad323f67adc1b51e402cb5d68d70")
            self.options['opencv'].with_ffmpeg = False  # video frame encoding lib
            self.options['opencv'].with_ade = False  # graph manipulations framework
            self.options['opencv'].with_tiff = False  # generate image in TIFF format
            self.options['opencv'].with_openexr = False  # generate image in EXR format
            self.options['opencv'].with_quirc = False  # QR code lib
            self.requires.add("zlib/1.2.13")
            self.requires.add("xz_utils/5.4.0")

        if self.options.vizInterface or self.options.opNav:
            self.requires.add("protobuf/3.17.1")
            self.options['zeromq'].encryption = False  # Basilisk does not use data streaming encryption.
            self.requires.add("cppzmq/4.5.0")

    def configure(self):
        if self.options.clean:
            # clean the distribution folder to start fresh
            self.options.clean = False
            root = os.path.abspath(os.path.curdir)
            distPath = os.path.join(root, "dist3")
            if os.path.exists(distPath):
                shutil.rmtree(distPath, ignore_errors=True)
        if self.settings.build_type == "Debug":
            print(warningColor + "Build type is set to Debug. Performance will be significantly lower." + endColor)

        # Install additional opencv methods
        if self.options.opNav:
            self.options['opencv'].contrib = True
            # Raise an issue to conan-center to fix this bug. Using workaround to disable freetype for windows
            # Issue link: https://github.com/conan-community/community/issues/341
            #TODO Remove this once they fix this issue.
            if self.settings.os == "Windows":
                self.options['opencv'].contrib_freetype = False

        if self.options.generator == "":
            # Select default generator supplied to cmake based on os
            if self.settings.os == "Macos":
                self.generator = "Xcode"
            elif self.settings.os == "Windows":
                self.generator = "Visual Studio 16 2019"
                self.options["*"].shared = True
            else:
                print("Creating a make file for project. ")
                print("Specify your own using the -o generator='Name' flag during conan install")
        else:
            self.generator = str(self.options.generator)
            if self.settings.os == "Windows":
                self.options["*"].shared = True
        print("cmake generator set to: " + statusColor + str(self.generator) + endColor)
    
    def package_id(self):
        if self.settings.compiler == "Visual Studio":
            if "MD" in self.settings.compiler.runtime:
                self.info.settings.compiler.runtime = "MD/MDd"
            else:
                self.info.settings.compiler.runtime = "MT/MTd"

    def imports(self):
        if self.settings.os == "Windows":
            self.keep_imports = True
            self.copy("*.dll", "../Basilisk", "bin")

    def generateMessageModules(self, originalWorkingDirectory):
        cmdString = [sys.executable, "GenCMessages.py"]
        if self.options.pathToExternalModules:
            cmdString.extend(["--pathToExternalModules", str(self.options.pathToExternalModules)])
        subprocess.check_call(cmdString)
        os.chdir(originalWorkingDirectory)
        print("Done")

    def build(self):
        # auto-generate C message definition files
        print(statusColor + "Auto-generating message definitions:" + endColor, end=" ")
        bskPath = os.getcwd()
        os.chdir(os.path.join(bskPath, "src/architecture/messaging/msgAutoSource"))
        self.generateMessageModules(bskPath)

        if self.options.pathToExternalModules:
            print(statusColor + "Including External Folder: " + endColor + str(self.options.pathToExternalModules))

        root = os.path.abspath(os.path.curdir)

        self.folders.source = os.path.join(root, "src")
        self.folders.build = os.path.join(root, "dist3")

        cmake = CMake(self, set_cmake_flags=True, generator=self.generator)
        if self.settings.compiler == "Visual Studio":
            cmake.definitions["CONAN_LINK_RUNTIME_MULTI"] = cmake.definitions["CONAN_LINK_RUNTIME"]
            cmake.definitions["CONAN_LINK_RUNTIME"] = False
        cmake.definitions["BUILD_OPNAV"] = self.options.opNav
        cmake.definitions["BUILD_VIZINTERFACE"] = self.options.vizInterface
        cmake.definitions["EXTERNAL_MODULES_PATH"] = self.options.pathToExternalModules
        cmake.definitions["PYTHON_VERSION"] = f"{sys.version_info.major}.{sys.version_info.minor}"
        cmake.parallel = True
        print(statusColor + "Configuring cmake..." + endColor)
        cmake.configure()
        self.add_basilisk_to_sys_path()
        if self.options.buildProject:
            print(statusColor + "\nCompiling Basilisk..." + endColor)
            start = datetime.now()
            if self.generator == "Xcode":
                # Xcode multi-threaded needs specialized arguments
                cmake.build(['--', '-jobs', str(tools.cpu_count()), '-parallelizeTargets'])
            else:
                cmake.build()
            print("Total Build Time: " + str(datetime.now() - start))
            print(f"{statusColor}The Basilisk build is successful and the scripts are ready to run{endColor}")
        else:
            print(f"{statusColor}Finished configuring the Basilisk project.{endColor}")
            if self.settings.os != "Linux":
                print(f"{statusColor}Please open project file inside dist3 with {self.generator} IDE "
                      f"and build the project for {self.settings.build_type}{endColor}")
            else:
                print(f"{statusColor}Please go to dist3 folder and run command "
                      f"`make -j <number of threads to use>`{endColor}")
        return

    def add_basilisk_to_sys_path(self):
        print("Adding Basilisk module to python\n")
        add_basilisk_module_command = [sys.executable, "-m", "pip", "install", "-e", "."]
        if not is_running_virtual_env() and self.options.autoKey != 's':
            add_basilisk_module_command.append("--user")

        process = subprocess.Popen(add_basilisk_module_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, err = process.communicate()
        if process.returncode:
            print("Error %s while running %s" % (err.decode(), add_basilisk_module_command))
            sys.exit(1)
        else:
            print("This resulted in the stdout: \n%s" % output.decode())
            print("This resulted in the stderr: \n%s" % err.decode())

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Configure the Basilisk framework.")
    # define the optional arguments
    parser.add_argument("--generator", help="cmake generator")
    parser.add_argument("--buildType", help="build type", default="Release", choices=["Release", "Debug"])
    # parser.add_argument("--clean", help="make a clean distribution folder", action="store_true")
    for opt, value in bskModuleOptionsBool.items():
        parser.add_argument("--" + opt, help="build modules for " + opt + " behavior", default=value,
                            type=lambda x: (str(x).lower() == 'true'))
    for opt, value in bskModuleOptionsString.items():
        parser.add_argument("--" + opt, help="using string option for " + opt, default=value)
    for opt, value in bskModuleOptionsFlag.items():
        parser.add_argument("--" + opt, help="using flag option for " + opt, action="store_true")
    args = parser.parse_args()

    # set the build destination folder
    buildFolderName = 'dist3/conan'

    # run the auto-module generation script
    # this ensures that this script is up to date with the latest BSK code base
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

    # run conan install
    conanCmdString = list()
    if is_running_virtual_env() or platform.system() == "Windows":
        conanCmdString.append('python -m conans.conan install . --build=missing')
    else:
        conanCmdString.append('python3 -m conans.conan install . --build=missing')
    conanCmdString.append(' -s build_type=' + str(args.buildType))
    conanCmdString.append(' -if ' + buildFolderName)
    if args.generator:
        conanCmdString.append(' -o generator="' + str(args.generator) + '"')
    for opt, value in bskModuleOptionsBool.items():
        conanCmdString.append(' -o ' + opt + '=' + str(vars(args)[opt]))
    for opt, value in bskModuleOptionsString.items():
        if str(vars(args)[opt]):
            if opt == "pathToExternalModules":
                externalPath = os.path.abspath(str(vars(args)[opt]).rstrip(os.path.sep))
                if os.path.exists(externalPath):
                    conanCmdString.append(' -o ' + opt + '=' + externalPath)
                else:
                    print(f"{failColor}Error: path {str(vars(args)[opt])} does not exist{endColor}")
                    sys.exit(1)
            else:
                conanCmdString.append(' -o ' + opt + '=' + str(vars(args)[opt]))
    for opt, value in bskModuleOptionsFlag.items():
        if vars(args)[opt]:
            conanCmdString.append(' -o ' + opt + '=True')
    conanCmdString = ''.join(conanCmdString)
    print(statusColor + "Running this conan command:" + endColor)
    print(conanCmdString)
    os.system(conanCmdString)

    # run conan build
    if is_running_virtual_env() or platform.system() == "Windows":
        cmakeCmdString = 'python -m conans.conan build . -if ' + buildFolderName
    else:
        cmakeCmdString = 'python3 -m conans.conan build . -if ' + buildFolderName
    print(statusColor + "Running cmake:" + endColor)
    print(cmakeCmdString)
    os.system(cmakeCmdString)



