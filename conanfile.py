import os
from datetime import datetime
from conans import ConanFile, CMake, tools
import shutil
import argparse

class BasiliskConan(ConanFile):
    name = "Basilisk"
    homepage = "http://hanspeterschaub.info/basilisk"
    version = '1.7.2'
    generators = "cmake_find_package_multi"
    requires = "eigen/3.3.7@conan/stable"
    settings = "os", "compiler", "build_type", "arch"
    build_policy = "missing"

    options = { "cleanDist" : [True, False],
                "python3": [True, False], 
                "generateProject": [True, False], 
                "buildProject": [True, False], 
                "opnav_packages": [True, False], 
                "vizInterface_packages": [True, False]}

    default_options = { "cleanDist": False,
                        "python3": True,
                        "generateProject": True,
                        "buildProject": False,
                        "opnav_packages": False, 
                        "vizInterface_packages": True}

    generator=None 

    def requirements(self):
        if self.options.opnav_packages: 
            self.options.vizInterface_packages = True
            self.requires.add("opencv/4.1.1@conan/stable")
            self.requires.add("zlib/1.2.11@conan/stable")
            self.requires.add("bzip2/1.0.8@conan/stable")

        if self.options.vizInterface_packages: 
            self.requires.add("libsodium/1.0.18@bincrafters/stable")
            self.requires.add("protobuf/3.5.2@bincrafters/stable")
            self.requires.add("cppzmq/4.3.0@bincrafters/stable")            


    def configure(self):
        if self.options.cleanDist:
            # clean the distribution folder to start fresh
            self.options.cleanDist = False
            root = os.path.abspath(os.path.curdir)
            distPath = root + "/dist"
            if self.options.python3:
                distPath += "3"
            if os.path.exists(distPath):
                shutil.rmtree(distPath)

        if self.settings.build_type == "Debug":
            print("Build type is set to Debug. Performance will be significantly lower.")

        # Install additional opencv methods
        if self.options.opnav_packages:
            self.options['opencv'].contrib = True

        if self.options.generateProject:
            # Select default generator supplied to cmake based on os
            if self.settings.os == "Macos":
                self.generator = "Xcode"
            elif self.settings.os == "Windows":
                self.generator = "visual_studio_multi"
            else:
                print("No default generator for Linux. Specify your own using the `-g GENERATOR` flag during conan install")

    def build(self):
        root = os.path.abspath(os.path.curdir)

        self.source_folder = root + "/src"
        self.build_folder = root + "/dist"
        if self.options.python3:
            self.build_folder += "3"  

        cmake = CMake(self, set_cmake_flags=True, generator=self.generator) 
        cmake.definitions["USE_PYTHON3"] = self.options.python3
        cmake.definitions["BUILD_OPNAV"] = self.options.opnav_packages
        cmake.definitions["BUILD_VIZINTERFACE"] = self.options.vizInterface_packages
        cmake.parallel = True
        cmake.configure()

        if self.options.buildProject: 
            start = datetime.now()
            if self.generator == "Xcode":
                cmake.build(['--', '-jobs', str(tools.cpu_count()), '-parallelizeTargets']) #Xcode multithreaded needs specialized arguments
            else:
                cmake.build()
            print("Total Build Time: "+ str(datetime.now()-start))
        return

        #self.run("python3 -m pip3 install --user matplotlib")
        #self.run("python3 -m pip3 install --user numpy")
        #self.run("python3 -m pip3 install --user pandas")




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Configure the Basilisk framework.")
    # define the optional arguments
    parser.add_argument("--python3", help="build for Python 3", default=True, type=lambda x: (str(x).lower() == 'true'))
    parser.add_argument("--generator", help="cmake generator")
    parser.add_argument("--buildType", help="build type", default="Release", choices=["Release", "Debug"])
    parser.add_argument("--buildProject", help="flag to compile the code", action="store_true")
    parser.add_argument("--opNav", help="build modules for OpNav behavior", default=False,
                        type=lambda x: (str(x).lower() == 'true'))
    parser.add_argument("--vizInterface", help="build vizInterface module", default=True,
                        type=lambda x: (str(x).lower() == 'true'))
    parser.add_argument("--clean", help="make a clean distribution folder", action="store_true")
    args = parser.parse_args()

    # set the build destination folder
    if args.python3:
        buildFolderName = 'dist3'
    else:
        buildFolderName = 'dist'
        print("Building for Python 2 (depreciated)")
    buildFolderName += '/conan'

    # run conan install
    conanCmdString = 'conan install . --build=missing'
    conanCmdString += ' -s build_type=' + str(args.buildType)
    conanCmdString += ' -if ' + buildFolderName
    conanCmdString += ' -o opnav_packages=' + str(args.opNav)
    conanCmdString += ' -o vizInterface_packages=' + str(args.vizInterface)
    if args.generator:
        conanCmdString += ' -g ' + str(args.generator) + ' -o generateProject=False'
    if args.clean:
        conanCmdString += ' -o cleanDist=True'
    if args.buildProject:
        conanCmdString += ' -o buildProject=True'
    print("Running this conan command:")
    print(conanCmdString)
    os.system(conanCmdString)

    # run conan build
    cmakeCmdString = 'conan build . -if ' + buildFolderName + '/conan'
    print("Running cmake:")
    print(cmakeCmdString)
    os.system(cmakeCmdString)