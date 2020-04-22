import os
from datetime import datetime
from conans import ConanFile, CMake, tools
import shutil
import argparse

# define BSK module option list (option name and default value)
bskModuleOptions = {
    "opNav": False,
    "vizInterface": True,
    "python3": True
}

class BasiliskConan(ConanFile):
    name = "Basilisk"
    homepage = "http://hanspeterschaub.info/basilisk"
    version = '1.7.2'
    generators = "cmake_find_package_multi"
    requires = "eigen/3.3.7@conan/stable"
    settings = "os", "compiler", "build_type", "arch"
    build_policy = "missing"

    options = { "clean": [True, False],
                "generateIdeProject": [True, False],
                "buildProject": [True, False]}

    default_options = { "clean": False,
                        "generateIdeProject": True,
                        "buildProject": False}

    for opt, value in bskModuleOptions.items():
        options.update({opt: [True, False]})
        default_options.update({opt: value})

    generator = None

    def requirements(self):
        if self.options.opNav:
            self.options.vizInterface = True
            self.requires.add("opencv/4.1.1@conan/stable")
            self.requires.add("zlib/1.2.11@conan/stable")
            self.requires.add("bzip2/1.0.8@conan/stable")

        if self.options.vizInterface:
            self.requires.add("libsodium/1.0.18@bincrafters/stable")
            self.requires.add("protobuf/3.5.2@bincrafters/stable")
            self.requires.add("cppzmq/4.3.0@bincrafters/stable")            


    def configure(self):
        if self.options.clean:
            # clean the distribution folder to start fresh
            self.options.clean = False
            root = os.path.abspath(os.path.curdir)
            distPath = root + "/dist"
            if self.options.python3:
                distPath += "3"
            if os.path.exists(distPath):
                shutil.rmtree(distPath)

        if self.settings.build_type == "Debug":
            print("Build type is set to Debug. Performance will be significantly lower.")

        # Install additional opencv methods
        if self.options.opNav:
            self.options['opencv'].contrib = True

        if self.options.generateIdeProject:
            # Select default generator supplied to cmake based on os
            if self.settings.os == "Macos":
                self.generator = "Xcode"
            elif self.settings.os == "Linux":
                self.generator = "Make"
            elif self.settings.os == "Windows":
                self.generator = "visual_studio_multi"
            else:
                print("OS not detected, conan auto-determines best generator. ")
                print("Specify your own using the `-g GENERATOR` flag during conan install")

    def build(self):
        root = os.path.abspath(os.path.curdir)

        self.source_folder = root + "/src"
        self.build_folder = root + "/dist"
        if self.options.python3:
            self.build_folder += "3"  

        cmake = CMake(self, set_cmake_flags=True, generator=self.generator) 
        cmake.definitions["USE_PYTHON3"] = self.options.python3
        cmake.definitions["BUILD_OPNAV"] = self.options.opNav
        cmake.definitions["BUILD_VIZINTERFACE"] = self.options.vizInterface
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
    parser.add_argument("--generator", help="cmake generator")
    parser.add_argument("--buildType", help="build type", default="Release", choices=["Release", "Debug"])
    parser.add_argument("--buildProject", help="flag to compile the code", action="store_true")
    parser.add_argument("--clean", help="make a clean distribution folder", action="store_true")
    for opt, value in bskModuleOptions.items():
        parser.add_argument("--" + opt, help="build modules for " + opt + " behavior", default=value,
                            type=lambda x: (str(x).lower() == 'true'))
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
    if args.generator:
        conanCmdString += ' -g ' + str(args.generator) + ' -o generateIdeProject=False'
    if args.clean:
        conanCmdString += ' -o clean=True'
    if args.buildProject:
        conanCmdString += ' -o buildProject=True'
    for opt, value in bskModuleOptions.items():
        conanCmdString += ' -o ' + opt + '=' + str(vars(args)[opt])
    print("Running this conan command:")
    print(conanCmdString)
    os.system(conanCmdString)

    # run conan build
    cmakeCmdString = 'conan build . -if ' + buildFolderName
    print("Running cmake:")
    print(cmakeCmdString)
    os.system(cmakeCmdString)