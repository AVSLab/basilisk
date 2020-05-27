import os, sys
from datetime import datetime
from conans import ConanFile, CMake, tools
import shutil
import argparse
import pkg_resources
import subprocess
import itertools

# define BSK module option list (option name and default value)
bskModuleOptionsBool = {
    "opNav": False,
    "vizInterface": True
}
bskModuleOptionsString = {
    "autoKey": ""
}
bskModuleOptionsFlag = {
    "clean": False,
    "allOptPkg": False,
    "buildProject": False
}

statusColor = '\033[92m'
failColor = '\033[91m'
warningColor = '\033[93m'
endColor = '\033[0m'

class BasiliskConan(ConanFile):
    name = "Basilisk"
    homepage = "http://hanspeterschaub.info/basilisk"
    f = open('docs/source/bskVersion.txt', 'r')
    version = f.read()
    f.close()
    generators = "cmake_find_package_multi"
    requires = "eigen/3.3.7@conan/stable"
    settings = "os", "compiler", "build_type", "arch"
    build_policy = "missing"
    license = "ISC"

    options = {"generator": "ANY"}
    default_options = {"generator": ""}

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

    def system_requirements(self):
        reqFile = open('docs/source/bskPkgRequired.txt', 'r')
        required = reqFile.read().replace("`", "").replace(",", "").split('\n')
        reqFile.close()
        pkgList = [x.lower() for x in required]
        checkStr = "Required"

        if self.options.allOptPkg:
            optFile = open('docs/source/bskPkgOptions.txt', 'r')
            optionalPkgs = optFile.read().replace("`", "").replace(",", "").split('\n')
            optFile.close()
            optionalPkgs = [x.lower() for x in optionalPkgs]
            pkgList += optionalPkgs
            checkStr += " and All Optional"

        print("\nChecking " + checkStr + " Python packages:")
        for elem in pkgList:
            try:
                pkg_resources.require(elem)
                print("Found " + statusColor + elem + endColor)
            except (pkg_resources.DistributionNotFound, pkg_resources.VersionConflict):
                if self.options.autoKey:
                    choice = self.options.autoKey
                else:
                    choice = input(warningColor + "Required python package " + elem + " is missing" + endColor +
                                   "\nInstall for user (u), system (s) or cancel(c)? ")
                installCmd = [sys.executable, "-m", "pip", "install"]
                if choice in ['s', 'u']:
                    if choice == 'u':
                        installCmd.append("--user")
                    installCmd.append(elem)

                    try:
                        subprocess.check_call(installCmd)
                    except subprocess.CalledProcessError:
                        print(failColor + "Was not able to install " + elem + endColor)
                else:
                    print(warningColor + "Skipping installing " + elem + endColor)

        # check the version of Python
        if not (sys.version_info.major == 3 and sys.version_info.minor >= 7):
            print(warningColor + "Python 3.7 should be used with Basilisk." + endColor)
            print("You are using Python {}.{}.{}".format(sys.version_info.major,
                                                         sys.version_info.minor,sys.version_info.micro))
        else:
            print(statusColor + "Python {}.{}.{}".format(sys.version_info.major,
                                                         sys.version_info.minor, sys.version_info.micro)
                  + " is acceptable for Basilisk" + endColor)

        print("\n")

    def requirements(self):
        if self.options.opNav:
            self.requires.add("opencv/4.1.1@conan/stable")
            self.requires.add("zlib/1.2.11@conan/stable")
            self.requires.add("bzip2/1.0.8@conan/stable")

        if self.options.vizInterface or self.options.opNav:
            self.requires.add("libsodium/1.0.18@bincrafters/stable")
            self.requires.add("protobuf/3.5.2@bincrafters/stable")
            self.requires.add("cppzmq/4.3.0@bincrafters/stable")            


    def configure(self):
        if self.options.clean:
            # clean the distribution folder to start fresh
            self.options.clean = False
            root = os.path.abspath(os.path.curdir)
            distPath = os.path.join(root, "dist3")
            if os.path.exists(distPath):
                shutil.rmtree(distPath, ignore_errors=True)

        if self.settings.build_type == "Debug":
            print("Build type is set to Debug. Performance will be significantly lower.")

        # Install additional opencv methods
        if self.options.opNav:
            self.options['opencv'].contrib = True

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
        print("cmake generator set to: " + str(self.generator))

    def build(self):
        root = os.path.abspath(os.path.curdir)

        self.source_folder = os.path.join(root, "src")
        self.build_folder = os.path.join(root, "dist3")

        cmake = CMake(self, set_cmake_flags=True, generator=self.generator)
        cmake.definitions["BUILD_OPNAV"] = self.options.opNav
        cmake.definitions["BUILD_VIZINTERFACE"] = self.options.vizInterface
        cmake.parallel = True
        cmake.configure()

        if self.options.buildProject: 
            start = datetime.now()
            if self.generator == "Xcode":
                # Xcode multi-threaded needs specialized arguments
                cmake.build(['--', '-jobs', str(tools.cpu_count()), '-parallelizeTargets'])
            else:
                cmake.build()
            print("Total Build Time: " + str(datetime.now()-start))
        return


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
    buildFolderName = 'dist3'
    buildFolderName += '/conan'

    # run conan install
    conanCmdString = 'conan install . --build=missing'
    conanCmdString += ' -s build_type=' + str(args.buildType)
    conanCmdString += ' -if ' + buildFolderName
    if args.generator:
        conanCmdString += ' -o generator="' + str(args.generator) + '"'
    for opt, value in bskModuleOptionsBool.items():
        conanCmdString += ' -o ' + opt + '=' + str(vars(args)[opt])
    for opt, value in bskModuleOptionsString.items():
        if str(vars(args)[opt]):
            conanCmdString += ' -o ' + opt + '=' + str(vars(args)[opt])
    for opt, value in bskModuleOptionsFlag.items():
        if vars(args)[opt]:
            conanCmdString += ' -o ' + opt + '=True'
    print(statusColor + "Running this conan command:" + endColor)
    print(conanCmdString)
    os.system(conanCmdString)

    # run conan build
    cmakeCmdString = 'conan build . -if ' + buildFolderName
    print(statusColor + "Running cmake:" + endColor)
    print(cmakeCmdString)
    os.system(cmakeCmdString)
