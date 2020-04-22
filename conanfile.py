import os
from datetime import datetime
from conans import ConanFile, CMake, tools

class BasiliskConan(ConanFile):
    name = "Basilisk"
    homepage = "http://hanspeterschaub.info/basilisk"
    version = '1.7.2'
    generators = "cmake_find_package_multi"
    requires = "eigen/3.3.7@conan/stable"
    settings = "os", "compiler", "build_type", "arch"
    build_policy = 'missing'

    options = { "python3": [True, False], 
                "generateProject": [True, False], 
                "buildProject": [True, False], 
                "opnav_packages": [True, False], 
                "vizInterface_packages": [True, False]}

    default_options = { "python3": True,
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
        self.install_folder = self.build_folder + "/conan"

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

