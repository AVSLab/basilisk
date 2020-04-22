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

    options = { "python2": [True, False], 
                "generateProject": [True, False], 
                "buildProject": [True, False], 
                "opnav_packages": [True, False], 
                "vizInterface_packages": [True, False]}

    default_options = { "python2": False,
                        "generateProject": True,
                        "buildProject": False,
                        "opnav_packages": False, 
                        "vizInterface_packages": True}

    generator=None # Autoselect fastest generator

    root = os.path.abspath(os.path.curdir)
    build_folder = root + "/dist3"
    source_folder = root + "/src"


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

        if self.options.opnav_packages:
            self.options['opencv'].contrib = True

        if self.options.generateProject:
            if self.settings.os == "Macos":
                self.generator = "Xcode"
            elif self.settings.os == "Windows":
                self.generator = "visual_studio_multi"
            else:
                print("No default generator for Linux. Specify your own using the `-g GENERATOR` flag during conan install")
                self.generator = None

    def build(self):

        if self.options.python2 == True:
            self.build_folder = self.root + "/dist"
        else:
            self.build_folder = self.root + "/dist3"        
        self.source_folder = self.root + "/src"
        self.install_folder = self.build_folder + "/conan"

        cmake = CMake(self,set_cmake_flags=True, generator=self.generator) 
        cmake.definitions["BUILD_OPNAV"] = self.options.opnav_packages
        cmake.definitions["BUILD_VIZINTERFACE"] = self.options.vizInterface_packages
        cmake.configure()

        if self.options.buildProject: # Need to build through xcode to support multithreading. 
            start = datetime.now()
            print("CPU Count: " +str(tools.cpu_count()))
            cmake.build()
            print("Total Build Time: "+ str(datetime.now()-start))
        return

        #self.run("python3 -m pip3 install --user matplotlib")
        #self.run("python3 -m pip3 install --user numpy")
        #self.run("python3 -m pip3 install --user pandas")

