import os
from datetime import datetime
from conans import ConanFile, CMake, tools

class BasiliskConan(ConanFile):
    root = os.path.abspath(os.path.curdir)
    name = "Basilisk"
    homepage = "http://hanspeterschaub.info/basilisk"
    version = '1.7.2'
    generators = "cmake_find_package_multi"
    requires = "eigen/3.3.7@conan/stable"
    settings = "os", "compiler", "build_type", "arch"
    build_policy = 'missing'

    options = { "xcodeProject": [True, False], 
                "opnav_packages": [True, False], 
                "vizInterface_packages": [True, False]}

    default_options = { "xcodeProject": False,
                        "opnav_packages": False, 
                        "vizInterface_packages": True}

    generator=None # Autoselect fastest generator

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


    def build(self):
        if self.options.xcodeProject == True:
            self.generator="Xcode"

        cmake = CMake(self,set_cmake_flags=True, generator=self.generator)    
        cmake.definitions["BUILD_OPNAV"] = self.options.opnav_packages
        cmake.definitions["BUILD_VIZINTERFACE"] = self.options.vizInterface_packages
        cmake.configure()

        if self.options.xcodeProject == False: # Need to build through xcode to support multithreading. 
            start = datetime.now()
            print("CPU Count: " +str(tools.cpu_count()))
            cmake.build()
            print("Total Build Time: "+ str(datetime.now()-start))
        
        #self.run("python3 -m pip3 install --user matplotlib")
        #self.run("python3 -m pip3 install --user numpy")
        #self.run("python3 -m pip3 install --user pandas")

