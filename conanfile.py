from conans import ConanFile, CMake

class BasiliskConan(ConanFile):
    generators = "cmake_find_package_multi"
    requires = "eigen/3.3.7@conan/stable"
    settings = "os", "compiler", "build_type", "arch"
    options = {"with_opnav": [True, False], "with_vizInterface": [True, False]}
    default_options = {"with_opnav": False, "with_vizInterface": True}

    def requirements(self):
        if self.options.with_opnav: 
            self.options.with_vizInterface = True
            self.requires("opencv/4.1.1@conan/stable")
            self.requires("zlib/1.2.11@conan/stable")
            self.requires("bzip2/1.0.8@conan/stable")

        if self.options.with_vizInterface: 
            self.requires("libsodium/1.0.18@bincrafters/stable")
            self.requires("protobuf/3.5.2@bincrafters/stable")
            self.requires("cppzmq/4.3.0@bincrafters/stable")

    def configure(self):
        if self.options.with_opnav:
            self.options['opencv'].contrib = True


#    def build(self):
#        cmake = CMake(self)
#        cmake.definitions[""]
#        cmake.configure()
#        cmake.build()
#
#        
