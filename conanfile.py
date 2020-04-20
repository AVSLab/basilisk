from conans import ConanFile, CMake

class BasiliskConan(ConanFile):
    name = "Basilisk"
    homepage = "http://hanspeterschaub.info/basilisk/index.html"
    generators = "cmake_find_package_multi"
    requires = "eigen/3.3.7@conan/stable"
    settings = "os", "compiler", "build_type", "arch"
    options = {"with_opnav": [True, False], "with_vizInterface": [True, False]}
    default_options = {"with_opnav": False, "with_vizInterface": True}

    def requirements(self):
        if self.options.with_opnav: 
            self.options.with_vizInterface = True
            self.requires.add("opencv/4.1.1@conan/stable")
            self.requires.add("zlib/1.2.11@conan/stable")
            self.requires.add("bzip2/1.0.8@conan/stable")

        if self.options.with_vizInterface: 
            self.requires.add("libsodium/1.0.18@bincrafters/stable")
            self.requires.add("protobuf/3.5.2@bincrafters/stable")
            self.requires.add("cppzmq/4.3.0@bincrafters/stable")

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
