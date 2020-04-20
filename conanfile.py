from conans import ConanFile, CMake

class BasiliskConan(ConanFile):
    name = "Basilisk"
    homepage = "http://hanspeterschaub.info/basilisk/index.html"
    generators = "cmake_find_package_multi"
    requires = "eigen/3.3.7@conan/stable"
    settings = "os", "compiler", "build_type", "arch"
    options = {"opnav_packages": [True, False], "vizInterface_packages": [True, False]}
    default_options = {"opnav_packages": False, "vizInterface_packages": True}

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
        if self.options.opnav_packages:
            self.options['opencv'].contrib = True


#    def build(self):
#        cmake = CMake(self)
#        cmake.definitions[""]
#        cmake.configure()
#        cmake.build()
#
#        
