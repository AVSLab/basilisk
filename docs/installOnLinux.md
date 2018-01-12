# Installing On Linux {#installLinux}


## Software setup

In order to run Basilisk, the following software will be necessary:

* [Cmake](https://cmake.org/)
* [Python 2.7 (numpy, matplotlib, pytest)](https://www.python.org/downloads/mac-osx/)


## Dependencies


Note: Depending on your system setup, administrative permissions (sudo or su) may be required to install these dependencies.


1. Cmake: Available using
```    
    $ apt-get install cmake-gui
```

2. Python 2.7 with Pip:
```        
    $ apt-get install python2.7
```

3. SWIG: Available using:
```    
    $ apt-get install swig
```

4. Boost Libraries 1.61: Available using
```    
    $ apt-get install libbost-all-dev
```

5. A C/C++ Compiler: This is included by default with most Linux systems (gcc), but is necessary to build Basilisk.

## Build Process

First run Cmake with the source directory set to the `{REPO}/src` directory and the build directory set to `{REPO}/build`. For generators, select the one that matches your preferred C++ compiler; this tutorial will assume you have used the built-in GNU compiler (gcc). This tutorial will also assume that you have selected "Unix Makefile."

With the makefile generated, navigate to the `{REPO}/build` directory. Open a terminal, and run `make` to begin the build process. Compiling should take 3-10 minutes depending on the machine.

To test that Basilisk has installed correctly, navigate back to the `{REPO}` directory and run `pytest src/` from the command line. It should show that all tests pass. Also, make sure that in a python interpreter, you can run `import Basilisk` to assure that Basilisk is correctly linked into python's search path for modules.
