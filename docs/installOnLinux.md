# Installing On Linux {#installLinux}


## Software setup

In order to run Basilisk, the following software will be necessary:

* [Cmake](https://cmake.org/)
* [Python 2.7 (numpy, matplotlib, pytest)](https://www.python.org/downloads/mac-osx/)
* [SWIG](http://www.swig.org/)
* [GCC](https://gcc.gnu.org/)

NOTE: the latest pytest is not compatible with all Basilisk modules. We are still investigating.  Be sure to pull version 3.6.1 for now using
```
pip install pytest==3.6.1
```


## Dependencies


Note: Depending on your system setup, administrative permissions (sudo or su) may be required to install these dependencies. Some distributions of Linux will use other package management commands such as 'yum', 'dnf', of 'pgk'. 


1. CMake: Available using CMake-GUI or CMake over the command line
```
	# GUI installation
    $ apt-get install cmake-gui
    
    # Command line installation
    $ apt-get install cmake
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
    $ apt-get install libboost-all-dev
```

5. A C/C++ Compiler: This is included by default with most Linux systems (gcc), but is necessary to build Basilisk.

## Build Process via Terminal


```
    # Create directory for build and change directory to that
    $ mkdir dist
    $ cd dist

    # CMake here in the build directory with Unix Makefiles, where the source code is located at: '../src'
    $ cmake -G "Unix Makefiles" ../src

    # Can do a multi core make by running 'make -j<number of cores +1>' such as 'make -j5'
    # May take 3-10 minutes depending on the device
    $ make

    # Redirect to src directory where the tests are located
    $ cd ../src/

    # Execute pytest
    $ pytest
```




## Build Process via GUI

First run Cmake with the source directory set to the `{REPO}/src` directory and the build directory set to `{REPO}/dist`. For generators, select the one that matches your preferred C++ compiler; this tutorial will assume you have used the built-in GNU compiler (gcc). This tutorial will also assume that you have selected "Unix Makefile."

With the makefile generated, navigate to the `{REPO}/dist ` directory. Open a terminal, and run `make` to begin the build process. Compiling should take 3-10 minutes depending on the machine.

To test that Basilisk has installed correctly, navigate back to the `{REPO}` directory and run `pytest src/` from the command line. It should show that all tests pass. Also, make sure that in a python interpreter, you can run `import Basilisk` to assure that Basilisk is correctly linked into python's search path for modules.



## Other packages some distributions of Linux may need

```
    # Update current software
    $ apt-get update

    # Helpful for Debian systems -  all packages need to compile such as gcc and g++ compilers and other utils.
    $ apt-get install build-essential

    # Installing the header files for the Python C API
    $ apt-get install python-dev 

    # Package development process library to facilitate packaging Python packages
    $ apt-get install python-setuptools

    # Tkinter
    $ apt-get install python-tk 

    # Python PIP
    $ apt-get install python-pip

    # Python pytest
    $ pip install pytest
```
