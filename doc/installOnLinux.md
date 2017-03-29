# Installing On Linux {#installLinux}


These instructions were tested on a Dell XPS15 running Ubuntu 16.04 LTS. 

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
This also installs pip by default. Pytest, matplotlib, and numpy are all available from pip using 
```    
    $ pip install XX
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
First, pull the Basilisk source to a folder named "Basilisk."

Next, run Cmake with the source directory set to the Basilisk directory and the build directory set to Basilisk/Build. For generators, select the one that matches your preferred C++ compiler; this tutorial will assume you have used the built-in GNU compiler (gcc). This tutorial will also assume that you have selected "Unix Makefile."

With the makefile generated, navigate to the Basilisk/Build directory. Open a terminal, and type "make" to begin the build process. Compiling should take 3-10 minutes depending on the machine.

To test that Basilisk has installed correctly, navigate back to the main Basilisk directory and run pytest from the command line. It should show that all tests pass.