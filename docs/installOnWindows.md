# Installing On Windows {#installWindows}

The following was developed using Windows 7, Python 2.7.12 release candidate 1, Visual Studio Community 2015 and Boost C++ libraries 1.61.

## Software setup

In order to run Basilisk, the following software will be necessary:

* [Cmake](https://cmake.org/)
* [Python 2.7 (numpy, matplotlib, pytest)](https://www.python.org/downloads/mac-osx/)

NOTE: the latest pytest is not compatible with all Basilisk modules. We are still investigating.  Be sure to pull version 3.6.1 for now using
``` 
    pip install pytest==3.6.1 
```


## Configuration
Decide whether target deployment is 32 (win32) or 64 (x64) bit. Which ever chosen you will need the matching python and software for that architecture.

### Configuring Python

Python is installed using the Windows installer found on the Python website. Make sure to choose the correct installer for your architecture. The Additional required Python packages (found in the [readme](@ref README)) are installed using the Python package manager pip (`pip.exe`) which comes default with the Windows Python installer. To install additional Python packages with pip the following command is executed at command line.

```
C:\Users\patrick> pip --trusted-host=pypi.python.org install <package name>
```


### Configuring Boost

If you require a different version of Boost than that provided with the project you may download and build the Boost libraries. To download the version of the Boost C++ libraries noted in the project [readme](@ref README). Executing the following commands from within the Boost directory will build all Boost libraries. Again selecting the appropriate architecture (32 or 64 bit) and ensure the runtime linkage parameter is set to 'static'.

```
C:\Users\patrick\Downloads\boost_1_61_0\boost_1_61_0> bootstrap
C:\Users\patrick\Downloads\boost_1_61_0\boost_1_61_0> b2 address-model=64 runtime-link=static
```


### How to run tests

Tests are run and managed by Pytest. To execute all tests the py.test command can be executed on the `src` directory from the command line.

```
C:\Users\patrick\Documents\basilisk> py.test src/
```
