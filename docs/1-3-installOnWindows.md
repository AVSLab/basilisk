# Installing On Windows {#installWindows}

The following was developed using Windows 7, Python 2.7.12 release candidate 1, Visual Studio Community 2015 and Boost C++ libraries 1.61.

## Software setup

In order to run Basilisk, the following software will be necessary:

* [Cmake](https://cmake.org/)
* [Python 2.7 (numpy==1.15.4, matplotlib, pytest)](https://www.python.org/downloads/mac-osx/)
* [pip](https://pip.pypa.io/en/stable/installing/)
* Visual Studios 15 or Greater
* [Swig](http://www.swig.org/download.html)



## Configuration
Decide whether target deployment is 32 (win32) or 64 (x64) bit. Which ever chosen you will need the matching python and software for that architecture.

#### Configuring Python

Python is installed using the Windows installer found on the Python website. Make sure to choose the correct installer for your architecture. The Additional required Python packages (found in the [readme](@ref README)) are installed using the Python package manager pip (`pip.exe`) which comes default with the Windows Python installer. To install additional Python packages with pip the following command is executed at command line.

```
C:\Users\patrick> pip --trusted-host=pypi.python.org install <package name>
```


#### Configuring Swig

The standard windows swig setup is suitable for Basilisk. [Configuration Instructions](http://www.swig.org/Doc1.3/Windows.html#Windows_swig_exe).

Example added path formats:

* PYTHON_INCLUDE = C:\Program Files\Python27\include
* PYTHON_LIB = C:\Program Files\Python27\libs\python27.lib


#### Install Conan
Go to the [Conan Website](https://conan.io/downloads.html) and download the windows installer. Proceed with installation. If it asks to be added to the PATH, allow it to add itself to the PATH.

## Installing

From Basilisk root directory:
```
mkdir dist $$ cd dist
```
Configure and Build:
```
cmake -G "Visual Studio 15 2017 Win64" ../src -DCMAKE_BUILD_TYPE=Debug -DUSE_PROTOBUFFERS=OFF
cmake --build . --target ALL_BUILD --config Release
```
For arch x86:
```
cmake -G "Visual Studio 15 2017 Win32" ../src
```

### How to run tests

Tests are run and managed by Pytest. To execute all tests the py.test command can be executed on the `src` directory from the command line.

```
C:\Users\patrick\Documents\basilisk> py.test src/
```
