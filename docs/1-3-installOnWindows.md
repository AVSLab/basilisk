# Installing On Windows {#installWindows}

The following was developed using Windows 7, Python 2.7.12 release candidate 1, Visual Studio Community 2017.

## Software setup

In order to run Basilisk, the following software will be necessary:

* [Cmake](https://cmake.org/)
* [Python 2.7 (numpy==1.15.4, matplotlib, pytest, conan, pandas)](https://www.python.org/downloads/windows/)
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

The standard windows swig version 3.0.12 setup is suitable for Basilisk (see [Configuration Instructions](http://www.swig.org/Doc1.3/Windows.html#Windows_swig_exe)).  Note that swig version 4.0 is not currently compatible with Basilisk.

Example added path formats:

```
PYTHON_INCLUDE = C:\Program Files\Python27\include 
PYTHON_LIB = C:\Program Files\Python27\libs\python27.lib
```

#### Install Conan
Go to the [Conan Website](https://conan.io/downloads.html) and download the windows installer. Proceed with installation. If it asks to be added to the PATH, allow it to add itself to the PATH.

## Installing

#### Setup
From Basilisk root directory: 

Python 2:
```
mkdir dist $$ cd dist
```
Python 3:
```
mkdir dist3 $$ cd dist3
```
#### Configuration and Build: 

Python 2:
```
cmake -G "Visual Studio <MSVC Version> <MSVC Product Year> Win<arch>" ../src -DCMAKE_BUILD_TYPE=Debug -DUSE_PROTOBUFFERS=OFF
cmake --build . --target ALL_BUILD --config Release
```
Python 3: 
```
cmake -G "Visual Studio <MSVC Version> <MSVC Product Year> Win<arch>" ../src -DCMAKE_BUILD_TYPE=Debug -DUSE_PROTOBUFFERS=OFF -DUSE_PYTHON3=ON
cmake --build . --target ALL_BUILD --config Release
```
Example command using x86:
```
cmake -G "Visual Studio <MSVC Version> <MSVC Product Year> Win32" ../src -DCMAKE_BUILD_TYPE=Debug -DUSE_PROTOBUFFERS=OFF
```

MSVC Mapping:
| MSVC Product Year | MSVC Version |
|-------------------|--------------|
| 2019              | 16           |
| 2017              | 15.9         |
|                   | 15.8         |
|                   | 15.7         |
|                   | 15.6         |
|                   | 15.5         |
|                   | 15.4 - 15.3  |
|                   | 15.2 - 15.0  |
| 2015              | 14           |
| 2013              | 12           |
| 2012              | 11           |
```
Example build commands:  
  
Arch x86, MSVC Year 2017, MSVC Version 15:
```
cmake -G "Visual Studio 15 2017 Win32" ../src
```
Arch x64, MSVC Year 2019, MSVC Version 16:
```
cmake -G "Visual Studio 16 2019" -A x64 ../src -DCMAKE_BUILD_TYPE=Debug -DUSE_PROTOBUFFERS=OFF
```
```
cmake -G "Visual Studio 15 2017 Win64" ../src -DCMAKE_BUILD_TYPE=Debug -DUSE_PROTOBUFFERS=OFF
```
## How to run tests

Tests are run and managed by Pytest. To execute all tests the py.test command can be executed on the `src` directory from the command line. 
```
python -m pytest src/ 
```
