# Installing With Cmake Configuration Options {#cmakeOptions}

This documents discusses the various Cmake build options that can be used when compiling Basilisk.  These build options can be set via the GUI Cmake program or specified via the command line.

\image html Images/doc/cmake-no-use.png "Screen Shot Showing Optional CMake Basilisk Flags" width=500px 

## Optional Cmake Flags
The Basilisk CMake file contains a series of flags that can be turned on to include additional features within Basilisk or to build in a particular configuration.  The flags are listed in the above image.  Note that tuning on some flags will invoke the `conan` package manager to download and install additional dependencies.  Basilisk modules that require particular dependencies will not be compiled unless their support software is installed.  Associated BSK module unit tests are skipped if the required software is not provided through these Cmake flags.

The possible flags are quickly summarized:
* `USE_COVERAGE` turns on code coverage when compiling with GCC on a Linux platform
* `USE_OPENCV` installs the [OpenCV](https://opencv.org) package via `conan`.
* `USE_PROTOBUFFERS` installs the [Google protobuffers](https://developers.google.com/protocol-buffers/) package via `conan`.
* `USE_ZMQ` installs the [0MQ distributed messaging system](http://zeromq.org) via `conan`.




## Simplest Basilisk Build
The bare-bones version of Basilisk is invoked by not turning on any Cmake flags as shown .  In the GUI Cmake all the optional compiler flags shown be de-selected as shown in the above figure.  To use the command line `cmake` command use the regular Basilisk set as discussed in the platform specific Basilisk installation files.


## Building to Support Vizard Basilisk Playback
To include the `vizInterface` module that enables saving off and communicating with the Vizard Unity based visualization, the two flags `USE_PROTOBUFFERS` and `USE_ZMQ` must be turned on as shown in the following CMake GUI screen show.
\image html Images/doc/cmake-Vizard.png "Screen Shot Showing CMake with Vizard Settings" width=500px 
To use the command line `cmake` command use the regular Basilisk compile tool with the additional arguments:
```
-USE_ZMQ=ON -USE_PROTOBUFFERS=ON
```


