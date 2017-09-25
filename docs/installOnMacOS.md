# Installing On macOS {#installMacOS}

In this basic tutorial, we'll go over the Software needed to run Basilisk, the installation process, how to build the process in the prefered IDE, and a few FAQs.

## Software setup

In order to run Basilisk, the following software will be necessary:

* [Homebrew](http://brew.sh/)
* [Cmake](https://cmake.org/)
* [Python 2.7 (numpy, matplotlib, pytest)](https://www.python.org/downloads/mac-osx/)


## Suggested IDEs

Aside from this necessary software, the user can decide on what IDEs are prefered, the following are suggested:

* PyCharm for python development - Use the default installation options
* Xcode for C and C++ 

## Installing

Now to install the needed software, we will need a command line window.
In a command line window, execute the following commands:

```
$ brew install python 
$ brew install pcre
$ brew install swig
```
```
$ sudo easy_install pip
$ pip install numpy
$ pip install matplotlib
$ pip install pytest
```

Note:
We suggest you remove any other python packages (such as Anaconda), or change the path in your .bash/profile (usr) if you really want to keep it

## Building the project

When all the installations are complete, the project can be built. 

If a Basilisk release is being used, download it directly from http://hanspeterschaub.info using provided username and passwords. Once downloaded, skip to bullet 3.

If creating a repository for Basilisk add-on development, start here:

1. Copy the project url from the bitbucket clone
\image html Images/doc/930895969-clone.png width=651px

2. Clone into prefered Git client (Source Tree for instance), or just clone the repository in the directory containing Basilisk 

3. Open Cmake
\image html Images/doc/3046062966-cmake.png width=489px

4. Click on browse Source, and select the source directory : the Basilisk repository that you just cloned 

5. Browse and select the build directory (Basilisk/build). If this directory does not exist, create it.
\image html Images/doc/219348025-confgen.png width=486px

6. Configure in Cmake 

7. Generate in Cmake and select the IDE you are using (Xcode for instance)

8. Open the IDE for which you built the project or navigate to the "Build" directory and then open the project file. 

9. The source code should appear and be ready for use 
\image html Images/doc/256564102-xcode.png width=419px


10. You can now build the project within the IDE 

11. To test your setup, run pytest in your root Basilisk directory (\Basilisk, not \Basilisk\Build.) You should see positive test results. 

## FAQs

1. Q: swig not installing 

    * A: Make sure you have pcre installed (using brew install preferably) 

2. Q: Experiencing problems when trying to change the directory in which to clone the url

    * A: clone it in the default directory, and copy it into the prefered one after it is done cloning.

3. Q : Trouble configuring in Cmake

    * A: When configuring for the first time, select the appropriate platform for the project (Xcode for instance)

4. Q : Permission denied when using brew

    * A: Add sudo to the start of the command. If you do not have superuser access, get superuser access.

5. Q : Python unexpectedly quit when trying to run pytest

    * A: Check the python installation. If the path is wrong, uninstall, and reinstall python using brew.

6. Q : I updated my macOS system to the latest released, and I can no longer run CMake or build with Xcode.

    * A: Most likely you just need to reset CMake.app to use the latest macOS information. In CMake.app, select File/Delete Cache, and then run Configure again.  The application will ask you to confirm the use of the latest macOS and Developer tools.  
