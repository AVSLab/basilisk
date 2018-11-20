# Installing On macOS {#installMacOS}

These instruction outline how to install Basilisk (BSK) on a clean version of macOS.  There are two methods presented.  The first method makes use of the macOS provided version of Python 2 while all python packages are installed in the user Library.  The second method shows how to use HomeBrew to install python.  This method provides more control over what version of python is being run. Basilisk only runs with Python 2.


## Developer Software Tools 

In order to run Basilisk on macOS, the following software is necessary:

1. Get the [Apple Xcode Developer](https://itunes.apple.com/us/app/xcode/id497799835?mt=12) tool from the App Store
    * After Xcode is installed, start up the program to let it finish installing development components
    * Open a Terminal window to install the command line tools using
```
$ xcode-select --install
```
2. Get the [CMake](http://cmake.org) application to be able to create the Xcode IDE file
3. (Optional) Get the [SourceTree](http://sourcetreeapp.com) application to be able to pull and manage a copy of Basilisk
4. (Optional) Get the [PyCharm](https://www.jetbrains.com/pycharm/) application to be able to edit python source files


## Install HomeBrew Support Packages
1. Install [HomeBrew](http://brew.sh) using a Terminal window and pasting the install script from the HomeBrew web site.  
    * *Note:* that this must be done within a `bash` terminal window.  The type of terminal emulation is shown on the top of the terminal window.  If you are running another terminal type, type `bash` to engage the Bash terminal environment.  This is just required to install HomeBrew.  Once it is installed, you can run all other commands from any terminal type.
2. Install the SWIG package using
```
$ brew install swig
```
  

## Setting up the Python Environment
The following setup will make use of the macOS supplied version of python, but will install all the required python packages in the user `~/Library/Python` folder.  This has the benefit that no `sudo` command is required to install and run Basilisk, and the user Python folder can readily be replaced if needed.  If you wish to use the HomeBrew version of python, configure the CMake Python paths as described in \ref customPython after following these instructions.

In the following instructions, be sure to follow the sequence of tasks as outlined below.

### Setting the `PATH` Environment Variable
As this installation will install all required Python packages in the user home directory `Library/Python` folder, the `PATH` variable must be setup within the terminal environment.  

1. Open a terminal window
2. Use the `nano` text editor to edit the setup file via steps 3 through 6 below
3.  If using a Bash shell, then 
    * type 
```
$ nano ~/.bash_profile
```
    * Add the line 
```
export PATH=~/Library/Python/2.7/bin:$PATH
```
4. If using a tcsh shell, then
    * type
```
$ nano .tcshrc
```
    * Add the line 
```
set path = ( ~/Library/Python/2.7/bin $path )
```
5. If you are using the Homebrew version of python, then add the path `/usr/local/bin` before the other paths.  If you type in a new Terminal window `which python` you should see a path to the Homebrew installation of python, most likely `/usr/local/bin/python`
6. Save and close the file
7. Open a new terminal window for the path to take effect

**Note:**
We suggest you remove any other python packages (such as Anaconda), or change the path in your terminal shell if you really want to keep it.

### Setup Required Python packages
* From the terminal window, enter the following commands:
```
$ easy_install --user pip
$ pip install --user --ignore-installed setuptools
```
* Copy the file called [`mac_fix_path.pth`](mac_fix_path.pth) from basilisk/docs to the directory `~/Library/Python/2.7/lib/python/site-packages/` 
For more information about this file see this [online discussion](https://apple.stackexchange.com/questions/209572/how-to-use-pip-after-the-os-x-el-capitan-upgrade/209577). **Note:** If you have installed python packages already using `sudo pip install`, then these are stored in `Library/Python/2.7/site-packages`.  You need to add the `mac_fix_path.pth` file to this folder as well to make macOS ignore the system installed packages.  Or, to only use home directory installed python packages, just remove `Library/Python` folder.
* From the terminal window, install the required general Python packages using:
```
$ pip install --user numpy
$ pip install --user matplotlib
```








## Cloning and Building the Basilisk Project

When all the prerequisite installations are complete, the project can be built as follows.

1. If needed, create your own [bitbucket.org](http://bitbucket.org) account
2. Use a browser to go to the [Basilisk Bitbucket Repository](https://bitbucket.org/avslab/basilisk)
3. Click on the Clone button on this page, and select the `https` option instead of the ssh option
4. Copy the project url (omit `git clone`) from the bitbucket clone panel
\image html Images/doc/bitbucket-clone-panel.png width=514px

5. Clone into preferred Git client (Source Tree for instance), or just clone the repository in the directory containing Basilisk.  In SourceTree, use `clone from url`, add the Basilisk repository url (without `.git` on the end), and select `develop` branch to pull the latest code.
\image html Images/doc/sourcetree-clone-panel.png width=568px



6. Open Cmake to create the build files
    
    \image html Images/doc/3046062966-cmake.png width=489px

    * Click on browse Source, and select the source directory, the Basilisk repository that you just cloned

    * Browse and select the build directory (`basilisk/dist/`). If this directory does not exist, create it.

    * Press `Configure` in Cmake, select the Xcode IDE if running for the first time.  <br>
    **Note:** If you wish to use the HomeBrew version of python  configure the Python paths in \ref customPython<br>
    **Potential Issue:** If you get an error message in CMake saying it can't find the compiler tools, open a Terminal window and type
```
    $ xcode-select -p
```
    This should return 
```
    /Applications/Xcode.app/Contents/Developer
```
    If instead you get a different director such as `/Library/Developer/CommandLineTools`, then correct this compiler directory path using 
```
    sudo xcode-select --reset
```
    Now clear the Cmake cache and try running `Configure` again.

    * Press `Generate` in Cmake to build the Xcode Basilisk project file inside the `dist` directory

7. Open the Xcode file `dist/basilisk.xcodeproj`

    * The source code should appear and be ready for use
    \image html Images/doc/256564102-xcode.png width=419px
    
    * You can now build the project within the Xcode IDE

8. To test your setup you can run one of the scenario scripts.
    * In the terminal window, make `basilisk/src/tests/scenarios` the current directory.
    * Run one of the tutorial scenarios, such as 
```
    $python scenarioBasicOrbit.py
```

## Optional Packages
The above direction install the Basilisk base software.  There are a series of \ref installOptionalPackages "optional packages" and software installs that enhance this capability, including `pytest` to run an automated test suite of unit and integrated tests.



##Power User Installation Tip  

The project can be configured and built from the command line via CMake.  Command line operations are run using the following setup.py script and parameters.
~~~~~~~
python setup.py <command_1 command_2 etc.>
~~~~~~~
clean: removes 'dist/build' and build artifacts.

cmake: configure the project and generate an XCode project file to the 'dist' directory. This parameter also installs Basilisk on your local path to be found along with other Python packages. This requires CMake command line version to be installed.

xcode: execute the project build scheme with XCode where the generated Basilisk python package is output to 'dist/'.

test: run pytest on the Basilisk project. This parameter takes additional user options, via 'pytest-args=' to pass through to pytest.

docs: build the documentation with doxygen. The generated html documentation is found in `src/html`. This requires doxygen command line version to be installed.

## FAQs

1. Q: swig not installing

    * A: Make sure you have pcre installed (using brew install preferably)

2. Q: Experiencing problems when trying to change the directory in which to clone the url

    * A: clone it in the default directory, and copy it into the preferred one after it is done cloning.

3. Q : Trouble configuring in Cmake

    * A: When configuring for the first time, select the appropriate platform for the project (Xcode for instance)

4. Q : Permission denied when using brew

    * A: Add sudo to the start of the command. If you do not have superuser access, get superuser access.

5. Q : Python unexpectedly quit when trying to run pytest

    * A: Check the python installation. If the path is wrong, uninstall, and reinstall python using brew.

6. Q : I updated my macOS system to the latest released, and I can no longer run CMake or build with Xcode.

    * A: Most likely you just need to reset CMake.app to use the latest macOS information. In CMake.app, select File/Delete Cache, and then run Configure again.  The application will ask you to confirm the use of the latest macOS and Developer tools.  
