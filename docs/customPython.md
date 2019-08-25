# Using a Custom Python Installation {#customPython}


The following instructions are guidelines on how to run Basilisk with a computer that is not using a system installed version of Python. First some general notes:

* Basilisk must be built and run with the same Python binary.
* For Unix platforms Cmake should select the default installed version of Python.

## Using the CMake GUI Application
Should you want to use a different Python installation, such as Python installed/managed by Homebrew for Mac, follow these instructions.

1. In Cmake ensure the “Advanced” option check box is checked. 
2. Configure the project. Selecting the “Advanced” check box will display the Python path options for the build.
3. Expanding the “PYTHON” option on the configure options window (screenshot 1) shows the Python options with default settings (on Mac OS X) (screenshot 2).
4. Click to select and change the path for each of these options (screenshot 3) as has been done to use the Homebrew installed Python.

It its important to note that if you plan to run a simulation scenario from the command line that your command line path must include the path to the same Python binary that has been used to build the project with Cmake. 


The following screen shots are taken on macOS using the CMake.app application:

\image html Images/doc/customPython1.png "Screenshot 1"

\image html Images/doc/customPython2.png "Screenshot 2" 

\image html Images/doc/customPython3.png "Screenshot 3" 


## Using the Command Line CMake Command
To run cmake with the custom python installation that is not found by the system by default, the path to the python installation can be provided as a command line argument.  Using a BASH shell and from within `dist3` folder, to compile with Python 3 you can use
```
cmake ../src -G "Xcode" -DPYTHON_LIBRARY=$(python3-config --prefix)/lib/libpython3.7.dylib -DPYTHON_INCLUDE_DIR=$(python3-config --prefix)/include/python3.7m -DUSE_PYTHON3=ON -DPYTHON_EXECUTABLE=$(python3-config --exec-prefix)/bin/python3.7
```
Note that the above command didn't work from a `tcsh` shell environment.  The `python3-config` command and package allows for easy configuring of what python to use.