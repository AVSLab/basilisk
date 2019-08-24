# Using a Custom Python Installation {#customPython}

The following instructions are guidelines on how to run Basilisk with a computer that is not using a system installed version of Python. First some general notes:

* Basilisk must be built and run with the same Python binary.
* For Unix platforms Cmake should select the default installed version of Python.

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


