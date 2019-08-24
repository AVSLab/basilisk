# FAQ: macOS {#FAQmacOS}

The following Frequency Answer Questions are specific for the macOS operating system.


**How do I get PyCharm to auto-complete Basilisk commands?**

* Answer: To auto index all of BSK python modules to have code completion for BSK in PyCharm, go to Preferences >> Project >> Interpreter, click the ellipsis near the interpreter drop down and select more, click the paths icon, the right most icon shown
\image html Images/doc/FAQmacOS1.png width=153px
Add the path to the “modules” directory.  You should now be able to use auto-completion in PyCharm.

**I upgraded my Mac operating system and now cannot use 'axes3D' from matplotlib. How can I fix this?**

* Answer: Mac OS has and manages its own install of Python. This python install is reportedly used by the OS to perform various administrative tasks. As a result if you are using the default Python install on Mac OS then any OS update (e.g. Yosemite to Sierra) may result in updated and installed packages (via pip or some other mechanism) to be over written in /System/Library/Frameworks/Python.framework/Versions/2.7/Extras. To update these packages one must run the ‘pip’ executable associated with the particular python install. For example to update a package used by the Mac OS system Python install:
	
        sudo /System/Library/Frameworks/Python.framework/Versions/2.7/bin/python2.7 -m pip install matplotlib --upgrade --ignore-installed six
    
This ensures that the 'pip' mechanism used to install the package is the one associated the default Python install you are using.
    	
In the case that you are using a 3rd party Python install such as Homebrew you must use the 'pip' binary associated with that install. There exists a ‘usr/local/lib/python/2.7/site-packages’, however this is used by a Homebrew install of python/pip. More over ‘usr/local/lib/python/2.7/site-packages’ does not appear on the sys.path list of paths for the default (usr/bin/python) install but it does for a Homebrew install /usr/local/Cellar/python/2.7.13/Frameworks/Python.framework/Versions/2.7/bin/python2.7.

**I am switching between the macOS supplied version of Python and the Homebrew installed version.  I have the correct Python paths in the CMake and BSK compiles. I can run BSK scripts via `python` command, but using `pytest` fails.  What could be going on?**

* You many need to uninstall pytest and reinstall it after switch the python binary source.  Run 

        pip uninstall pytest
then run

        pip install --user pytest==3.6.1


### Take aways:
- One must use the pip associated with the python install for which you want to adjust the packages. This can be done by running “python2.7 -m pip” where “python2.7” can be replaced by any python binary desired.
- The site packages/third party packages for the Mac OS default Python install are located separate to those of the Homebrew install.
- Mac OS Sierra requires an updated Matplotlib to use mpl_toolkits.

