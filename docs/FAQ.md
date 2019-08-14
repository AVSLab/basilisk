# FAQ - Frequency Asked Questions  {#FAQ}

The following Frequency Answer Questions are general and not operating specific.


**How do I run `pytest` to ensure all unit and integrated tests still pass**

* Answer: The following response assumes you have the Basilisk soure code.  You need to install the python utility `pytest` if this is not available on your system. Instructions are found at  \ref installOptionalPackages.  Next, navigate to the folder `Basilisk\src` and run `pytest` from there.

**How can I run `pytest` faster?**

* Answer: Glad you asked.  While Basilisk is a single threaded simulation, it is possible to run `pytest` in a multi-threaded manner.  
```
pip install --user pytest-xdist
```
After installing this utility you now run the multi-threaded version of `pytest` for 8 threads using
```
pytest -n 8
```

**How can I build my own version of the HTML Basilisk documentation?**
* Answer: Documentation is critical.  The most up to date version will always be the one build by your copy of Basilisk.  The web site documentation only shows that of the latest tagged release.  To build you own version, you need to use the Doxygen application (http://www.doxygen.org/download.html) or command line tool version.  Run the application with the file 
```
basilisk/docs/DoxyData
```
For the SVG graphics to properly be created, you must install `graphviz` package on your system.  On macOS you can use
```
brew install graphviz
```
to get the `graphviz` tools installed.

**How do I perform a clean build of Basilisk?**
* IDE's like X-Code provide a "clean" function.  This will remove some compiled code, but in Basilisk it does not get rid of all the SWIG'd code, and there can be compiler warnings related to the last CMAKE settings used.  To really have a clean clean build, you can
    * delete the folder of `dist3` or `dist` and create a new folder with that name
    * delete any CMake cache
    * delete the `.conan` directory in your home folder.
* Now when you run CMAke it will pull a fresh copy of any required libraries and proceed to build freshly minted version of Basilisk.