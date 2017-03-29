# README  {#README}

### Basilisk ###

* [Summary of Basilisk](@ref cover)
* [Versions](http://hanspeterschaub.info/bskReleaseNotes.html)
* [Basilisk Wiki](https://bitbucket.org/avslab/basilisk/wiki/Home) (only available to Basilisk developers)

### How do I get set up? ###
Of the following requirements Python is the only one for which prebuilt libraries are not included in the project repository:

* [Cmake](https://cmake.org) for configuring the and generating the Visual Studio solution file or macOS XCode proejct.
* [Python 2.7](https://www.python.org/downloads/windows/) (numpy, matplotlib, pytest)

[Setup a macOS Development Environment](@ref installMacOS)

[Setup a Linux Development Environment](@ref installLinux)

[Setup a Windows Development Environment](@ref installWindows)


### Basilisk Development guidelines ###

* [Setup a Linux Development Environment](@ref codingGuidelines)
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)


### Getting Started
To get started with Basilisk (BSK), several tutorial python files are found in the SimScenario folder.  Within this web page documentation site, they are listed and discussed in the <a href="modules.html">Modules</a> tab.  To play with the tutorials, it is suggested the user makes a copy of the `test_scenarioXXX.py` file they are testing to learn, test and experiment.  The documentation lists the scenarios in an order that facilitates learning basic BSK features. For example, a good start would be to run `test_scenarioBasicOrbit.py`.

Note that to run this pythons simulation script from outside the Basilisk directory, which is recommended, the local path to the Basilisk installation must be edited through the `bskPath` variable.

Any new BSK module development should not occur within the BSK folder as this will be updated rapidly.  Rather, new FSW algorithm or simulation coce modules should be created in a custom folder outside of the BSK directory.  A sample folder is provided named `BasiliskCustom` wich contains sample FSW and Simulation modules.

To use the standalone 3D Visualization, download the Basilisk Qt code.  This is more experimental, but does provide a 3D view of many of the simualtion states.  The Viz and BSK simulation talk through TCP/IP.  After opening the Viz, use the `open connection` or `CMD-O` command to open the default 127.0.0.1 address.  In the `test_scenarioXXX.py` script, uncomment the marked 3 lines to enable a communication interface with the Viz.


### Who do I talk to? ###

- <mailto:basilisk-info@colorado.edu> - Use this email to request information about Basilisk, ask to be added as a test user, or provide suggestions
- <mailto:basilisk-bug@colorado.edu> - Use this email to send us info on bugs or issues.