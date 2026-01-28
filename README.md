# README

## Basilisk

* [Summary of Basilisk](docs/source/index.rst)
* [Release Notes](docs/source/Support/bskReleaseNotes.rst)

### Installation

Basilisk can be installed in two ways, either from PyPI or by building from
source.

For most users, installing from PyPI is the easiest and fastest way to get
started. Building from source is recommended if you need to link to external
C++ modules or want to customize the build configuration.

#### Install from PyPI

The easiest way to get started with Basilisk is to install the prebuilt wheel
from [PyPI](https://pypi.org/project/bsk/):

```bash
pip install bsk
```

This installs the latest stable version with all standard features
(e.g. optical navigation and MuJoCo). See the [install](docs/source/Install.rst)
docs for supported platforms and additional details about the wheels.

#### Build from Source

If you need to use external C++ modules or want to customize the build, follow
the platform-specific setup instructions:

* [Setup a macOS Development Environment](docs/source/Build/installOnMacOS.rst)

* [Setup a Linux Development Environment](docs/source/Build/installOnLinux.rst)

* [Setup a Windows Development Environment](docs/source/Build/installOnWindows.rst)

See the [Build from Source docs](docs/source/Build.rst) for full details.

### Basilisk Development guidelines

* [Contributing](CONTRIBUTING.md)
* [Coding Guidelines](docs/source/Support/Developer/CodingGuidlines.rst)

### Getting Started

To get started with Basilisk (BSK), several tutorial python files are provided in the installed package.  Within this
web page documentation site, they are listed and discussed in the [integrated example script](docs/source/examples/index.rst) page.

The documentation lists the scenarios in an order that facilitates learning basic BSK features. The python scripts
are stored under `basilisk/examples`. A good start would be to run `scenarioBasicOrbit.py`.

To play with the tutorials, it is suggested the user makes a copy of these tutorial files, and use the copies in order
to learn, test and experiment. Copy the folder `basilisk/examples` into a
new folder, and change to that directory.
To run the default scenario of `scenarioBasicOrbit`, in the directory of the
copied tutorials, execute the python script: `python scenarioBasicOrbit.py`

Now, when you want to use a tutorial, navigate inside that folder, and edit and execute the *copied* integrated tests.

Any new BSK module development should not occur within the BSK folder as this will be updated rapidly.  Rather,
new FSW algorithm or simulation code modules should be created in a custom folder outside of the BSK directory.
See the [building custom modules](docs/source/Build/buildExtModules.rst) web page
for more information.

To use the standalone 3D Visualization, download the [Vizard](docs/source/Vizard/VizardDownload.rst).
This is in development, but does provide a 3D view of many of the simulation states.

### Who do I talk to?

Questions and answers are fielded in the project's [Github Discussions](https://github.com/AVSLab/basilisk/discussions).
