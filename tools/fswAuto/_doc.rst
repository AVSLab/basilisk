.. toctree::
   :hidden:

Auto-wrapping Flight Software
-----------------------------

Because Basilisk is written in a combination of C, C++ and Python, when FSW is migrated out of the Basilisk
desktop environment into an embeddable flight target, the Python portion of the architecture needs to be translated
into other programming languages (like C, C++ or MicroPython).
The ``fswAuto`` tools are meant to facilitate the migration of Basilisk flight algorithms
into either: a pure C application (``autosetter``) or a C++ MicroPython application (``autowrapper``).
For further details on migration strategies the reader is referred to
`this paper <https://hanspeterschaub.info/Papers/ColsMargenet2019a.pdf>`_.


``autosetter`` Directory
~~~~~~~~~~~~~~~~~~~~~~~~

The purpose of this script is to automatically translate the setup code from Basilisk
Python scenarios into C.  This facilitates the process to move a BSK simulation of algorithms into
a pure C software architecture.

The setup code encompasses:

    1) C modules' variables initialization and
    2) Grouping of modules in tasks

Run the ``autosetter.py`` file that lives inside the ``autosetter`` directory.  The expected output has
one header and source file containing the setup code written in C will be created in a fresh local
directory named ``outputFiles``.


``autowrapper`` Directory
~~~~~~~~~~~~~~~~~~~~~~~~~
The purpose of this script is to create C++ wrapper classes with setters and getters around each C module.
For instance, autrowrapping the ``inertial3D`` C module will create an ``inertial3D.hpp`` wrapper class.
In addition, source code required to integrate the generated C++ wrappers into MicroPython is also created
(in ``module.hpp``). Note that this glue code is specific to MicroPython.

Run the ``autowrapper.py`` file that lives inside the ``autowrapper.py`` directory to execute an example.
The expected output are the C++ wrapper classes of the C FSW modules and the MicroPython integration/glue code
in a fresh local directory named ``outputFiles``.

``fsw_examples`` Directory
~~~~~~~~~~~~~~~~~~~~~~~~~
This directory contains the FSW simulation defining the modules upon which the ``autosetter.py`` and ``autowrapper.py``
will operate: ``desktopFswSim.py``. All the other scripts in the directory are helper scripts to set up the FSW
simulation.