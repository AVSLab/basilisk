
.. _FAQ:

FAQ - Frequency Asked Questions
===============================

The following Frequency Answer Questions are general and not operating system specific.

#. Configure `PyCharm <https://www.jetbrains.com/pycharm/>`__ to auto-complete Basilisk commands and provide
   module variables, see the
   `PyCharm Support <https://www.jetbrains.com/help/pycharm/configuring-project-structure.html>`__ web page.

#. How do I run ``pytest`` to ensure all unit and integrated tests still pass

    The following response assumes you have the Basilisk soure code. You need to install the python utility ``pytest`` if this is not available on your system. Instructions are found at :ref:`installOptionalPackages`. Next, navigate to the folder ``Basilisk\src`` and run ``pytest`` from there.

#. How can I run ``pytest`` faster?

    One can distribute python tests across multiple processes. This is achieved with the ``pytest-xdist``
    package using::

       pip3 install pytest-xdist

    After installing this package you can now pytest such that it distribute tests across multi-processes.
    ``pytest`` for 8 processes using::

       python3 -m pytest -n 8

    or replace `8` with either the number of processors (virtual or otherwise) of your host machine, or "auto" to use all
    available processors.

#. How can I used ``pytest`` to generate a Basilisk validation HTML report?

    You will need to install ``pytest-html`` package, see :ref:`installOptionalPackages`.  Then you
    can do this with::

        python3 -m pytest --report

    This generates an HTML report in a local ``tests/report`` folder.

#. How do I perform a clean build of Basilisk?

   IDE’s like X-Code provide a “clean” function. This will remove some compiled code, but in Basilisk it does not get rid of all the SWIG’d code, and there can be compiler warnings related to the last CMAKE settings used.

   To do a basic clean build that will cover most odd cases, you can do this manually by

     - delete the folder of ``dist3`` or ``dist`` and create a new folder with that name
     - follow the regular configure and build instruction in :ref:`configureBuild`

   You can also use the ``clean`` flag in the instruction at :ref:`configureBuild` to do a clean and configure all in one.

   To really have a clean clean build you want to get rid of the `.conan` file that stores the dependencies
   for Basilisk.  To do this you

     - delete the ``.conan`` folder in your home directory
     - follow the regular build instructions in :ref:`configureBuild` using the ``clean`` flag
