

.. _FAQwindows:

FAQ: Microsoft Windows
======================

The following Frequency Answer Questions are specific for the Microsoft Windows operating system.


#. I'm having trouble running ``pytest`` on Windows from within the ``src`` directory.


    It is recommended to run the ``pytest-xdist`` version which allows multi-threaded executions using ``pytest -n 8``
    for 8 threads, for example.  See :ref:`installOptionalPackages` for more information on installing ``pytest-xdist``.

#. I have upgraded to Basilisk version 1.8.0 or newer and I'm following the revised build process.  However,
   I'm seeing warnings in the terminal window saying I don't have the correct python packages installed
   when I know that I do, and error messages about ``conan`` commands.

    You are likely installed ``conan`` using the binary download link from the conan web site.  This
    installs a self-contained binary, including it's own copy of Python.  With 1.8.0 and higher it is recommended
    to avoid installing the conan binary and using ``pip install conan`` instead.  This ensures that the conan,
    and associated support packages, are installed for the particular version of python that you want to run.