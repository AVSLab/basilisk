

.. _FAQwindows:

FAQ: Microsoft Windows
======================

The following Frequency Answer Questions are specific for the Microsoft Windows operating system.


#. I'm having trouble running ``pytest`` on Windows from within the ``src`` directory.


    It is recommended to run the ``pytest-xdist`` version which allows multi-threaded executions using ``pytest -n 8``
    for 8 threads, for example.  See :ref:`installOptionalPackages` for more information on installing ``pytest-xdist``.

