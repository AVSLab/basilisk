
.. _migratingToPython3:

Migrating BSK Scripts to Python 3
=================================

Basilisk supports Python 3 only. This document summarizes common changes
needed when migrating legacy Python 2 BSK scripts to Python 3. New and
updated scripts should target the supported Python 3 versions declared by
the project. This is not a comprehensive list of the differences between
Python 2 and Python 3.

Dividing Scalars
----------------

Python 2 and 3 treat the divide operator ``/`` differently if two
integers operated on. Thus::

       a = 3/2

resulted in an integer value of 1 in Python 2, but yields a float value
of 1.5 in Python 3. During migration, choose floor division or
floating-point division explicitly to preserve the intended behavior::

       a = 3//2
       a = 3./2

Without modification the user will see an error in Python 3 complaining about an unsupported type conversion::

   File "/Users/hp/Documents/Research/Basilisk/dist3/Basilisk/simulation/sim_model/sim_model.py", line 4351, in logThisMessage
       return _sim_model.SimModel_logThisMessage(self, messageName, messagePeriod)
   NotImplementedError: Wrong number or type of arguments for overloaded function 'SimModel_logThisMessage'.
     Possible C/C++ prototypes are:
       SimModel::logThisMessage(std::string,uint64_t)
       SimModel::logThisMessage(std::string)

Dictionary Iteration
--------------------

Python 3 removed the ``iteritems()`` method. Replace it with ``items()``.

Range, Map, Zip
---------------

In Python 2 range() returns a list, while in Python 3 it returns an
iterable object. To preserve functionality, cast as a list::

       list(range(x))

Print
-----

Print is treated as a statement in Python 2 and as a function in Python 3.
Use the function-call form::

   print(x)

A sample warning is::

   File "scenarioAttitudeFeedbackRW.py", line 715
       print dataUsReq
                     ^
   SyntaxError: Missing parentheses in call to 'print'. Did you mean print(dataUsReq)?

Strings
-------

External python packages will give warnings in ``pytest`` if
python strings include ‘:raw-latex:`\x`’ where x is not a pythonic valid escape character. These warnings did not appear using Python 2, when using strings as input for latex or for other text processing, they should be made a raw string by appending an r::

   r"..."

A sample warning is::

     /Users/hp/Documents/Research/Basilisk/src/tests/testScripts/../scenarios/scenarioAttitudeFeedbackRW.py:91: DeprecationWarning: invalid escape sequence \o
       label='$\omega_{BR,' + str(idx) + '}$')


Pyswice Imports
-----------------
Changes to BSK module importing has changed the
pyswice importing convention to be completely explicit:

From::

   from Basilisk import pyswice
   pyswice.spkRead(...)

To::

   from Basilisk.pyswice.pyswice_spk_utilities import spkRead
   spkRead(...)
