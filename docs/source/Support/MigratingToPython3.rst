
.. _migratingToPython3:

Migrating BSK Scripts to Python 3
=================================

With release Basilisk v0.8.x onward the software framework now supports using Python 3. The purpose of this document is to illustrate how to
migrate Python 2 BSK scripts such that they will function in both Python
3 and Python 2. For the time being Python 2 is still supported as a
depreciated functionality. But, python scripts committed to Basilisk
should be written such that they support Python 3 and 2 for now. This
document serves as compilation of BSK common syntactical adjustments
needed to use Python 3. It is not a comprehensive list of the
differences between Python 2 and Python 3.

Dividing Scalars
----------------

Python 2 and 3 treat the devide operator ``/`` differently if two
integers operated on. Thus::

       a = 3/2

resulted in an integer value of 1 in Python 2, but yields a float value
of 1.5 in Python 3. To get the same result in Python 3 and 2, you can
use either of the following options which work in both version of
Python::

       a = 3//2
       a = 3./2 

Without modification the user will see an error in Python 3 complaining about an unsupported type conversion::

   File "/Users/hp/Documents/Research/Basilisk/dist3/Basilisk/simulation/sim_model/sim_model.py", line 4351, in logThisMessage
       return _sim_model.SimModel_logThisMessage(self, messageName, messagePeriod)
   NotImplementedError: Wrong number or type of arguments for overloaded function 'SimModel_logThisMessage'.
     Possible C/C++ prototypes are:
       SimModel::logThisMessage(std::string,uint64_t)
       SimModel::logThisMessage(std::string)

Returning Lists Instead of Iterables
------------------------------------

Python 3 removed ``iteritems()`` method. The same functionality can be achieved in both Python 2 and 3 with ``items()``.

Range, Map, Zip
---------------

In Python 2 range() returns a list, while in Python 3 it returns an
iterable object. To preserve functionality, cast as a list::

       list(range(x))

Print
-----

Print is treated as a statement in Python 2 and strictly a function in Python 3. For both 3 and 2::

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
