
Basilisk Module Checkout List 
=============================

This documents contains a series of action items that should be checked
before a Basilisk (BSK) module is approved.

Style and Formatting
--------------------

-  Do the code variables satisfy the :ref:`Basilisk code style
   guidelines <codingGuidelines>`?
-  Are 4 spaces used instead of tabs?

Module Programming
------------------

-  If new message types are declared, are these also Swig’d in
   ``simulation/simMessages/simMessages.i``,
   ``simulation/simFswInterfaceMessages/simFswInterfaceMessages.i`` or
   ``fswAlgorithms/fswMessages/fswMessages.i``
-  Are all module input and output messages Swig’s in the module ``*.i``
   file
-  Does the code contain appropriate comments
-  Does the code contain Doxygen compatible function descriptions,
   variable defintions, etc.
-  Module startup and initialization

   -  The ``SelfInit()`` routine should declare the module output
      messages
   -  The ``CrossInit()`` routine should subscribe to the module input
      messages
   -  The ``Reset()`` in the FSW modules should reset all the default
      moudle configuration parameters.

Module Functionality Testing
----------------------------

Is a \_UnitTest folder included that:

-  has the python file name start with\ ``test_``
-  only uses the test module (if possible), and creates the various
   required input messages
-  checks the module output for all input and module configuration
   conditions
-  performs the validation checking against either custom-computed
   results in other programs (Matlab, Mathematica, hand calculation),
   live Python computed results, or against expected simulation data
   (consistency checking)
-  can also be run with the python command instead of pytest (by
   updating the ``__main()__`` function at the bottom of the python
   file)
-  if the module depends on other modules, clear out teh ``*.pyc`` files
   in ``utilities/``

Module Documentation
--------------------

Does the module ``*.h`` file contain doxygen code that provides the module documentation?  A sample can be found in the ``fswTemplateModule`` folder.

Module Integrated Test
----------------------
If an integrated test is provided as a ``test_XXX.py`` file, does the text function include the expected validation setup, assumption and results documentation within the method doc-string?  A sample can be found in the ``fswTemplateModule`` folder.


