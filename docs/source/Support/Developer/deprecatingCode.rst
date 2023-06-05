.. _deprecatingCode:

Deprecating code in Basilisk
============================

Motivation
----------
The nature of a fast evolving software such as Basilisk is that systems are consistently improving, many times making older functionality obsolete. 
Thus, we face the challenge of handling older code while we move towards the new systems. 
We cannot simply remove old functionality, as we don't want user code to break overnight. 
Instead, we enter a phase of deprecation, when we warn users about the use of deprecated code, 
but otherwise continue to allow it and support it. After enough time has passed for our users to update 
their code, the deprecated functionality can be removed.

This support page explains the different mechanisms we have available in Basilisk to mark code as deprecated.
Deprecated code will cause deprecation warnings to pop-up in the user's console, but otherwise it should
work as expected.

Deprecating Python Code
-----------------------
For code that is entirely defined in Python, we can make use of the utility decorators and descriptors
defined in ``Basilisk.utilities.deprecated``. This section shows how to deprecate functions, classes, attributes,
and properties.

To illustrate this functionality, let's imagine the following code:

.. code-block:: python

    def standaloneFun(arg):
        ...

    class MyClass:

        def __init__(self):
            self.myAttribute = 0
            self._myPropertyInner = 0

        def myFun1(self):
            ...

        @property
        def myProperty(self):
            return self._myPropertyInner * 2
        
        @myProperty.setter
        def myProperty(self, value: int):
            self._myPropertyInner = value / 2

There is a standalone function ``standaloneFun``, a class ``MyClass`` with two attributes
``myAttribute`` and ``_myPropertyInner``, a class method ``myFun1``, and a property ``myProperty``
with a getter and setter (which makes use of the ``_myPropertyInner`` private attribute).

If we want to deprecate the **standalone function** and the **class method**, the syntax is
as follows:

.. code-block:: python

    from Basilisk.utilities import deprecated

    @deprecated.deprecated("2099/05/05", "Use standaloneFun_new() instead!")
    def standaloneFun(arg):
        ...

    class MyClass:

        ...

        @deprecated.deprecated("2000/05/05", "myFun1 is super old!")
        def myFun1(self):
            ...

        ...

The first argument to ``@deprecated.deprecated`` must be a string with the date when the function is expected
to be removed (as a rule of thumb, between 6 to 12 months after the release of
the deprecated code). The second argument is a message that is shown directly
to users. Here, you may explain why the function is deprecated, alternative functions, 
links to documentation or scenarios that show how to translate deprecated code...

If you want to deprecate a **class**, then use:

.. code-block:: python

    from Basilisk.utilities import deprecated

    @deprecated.deprecated("2099/05/05", "This entire class is replaced by MyOtherClass")
    class MyClass:
        ...

This is the same syntax as deprecating functions, and the arguments behave in the same way.

If you want to deprecate an **attribute**, that is, a class variable, then do:

.. code-block:: python

    from Basilisk.utilities import deprecated

    class MyClass:
        
        myAttribute = deprecated.DeprecatedAttribute(
            "2099/05/05", "myAttribute is no longer used in the simulation"
        )

        def __init__(self) -> None:
            with deprecated.ignore("myAttribute"):  # Prevents warnings here
                self.myAttribute = 0

            ...

The input arguments to ``deprecated.DeprecatedAttribute`` are the same as the arguments
for ``deprecated.deprecated``. Note that if you want to initialize the attribute to
some variable (or otherwise manipulate it in any way) without raising deprecation
warnings, you should use the ``deprecated.ignore`` context manager.

Finally, if you need to deprecate a **property**, then use:

.. code-block:: python

    from Basilisk.utilities import deprecated

    class MyClass:
        
        @property
        def myProperty(self):
            return self.myPropertyInner * 2

        @myProperty.setter
        def myProperty(self, value: int):
            self.myPropertyInner = value / 2

        myProperty = deprecated.DeprecatedProperty(
            "2099/05/05", "myProperty is no longer used in the simulation", myProperty
        )

The first two arguments to ``deprecated.DeprecatedProperty`` are the same as the
arguments to ``deprecated.deprecated`` or ``deprecated.DeprecatedAttribute``.
The third argument, however, shold be the name of the property to deprecate.

Deprecating C++ Code Wrapped by SWIG
------------------------------------
This section explains how to deprecate code that is written in C++ and exposed to 
Python through a SWIG interface. Note that deprecation warnings will be raised only
when the Python wrappers to C++ functionality are invoked. Currently, it is not
possible to emit deprecation warnings when the deprecated functionality is called from
C++.

In order to deprecate functions, classes, or variables in C++, we use special
macros in the SWIG file that is exposing
the deprecated functionality. For example, let's consider we have this C++ code:

.. code-block:: cpp

    // example.h

    void standaloneFun(int, double) {};

    struct MyClass
    {
        void myFun() {};

        int myAttribute;
    };

with the following SWIG interface file:

.. code-block::
    
    // example.i

    %module example
    %{
       #include "example.h"
    %}

    %include "example.h"

If we want to deprecate the **standalone function** and **class function**, then we
would change the SWIG file to:

.. code-block::
    
    // example.i

    %module example
    %{
       #include "example.h"
    %}

    %include "swig_deprecated.i"
    %deprecated_function(standaloneFun, "2023/01/01", "You should use standaloneFunNew")
    %deprecated_function(MyClass::myFun, "2023/01/01", "myFun has no effects.")

    %include "example.h"

In the code above, we have included ``"swig_deprecated.i"``, which makes the
``%deprecated_function`` macro available. Then, we have called this macro **before we included the header file** 
``"example.h"``. The first input to the macro is the SWIG identifier for the function.
For standalone functions this is simple the function name, but for class functions this is
``[CLASS_NAME]::[FUNCTION_NAME]``. The next two arguments are the expected removal date
and message, as covered in the previous section.

If we want to deprecate an entire **class**, then the SWIG file ought to change to:

.. code-block::
    
    // example.i

    %module example
    %{
       #include "example.h"
    %}

    %include "swig_deprecated.i"
    %deprecated_function(MyClass::MyClass, "2023/01/01", "Use MyNewClass.")

    %include "example.h"

Again, we use ``%deprecated_function`` before ``%include "example.h"``. This time, however,
we need to target ``[CLASS_NAME]::[CLASS_NAME]``.

Finally, to deprecate a class variable, the SWIG file would change to:

.. code-block::
    
    // example.i

    %module example
    %{
       #include "example.h"
    %}

    %include "swig_deprecated.i"
    %deprecated_variable(MyClass, myAttribute, "2023/01/01", "Use MyNewClass.")

    %include "example.h"

This time, we call the macro ``%deprecated_variable``, although always 
before ``%include "example.h"``. In this case, the two first arguments to ``%deprecated_variable``
are the name of the class that contains the variable, and then the name of the varible.
The final two arguments are the expected removal date and the message.
