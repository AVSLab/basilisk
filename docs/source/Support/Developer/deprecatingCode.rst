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

If a C++ structure or one of its fields are renamed, an alias can be used to deprecate the entity while retaining support for the old name. This is done using the ``_DeprecatedWrapper`` class defined in ``"swig_deprecated.i"``. Its implementation is detailed below.

.. code-block::

    // example.i

    %module example
    %{
        #include "example.h"
    %}

    %include "swig_deprecated.i"
    %include example.h

    %pythoncode %{
    import sys

    mod = sys.modules[__name__]

    mod.ExampleStruct = _DeprecatedWrapper(
        mod.ExampleStruct,
        targetName="ExampleStruct",
        deprecatedFields={"oldField": "newField"},
        typeConversion="scalarTo3D",
        removalDate="YYYY/MM/DD"
    )

    mod.OldStructure = _DeprecatedWrapper(
        mod.NewStructure,
        aliasName="OldStructure",
        targetName="NewStructure",
        removalDate="YYYY/MM/DD"
    )

    protectAllClasses(sys.modules[__name__])
    %}

If a field is renamed, the top chunk creates a wrapper that contains the old field name and handles deprecation warnings and getter/setter behavior. The ``typeConversion`` parameter can be set to ``"scalarTo3D"`` to deprecate a scalar variable and alias to a new 3D variable with repeated values.

If a structure is renamed, the second chunk creates a wrapper that generates an alias using the old structure name for continued support.

Deprecating Rust Code
---------------------
Native Rust Basilisk modules have two deprecation mechanisms with different
audiences:

* Basilisk's dated module and field annotations generate runtime warnings for
  Python users.
* Rust's standard ``#[deprecated]`` attribute generates a compiler warning for
  Rust code that uses a deprecated Rust item.

The standard Rust attribute does not generate a warning when a user imports or
uses the generated Python module, and it does not implement Basilisk's dated
urgent-warning behavior. It is therefore not a substitute for
``#[bsk(deprecated(...))]`` on a Python-visible interface. Choose the mechanism
based on whether the deprecated interface is visible to Python users or only
to other Rust code.

Python-Visible Rust Modules
~~~~~~~~~~~~~~~~~~~~~~~~~~~
Deprecate construction of an entire native Rust Basilisk module by adding
deprecation metadata to its module annotation:

.. code-block:: rust

    #[bsk_build::module(
        deprecated(
            removal_date = "2099/05/05",
            message = "Use replacementModule instead."
        )
    )]
    #[repr(C)]
    pub struct oldModuleConfig {
        // Configuration fields and message ports
    }

Constructing ``oldModule.oldModule()`` from Python then issues the standard
Basilisk deprecation warning. The warning does not prevent the module from
being configured, connected, or scheduled during its deprecation period. It
becomes ``BSKUrgentDeprecationWarning`` on or after the removal date.

Python-Visible Configuration Fields
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
A non-message-port configuration field in a native Rust Basilisk module can be
deprecated as follows:

.. code-block:: rust

    #[bsk(deprecated(
        removal_date = "2099/05/05",
        message = "Use newGain instead."
    ))]
    pub oldGain: f64,

Reading or assigning ``oldGain`` from Python issues Basilisk's standard dated
deprecation warning. The generated ``getOldGain()`` and ``setOldGain()`` methods
issue the same warning. The field remains functional during the deprecation
period, so existing scripts do not immediately break.

The generated wrapper calls the same ``deprecated.deprecationWarn()`` function
used by the C++ deprecation wrappers. Before the removal date it emits
``BSKDeprecationWarning``. On or after that date it automatically emits the
bold-red ``BSKUrgentDeprecationWarning``.

The removal date must use ``YYYY/MM/DD``. As with Python and C++ deprecations,
the message should name the preferred replacement and provide enough
information for users to migrate. A deprecated field may also retain a
validator:

.. code-block:: rust

    #[bsk(
        validate = validate_old_gain,
        deprecated(
            removal_date = "2099/05/05",
            message = "Use newGain instead."
        )
    )]
    pub oldGain: f64,

The ``#[bsk(deprecated(...))]`` annotation currently applies only to
Python-visible, non-port configuration fields. Direct access to the field from
within Rust does not issue this Python runtime warning.

Rust-Only Modules, Methods, and Other Items
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Use Rust's standard `deprecated attribute
<https://doc.rust-lang.org/reference/attributes/diagnostics.html#the-deprecated-attribute>`__
for an API consumed by other Rust code:

.. code-block:: rust

    #[deprecated(
        since = "2.13.0",
        note = "Use calculate_new_control instead."
    )]
    pub fn calculate_old_control() {
        // Retained implementation
    }

Rust emits a compiler warning when Rust code uses this function, and
``rustdoc`` identifies it as deprecated. The attribute can also be applied to
an inherent method, type, structure field, or Rust module. Deprecating a Rust
module causes its child items to inherit the deprecation.

Do not apply ``#[deprecated]`` to the ``init``, ``reset``, or ``update``
methods in a ``BskModule`` trait implementation. These are Basilisk framework
callbacks rather than user-facing methods, and Rust deprecation attributes
cannot usefully deprecate individual trait implementation methods.

Current Support Summary
~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 34 30 36

   * - Interface
     - Current mechanism
     - Warning audience
   * - Python-visible Rust configuration field
     - ``#[bsk(deprecated(...))]``
     - Python property, getter, and setter users
   * - Rust helper function, inherent method, type, or Rust module
     - ``#[deprecated(...)]``
     - Rust callers during compilation
   * - Entire Python-visible Rust Basilisk module
     - ``#[bsk_build::module(deprecated(...))]``
     - Python users constructing the module
   * - Python-visible Rust message port
     - Not currently generated
     - Document the deprecation until port-level wrapper support is added
   * - Python-visible custom Rust method
     - Not currently generated
     - Generated configuration getters and setters are covered by their field

Thus, a user-settable Rust module parameter can be deprecated today with a
Basilisk runtime warning, as can construction of an entire Rust Basilisk
module. Rust-only modules and methods can be deprecated for Rust callers.
Message ports and future custom methods exposed through the generated Python
wrapper require additional generator support. Adding ``#[deprecated]`` to the
Rust configuration structure alone is not a substitute because Python users
will not see that compiler warning.
