.. _bskPrinciples-10:

.. warning:: 

    This section refers to a deprecated way of operating with C modules. Refer to previous documentation pages for the updated way.

Deprecated: Using old-style C modules
=====================================
In more recent Basilisk scripts, whether a module is implemented in C++ or C should not make
any difference on how this module is used in Python scripts. However, this has not always been
the case, and you might encounter some code that uses the older syntax. This documentation 
summarizes how to use this older syntax.

Previous documentation pages have taught us that C++ modules (and new-syntax C modules) are
created, configured, and added to the simulation through the following syntax::

    module = someModule.someModule()
    module.someParameter = 5
    module.ModelTag = "someModuleName"
    scSim.AddModelToTask("taskName", module, priority)

In order to perform the same operations on an old-syntax C module, one would do::

    moduleConfig = someModule.someModuleConfig()
    moduleConfig.someParameter = 5
    moduleWrap = scSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "someModuleName"
    scSim.AddModelToTask("taskName", moduleWrap, moduleConfig, priority)

Note that in this case, we created a "Config" object ``someModule.someModuleConfig``. Connecting 
messages and setting parameters of the module is done through this object. Then, the ``setModelDataWrap``
method of the simulation object is called on the "Config" object, which generates the "Wrap" object.
The unique name must be set on the "Wrap" object. Finally, the module is added to the simulation by
using both the "Wrap" and "Config" objects in the ``scSim.AddModelToTask`` method.

The need for separate "Config" and "Wrap" objects arises from the lack of classes in the C programming language. 
The "Config" objects, as well as the relevant ``UpdateState``, ``Reset``, and ``SelfInit`` methods, 
are written in pure C for C modules. However, the simulation framework is written in C++ and it expects
the modules to be C++ classes. The "Wrap" object is this C++ class, which holds references to
the "Config" object and the relevant methods so that they can be accesses from C++ as a single class.

After the refactor of C modules, the working principle remains largely the same. However, the "wrapping"
now happens at a stage that is hidden from users, so that it appears that C modules are just like
C++ modules.
