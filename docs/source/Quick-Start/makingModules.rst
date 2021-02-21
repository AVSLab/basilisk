.. _makingModules:

Basics of Writing Basilisk Modules
==================================


This chapter covers the basics of designing Basilisk modules, as well as how to write a module using C++, C or Python.  As always with code documentation, it is never complete and always work-in-progress.

The following pages cover the primary tasks required to make a Basilisk module.  This includes how to design the basic module function, how messages work, what methods are required for a module, etc.  Note that the dynamics modules are both Basilisk modules and also a sub-class of either :ref:`dynamicEffector` (provide external forces and torques acting on the body, but doesn't have state differential equations to integrate), or :ref:`stateEffector` (has internal state differential equations to integrate that couple with the spacecraft rigid hub).  These types of Basilisk effector modules are discussed separately below.

.. warning::
    The goal of the following content is to facilitate the process of creating new modules.  However, this documentation does not avoid the need to study the Basilisk source code, including how other modules were written, to understand how to create new modules.


.. toctree::
   :maxdepth: 2

   makingModules/makingModules-1
   makingModules/makingModules-2
   makingModules/cppModules
   makingModules/cModules
   makingModules/pyModules
   makingModules/makingModules-3
   makingModules/makingModules-4
   makingModules/makingModules-5


