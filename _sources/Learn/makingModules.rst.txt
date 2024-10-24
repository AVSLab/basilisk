.. _makingModules:

Basics of Writing Basilisk Modules
==================================


.. sidebar:: Making New Modules

    You should build your own Basilisk modules outside of the primary Basilisk source folder.  This keeps your Basilisk source folder free of local customizations that interfere with upgrading the simulation framework.  The only exception is if you are planning on pushing the module back to the primary Basilisk source itself.  Instructions on how to create modules outside of the main source folder are found at :ref:`buildExtModules`.

This chapter covers the basics of designing Basilisk modules, as well as how to write a module using C++, C or Python.  As always with code documentation, it is never complete and always work-in-progress.

For general developer support pages on coding guidelines, forking, building the Sphinx documentation system etc., see
``Support/Developer Information`` in :ref:`makingModules`.

The following pages cover the primary tasks required to make a Basilisk module.  This includes how to design the basic module function, how messages work, what methods are required for a module, etc.  Note that the dynamics modules are both Basilisk modules and also a sub-class of either :ref:`dynamicEffector` (provide external forces and torques acting on the body, but doesn't have state differential equations to integrate), or :ref:`stateEffector` (has internal state differential equations to integrate that couple with the spacecraft rigid hub).

.. warning::
    The goal of the following content is to facilitate the process of creating new modules.  However, this documentation does not avoid the need to study the Basilisk source code, including how other modules were written, to understand how to create new modules.


.. toctree::
   :maxdepth: 2

   makingModules/makingModules-1
   makingModules/makingModules-2
   makingModules/cModules
   makingModules/cppModules
   makingModules/pyModules
   makingModules/makingModules-3
   makingModules/makingModules-4
   makingModules/makingDraftModule
   makingModules/makingModules-5
   makingModules/advancedTopics


