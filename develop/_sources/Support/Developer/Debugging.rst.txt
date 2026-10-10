
.. _debugging:

Accessing the Debugger for C/C++ Basilisk Modules
=================================================================

Motivation
----------

Debuggers are very powerful tools for any developer. While some problems can be diagnosed using temporary print statements throughout the code, this technique can be unnecessarily time consuming and cluttering -- requiring recompilation, iteration, and all-the-while dramatically increasing the likelihood of accidentally modifying code. Using debuggers allows the developer to carefully analyze control flow and access the program's memory in real time, allowing developers to solve problems more efficiently and in a more controlled manner. 

When developing for Basilisk, it is particularly useful to know how to access a debugger at the C/C++ module level. One might assume that the Python debugger would be sufficient to access the call stack of a C/C++ module, because the C/C++ modules are wrapped into Python using SWIG. **This is incorrect.** Instead, developers need to invoke a C/C++ debugger like GDB either through the command line or a C/C++ compatible IDE to gain access to the module's stack. This support page provides the developer with the steps necessary to accomplish this.

Basilisk build requirements
---------------------------

Before debugging, one must first configure Basilisk with the appropriate build type using the following command::

    python3 conanfile.py --buildType Debug

This ensures that the correct conan dependencies will be pulled, and that the configured project will be set to ``Debug`` rather than ``Release`` by default. 

.. Note::
    When projects are configured to build in ``Release``, the compiler automatically applies optimizations to the machine code during the build. These optimizations effectively rewrite small algorithms, and such rewrites will not necessarily be reflected in the human-readable code. This is why using debuggers to step through programs that are compiled for Release/Profiling is unstable and may yield unintuitive control flow or unreadable memory. Configuring projects for ``Debug`` ensures that these optimizations are not applied, such that developers can be sure that there is a direct mapping between the human-readable code and the machine code being executed. 

By default, the Basilisk project will also be built using the previous command. However, if the developer prefers to build the project manually, they should add ``--buildProject False`` to the previous command and remember to explicitly build for ``Debug``. More information on building the software framework can be read in :ref:`configureBuild`.

How to catch breakpoints in a C/C++ module
------------------------------------------

The first step to catching a breakpoint set in a C/C++ module is to add a breakpoint to the Python script that calls said module. The location of this breakpoint in the Python script is not 
important, as long as it is caught by the Python debugger before the C/C++ code in question is executed. After setting the Python breakpoint, run the script using the Python debugger (or in a debugging mode if using an IDE) and wait until the Python breakpoint is reached.

After that, the developer must attach the C/C++ debugger to the newly spawned Python process. This can be done within the developer's C/C++ IDE of choice or using GDB directly. This page focuses on the former case. Within the developer's C/C++ IDE, one must search for the option ``Attach to Process`` that is usually under the ``Debug`` tab on the IDE. A list of active processes will appear, and the developer can search for ``Python`` to find the current active Python processes. Multiple Python processes may be active, but only one is currently stopped at the breakpoint in the Python script. Identifying the correct process identifier (PID) can be difficult. Some IDEs report the PID in the first line of text output to the terminal. If this is not true for the developer's IDE, it is recommended to select the Python PID with the highest value. This generally corresponds to most recently spawned process which is often the process containing the relevant breakpoint. 

Once the C/C++ debugger is attached to the correct Python process, the developer can allow the Python debugger to ``continue``. The C/C++ debugger will then stop at the set breakpoint in the C/C++ Basilisk module and the developer can then begin using the C/C++ debugger in its entirety.