
.. _debugging:

Debugging
=========

Motivation
----------

Debugging is a very powerful tool for any developer. While some problem solving can be done using temporary print statements throughout the code, this technique can be unnecessarily time consuming, while also 
requiring changing the code, which can be prohibitive and leads to the developer forgetting to delete the temporary code. Debugging always the developer to directly analyze the program's memory in real time, 
while also having the ability to change that memory on the fly. It also allows the developer to better understand and exploit the code's logic and flow. A developer with good debugging skills solves problems 
faster and more efficiently than one without them.

However, debugging in Basilisk is not as straight forward as with other software tools. The fact that the user interface is done in Python while the code is run in C/C++ means that the IDE the user 
is using in Python is not sufficient to debug problems in modules writen in C/C++. This support page has the goal of teaching the developer how to properly debug in Basilisk, which will hopefully allow anyone 
to more quickly find and solve problems while doing code development.

Basilisk build requirements
---------------------------

Before one can start debugging, it must first build Basilisk with the appropriate build type, otherwise debugging will not work because the necessary dependencies will not be set. By default, Basilisk is built 
for ``Release``, which will not allow for debugging. To do so, the developer must build Basilisk for ``Debug``, which is done through the following command::

    python3 conanfile.py --buildType Debug

By default, Basilisk will be built using the previous command. However, if the developer prefers to build the project using their own IDE, they should add ``--buildProject False`` to the previous command. more
information on building the software framework can be read in :ref:`configureBuild`.

How to catch breakpoints in a C/C++ module
------------------------------------------

The first step to catching a particular breakpoint in a module is to add a breakpoint to the Python script that the developer wants to analyze. The location of this breakpoint in the Python script is not 
important, as long as this breakpoint is caught in the Python IDE before the module to analyze is run. After setting the breakpoint, run the script in debugging mode, which should stop at the breakpoint.

After that, the developer should add a breakpoint in the C or C++ module to be analyzed. After that, the Python process needs to be attached in the C/C++ IDE of choice. This can be achieved by choosing the option 
``Attach to Process`` that is usually under the ``Debug`` tab on the IDE. A list of active processes will appear, and one can usually search for ``Python`` to find the current active Python processes. Multiple 
processes may be active, one of them being the one that is stopped at the breakpoint in the Python script. There is no good way to tell which process is the correct one, so some trail and error might be need to 
figure out which one is the correct one to attach to. If the correct process is attached, the breakpoin that was set on the C/C++ script should appear as active.

Once the breakpoint is added and the correct process is attached, the developer can choose to ``continue`` the Python script, which will then stop at the set breakpoint in the C/C++ script.