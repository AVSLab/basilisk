
.. _migratingToBsk2:

Migrating Basilisk Scripts from Version 1.X to 2.X
==================================================

Motivation
----------
This document discusses what user-facing changes occured with the new messaging system in Basilisk version 2.0
and higher.  The string messaging system is replaced with a smarter message object system that

- prevents the user from connecting to the wrong message type in the C/C++ code
- warns the user if the Python scripts connects messages of the wrong type
- has much faster logging functionality
- requires explicit message connection setup, i.e. no hidden implicit message naming

The overall goal is to create a new implementation is that is easier to use and understand, as well as faster
to execute.

However, nothing is for free.  Making these changes was not possible without breaking existing code.  This migration
help page outlines all the user-facing changes that have occured.  This facilities the process of upgrading legacy
Basilisk python scripts to function with the new message system, etc.


Message Names
-------------
The module input and output messages are no longer specified through a name, and then subscribed to via an ID handle.
Rather, the messages have become smart objects that can be directly connected to the receiving module.  Thus,
while before we had ``stateOutMsgName`` and ``stateOutMsgId`` variables, now a single msg variable named
``stateOutMsg`` is used.   See :ref:`codingGuidelines` for more info on message naming.

Logged Data
-----------
The logging of messages is much simplified.  There are a few changes to note in the format of the logged data.  In
Basilisk v2.0 and higher the time information is no longer pre-pended in a first column, but rather provided as a
separate array accessed through ``.times()``.  This means logging `N` time steps of a 3D vector no longer no longer
yields a `Nx4` array, but rather a `Nx3` array.  Some plotting or value checking logic might have to be updated.

Module and Message Naming Changes
---------------------------------
Some early Basilisk modules and message names never complied with the naming guidelines in :ref:`codingGuidelines`.
The following list outlines any module or message naming changes that occured in this upgrade process.  This makes it
simple to see what naming will need to be changed.


