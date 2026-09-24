Executive Summary
-----------------

This is a very basic sample C++ Basilisk module that can be used as a template to create other C++ modules.
It mimics the functionality of :ref:`cModuleTemplate`.  See that module for a more complete discussion
of how to write the RST module documentation file.


Message Connection Descriptions
-------------------------------
The following diagram and table list all the module input and output messages.  The module message connection is
set by the user from Python.  The message type contains a link to the message structure definition, while the
description provides information on what this message is used for.

.. bsk-module-io:: cppModuleTemplate
    :caption: Module I/O Messages

    input dataInMsg CModuleTemplateMsgPayload
        (optional) Input message description.  Note here if this message is optional, and what the default behavior
        is if this message is not provided.

    output dataOutMsg CModuleTemplateMsgPayload
        Output message description.

User Guide
----------
The sample variables follow the :ref:`configuration and runtime state roles <moduleTemplateVariableRoles>`
described for the C template. Use ``setSampleConfigVector()`` to set the vector for configuration and logging
examples; its value is preserved by resets and updates. ``setUpdateCounter()`` demonstrates a scalar setter,
but ``updateCounter`` is runtime state: ``Reset()`` clears it and each update increments it. The corresponding
``getSampleConfigVector()`` and ``getUpdateCounter()`` methods allow these private variables to be recorded as shown
in :ref:`bskPrinciples-6`.
