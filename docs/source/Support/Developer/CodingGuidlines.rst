
.. _codingGuidelines:

Coding Guidelines
=================

Motivation
----------

Coding style is important. A clean, consistent style leads to code that
is more readable, debuggable, and maintainable. To this end, we
prescribe (and proscribe) a variety of practices. Our goal is to
encourage agile but reasoned development of code that can be easily
understood by others.

**C/C++ foundational guidelines**: this document uses as its foundation
the coding guidelines developed by Stroustrup and Sutter.

**Python foundational guidelines**: the `PEP
8 <https://www.python.org/dev/peps/pep-0008/>`__ guidelines are to be
followed for python code development.

The both of these foundational sets of guidelines are extensive and
where the Basilisk document is silent these guidelines will prevail.

These are guidelines, not rules. With very few exceptions, this document
does not completely ban any particular C/C++ or Python pattern or
feature, but rather describes best practice, to be used in the large
majority of cases. When deviating from the guidelines given here, just
be sure to consider your options carefully, and to document your
reasoning, in the code.

Above all else, be consistent. Follow this guide whenever possible, but
if you are editing a package written by someone else, follow the
existing stylistic conventions in that package (unless you are
retrofitting the whole package to follow this guide, for which you
deserve an award).

What about non-conforming code?
-------------------------------

Some Basilisk code was written prior to the release (and updates) of
this style guide. Thus, the codebase may contain code that doesn’t
conform to this guide. The following advice is intended for the
developer working with non-conforming code:

1. All new code should conform to this guide.
2. Unless you have copious free time, don’t undertake converting the
   existing codebase to conform to this guide.
3. If you are the author of a non-conforming package, try to find time
   to update the code to conform.
4. If you are doing minor edits to non-conforming code, follow the
   existing stylistic conventions in that code (if any). Don’t mix
   styles.
5. If you are doing major work on non-conforming code, take the
   opportunity to re-style it to conform to this guide.

Naming
------

The following shortcuts are used in this document to denote naming
schemes:

-  CamelCased: The name starts with a capital letter, and has a capital
   letter for each new word, with no underscores.
-  camelCased: Like CamelCase, but with a lower-case first letter
-  under_scored: The name uses only lower-case letters, with words
   separated by underscores.
-  ALL_CAPITALS: All capital letters, with words separated by
   underscores.

General Guidelines
------------------

Variables
~~~~~~~~~

No single letter variables. The only exceptions are ‘i’ as an iteration
index and a select list of mathematical symbols.

Naming Variable for Mathematics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following section specifies general guidelines for the naming of
variable to be used in code which implements mathematical operations.
The naming convention is heavily influenced by the textbook `Analytical
Mechanics of Space
Systems <https://arc.aiaa.org/doi/book/10.2514/4.105210>`__ by Schaub and
Junkins.

Indicating Reference Frames
~~~~~~~~~~~~~~~~~~~~~~~~~~~

A vector variable expressed with components in a reference frame :math:`\cal B`, is represented with the variable name followed by
an underscore and a capital letter denoting the frame :math:`{}^{\cal B}\bf v` as ``vector_B``.

An angular rate variable expressed in one frame :math:`\cal B`
with respect to a second  :math:`\cal R`, where components are
expressed in the frame  :math:`\cal B`, :math:`{}^{\cal B}\pmb\omega_{\mathcal{B}/\mathcal{R}}`, is given
as  ``omega_BR_B``.

MRP’s and DCM’s
~~~~~~~~~~~~~~~

A direction cosine matrix is expressed as  :math:`[BN]`, a mapping
of an  :math:`\cal N` frame vector into a  :math:`\cal B`
frame vector, is written ``dcm_BN``. Similarly for the Modified
Rodrigues Parameters (MRP) attitude parameterization the :math:`\pmb\sigma_{\mathcal{B}/\mathcal{N}}` is written ``sigma_BN``.

.. warning::
   If you are using the `Intel Eigen library <http://eigen.tuxfamily.org>`_ library to do linear algebra, the
   mapping from an attitude description such as quaternions or MRPs to a direction cosine matrix (DCM)
   using ``.toRotationMatrix()`` will return :math:`[NB]`, not :math:`[BN]`.

Inertia Tensor
~~~~~~~~~~~~~~

The Inertia tensor :math:`[I_C]` of the hub about the point  :math:`C` is defined in the body frame :math:`\cal B` components using the variable ``IHubPntC_B``.

Derivatives
~~~~~~~~~~~

The first and second time derivatives of scalar (:math:`\dot{x}`, :math:`\ddot{x}`) or vector (:math:`\dot{\bf{x}}`, :math:`\ddot{\bf{x}}`) quantities, respectively are written as ``xDot`` and ``xDDot``.

The first and second time derivatives with respect to a variable other than time should use the same pattern as time derivatives but with a different modifier. For example,  :math:`f '(x)` and :math:`f ''(x)` are written as ``xPrime`` and ``xDPrime`` respectively.

Common Usage Examples
~~~~~~~~~~~~~~~~~~~~~

-  Position vector from \ :math:`\cal N` to \ :math:`\cal B`
   in inertial frame components
   \ :math:`{}^{\cal N} \bf r_{\mathcal{B/N}}`: ``r_BN_N``
-  Inertial time derivative of position vector from
   \ :math:`\cal N` to \ :math:`\cal B` in inertial frame
   components \ :math:`{}^{\cal N} \dot{\bf r}_{\cal B/N}`:
   ``rDot_BN_N``
-  Time derivative with respect to the body of position vector from :math:`B` to :math:`H` in body frame components :math:`{}^{\cal B} \bf r'_{H/B}`: ``rPrime_HB_B``
-  Unit direction vector from \ :math:`B` to \ :math:`S` in
   body frame components \ :math:`{}^{\cal B} \hat{\bf s}_{S/B}`:
   ``sHat_SB_B``
-  Inertial time derivative of body angular rate with respect to the
   inertial frame in body frame components
   \ :math:`{}^{\cal B} \dot{\pmb\omega}_{\mathcal{B}/\mathcal{N}}`:
   ``omegaDot_BN_B``
-  DCM of the body frame with respect to the inertial frame
   \ :math:`[BN]`: ``dcm_BN``

Modules
-------

Messages
~~~~~~~~

Variables holding message names are to be composed in the following
manner.

.. code:: cpp

   SomeMsg_C descriptionInMsg;                      // C interface to input msg
   SomeMsg_C descriptionOutMsg;                     // C interface to output msg
   ReadFunctor<SomeMsgPayload> descriptionInMsg;    // C++ interface to input message
   Message<SomeMsgPayload> descriptionOutMsg;       // C++ interface to output message

-  ``SomeMsgPayload``: message structure definition
-  ``In`` (``Out``): indicates the direction of the message with respect to the module.
-  ``Msg``: explicitly identifies the variable as a message.

Variables holding data from a read message are to be composed in the following manner.

.. code:: cpp

   SomeMsgPayload descriptionInBuffer;

-  ``description``: description of the data.
-  ``In`` (``Out``): indicates the direction of the data being written
   to the buffer with respect to the module.
-  ``Buffer``: explicitly identifies the variable as having a data
   buffer functionality.

.. _bsk2MessageDefinition:

Message Definitions
~~~~~~~~~~~~~~~~~~~
The C based messages are stored in ``src/architecture/msgPayloadDefC`` as a ``*.h`` file.
The C++  messages are stored in ``src/architecture/msgPayloadDefCpp`` as a ``*.h`` file.
The file name uses Upper Camel Case and should be identical to the message name within the file.
The last  letters should be ``MsgPayload``.
For example, a particular spacecraft sensor message could be named ``SpecialSensorMsgPayload.h``.  The contents
could be

.. code:: cpp

    #ifndef SPECIAL_SENSOR_MESSAGE2_H
    #define SPECIAL_SENSOR_MESSAGE2_H

    /*! @brief Describe the purpose of the message */
    typedef struct {
        double sensorOutput_B[3];   //!<        sensor vector in B frame components */
        double sensorSignal;        //!<        raw sensor signal
        int status;                 //!<        sensor status flag
    }SpecialSensorMsgPayload;

    #endif

When running the Basilisk setup command ``python3 conanfile.py`` the related message interface files
are then automatially created and included in the project.

C/C++ Exceptions
----------------

-  Currently no language specific exceptions

Python Exceptions
-----------------

-  Variables are to be lower camelCase. This is done to maintain consistency across the C/C++ and Python code bases which are interfaced via SWIG.
-  Inline comments are accepted so long as they are kept brief.
-  Binary operator spaces will be adhered to as specified in PEP 8, however, not for math symbols operations. E.g. no spaces are included around \*, /, +, -, etc

.. code:: py

   # Yes
   x = (4*9/2)-1
   # No
   x = (4 * 9 / 2) - 1

