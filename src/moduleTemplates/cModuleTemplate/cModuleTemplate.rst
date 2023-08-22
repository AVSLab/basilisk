Executive Summary
-----------------
Provide a brief introduction to purpose and intent of this module.  This should be a short description.
If this requires lots explanation, images, equations, etc., then use the `Detailed Module Description`_
section below.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. _ModuleIO_FSW_MODULE_TEMPLATE:
.. figure:: /../../src/moduleTemplates/cModuleTemplate/_Documentation/Images/moduleIOcModuleTemplate.svg
    :align: center

    Figure 1: ``cModuleTemplate()`` Module I/O Illustration


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - dataInMsg
      - :ref:`CModuleTemplateMsgPayload`
      - (optional) Input message description.  Note here if this message is optional, and what the default behavior
        is if this message is not provided.
    * - dataOutMsg
      - :ref:`CModuleTemplateMsgPayload`
      - Output message description.

Detailed Module Description
---------------------------
Provide a brief introduction to the material being discussed in this report.  For example, include what the
motivation is, maybe provide a supportive figure such as shown below, reference earlier work if needed in a
literature review web links. Describe the module including mathematics, implementation, etc.

Equations
^^^^^^^^^
Equations can be provided with LaTeX as well.  For example, the code::

    :math:`a = b^{2}`

produces this equation inline :math:`a = b^{2}` equation.  In contrast, this code::

    .. math::
        a = b^2

or this compact version for 1 liners::

    .. math:: a = b^2

creates this block of math.

.. math::
    a = b^2

To create a numbered equation you need to add a label::

    .. math::
        :label: eq-fswModule-firstLaw

        a = b^2

which creates this

.. math::
    :label: eq-fswModule-firstLaw

    a = b^2

This label can be referenced using ``:eq:`eq-fswModule-firstLaw``` to cite Eq. :eq:`eq-fswModule-firstLaw`.
Note that these label names must be unique across all of the Basilisk RST documentation.  It is encouraged to use
a module-unique naming scheme.

To do bold math, we can't use the popular ``\bm`` command.  Instead, we can use ``{\bf u}`` (regular letters) or
``\pmb \omega`` (greek letters).  The following math is an example of this showing both bold and un-bold letters
next to each other:

.. math:: {\bf u} u = 3 \hat{\bf e}_3
    :label: eq-2

.. math::  \pmb \omega \omega = 2 \hat{\imath}_{\theta}
    :label: eq-3

More details on how to typeset TeX math in Sphinx can be found `here <https://documentation.help/Sphinx/math.html>`__.

If the module description requires extensive math discussion, this can be TeX'd up using the technical note
template inside the ``_Documentation`` folder. A link should be included in the HTML documentation to
the :download:`Detailed PDF Documentation </../../src/moduleTemplates/cModuleTemplate/_Documentation/Basilisk-MODULENAME.pdf>`
using the code::

    :download:`Detailed PDF Documentation </../../src/moduleTemplates/cModuleTemplate/_Documentation/Basilisk-MODULENAME.pdf>`

The PDF technical should only be used as a last resort effort if the math is simply too complex and long to
include in the `spinx` documentation.  Another option is to link to a web site, conference paper, journal
paper, book or thesis document that discussed the mathematical developments used.

Citations
^^^^^^^^^
If you want to cite other papers or text, provide a web link to a paper.  For example::

    `The link text <http://example.net/>`__

creates `The link text <http://example.net/>`__.

Images and Figures
^^^^^^^^^^^^^^^^^^
To include static, non-``pytest`` generated images and figures, you must copy the web compatible image (svg, jpg, png)
to a local sub-folder ``cModuleTemplate/_Documentation/Images/``.   This keeps the modules images grouped
within this sub-folder and contained within the main module folder.  The SVG image format is preferred as it is
a vectorized format that renders in a higher quality.  Further, when viewed in dark mode the svg will
automatically convert to a dark image (preserving colors).  Pixelated formats such as jpg and png remain the same
in light and dark mode of the documentation web page.

For example, to include an image (has no caption) you can use code such as::

    .. image:: /../../src/moduleTemplates/cModuleTemplate/_Documentation/Images/fig1.svg
        :align: center

to generate the following image.

.. image:: /../../src/moduleTemplates/cModuleTemplate/_Documentation/Images/fig1.svg
     :align: center

Note that with pixelated images such as ``jpg`` and ``png`` format save the file at twice the resolution
that you need, then provide ``:scale: 50 %`` to shrink it to the normal size.  This way the image has
enough resolution to look good on high-resolution displays.

To include a figure (has a caption and you can add label), use the following code::

    .. _figLabel:
    .. figure:: /../../src/moduleTemplates/cModuleTemplate/_Documentation/Images/fig1.svg
        :align: center

        Figure 2: Concept Illustration of the Math used in this Module

This yields

.. _figLabel:
.. figure:: /../../src/moduleTemplates/cModuleTemplate/_Documentation/Images/fig1.svg
    :align: center

    Figure 2: Concept Illustration of the Math used in this Module

You can cite the figure using ``:ref:`figLabel```. For example, as seen in :ref:`figLabel`, the figure can
now be referenced.

More information on how to include images or figures using sphinx can be found
`here <http://docutils.sourceforge.net/docs/ref/rst/directives.html#images>`__.  In particular, it is
also possible to include an image as a figure which has a caption.


Tables
^^^^^^
The standard sphinx table formatting can be used to generate tables.  More information on spinx table formatting
can be found `here <http://docutils.sourceforge.net/docs/ref/rst/restructuredtext.html#grid-tables>`__.
For example, the code::

    .. table:: Module I/O Messages

        +------------------------+------------+----------+----------+
        | Header row, column 1   | Header 2   | Header 3 | Header 4 |
        | (header rows optional) |            |          |          |
        +========================+============+==========+==========+
        | body row 1, column 1   | column 2   | column 3 | column 4 |
        +------------------------+------------+----------+----------+
        | body row 2             | Cells may span columns.          |
        +------------------------+------------+---------------------+
        | body row 3             | Cells may  | - Table cells       |
        +------------------------+ span rows. | - contain           |
        | body row 4             |            | - body elements.    |
        +------------------------+------------+---------------------+

will generate the following table:

.. table:: Module I/O Messages

        +------------------------+------------+----------+----------+
        | Header row, column 1   | Header 2   | Header 3 | Header 4 |
        | (header rows optional) |            |          |          |
        +========================+============+==========+==========+
        | body row 1, column 1   | column 2   | column 3 | column 4 |
        +------------------------+------------+----------+----------+
        | body row 2             | Cells may span columns.          |
        +------------------------+------------+---------------------+
        | body row 3             | Cells may  | - Table cells       |
        +------------------------+ span rows. | - contain           |
        | body row 4             |            | - body elements.    |
        +------------------------+------------+---------------------+



.. note:: Doing tables with spinx is not simple.  The table outline must abide by tedious spacing rules.

The ``list-table`` command is nice in that it allows for a simple table to be created where the table
structure does not have to be drawn with ASCII vertical and horizontal lines.  However, the formatting options
are more limited than with the above method.  See
`documentation <https://docutils.sourceforge.io/docs/ref/rst/directives.html#list-table>`__ for more info.
For example, the code::

    .. list-table:: List Based Table Title
        :widths: auto
        :header-rows: 1

        * - Header 1
          - Header 2
          - Header 3
        * - Label 1
          - text
          - more text
        * - Label 2
          - text
          -
        * - Label 3
          - text
          - some more text

will produce this table:

.. list-table:: List Based Table Title
    :widths: auto
    :header-rows: 1

    * - Header 1
      - Header 2
      - Header 3
    * - Label 1
      - text
      - more text
    * - Label 2
      - text
      -
    * - Label 3
      - text
      - some more text

HTML Highlight Options
----------------------
With Sphinx you can easily create HTML highlight blocks called admonitions such as
attention, caution, danger, error, hint, important, note, tip, warning.  Here are samples of what these
blocks look like.

.. danger::

    text goes here

.. error::

    text goes here

.. attention::

    text goes here

.. caution::

    text goes here

.. warning::

    text goes here

.. hint::

    text goes here

.. important::

    text goes here

.. tip::

    text goes here

.. note::

    text goes here


Module Assumptions and Limitations
----------------------------------
This section should describe the assumptions used in formulating the mathematical model and how those assumptions
limit the usefulness of the module.


User Guide
----------
This section contains information directed specifically to users. It contains clear descriptions of what inputs
are needed and what effect they have. It should also help the user be able to use the model for the first time.

Add sample code as needed.  For example, to specify that the module variables ``dummy`` and ``dumVector`` must
be setup first, you can include python formatted code block using::

    .. code-block:: python
        :linenos:

        module.dummy = 1
        module.dumVector = [1., 2., 3.]

to show:

.. code-block:: python
    :linenos:

    module.dummy = 1
    module.dumVector = [1., 2., 3.]

More information of including code blocks can be found `here <https://www.sphinx-doc.org/en/master/usage/restructuredtext/directives.html#directive-code-block>`_.

In the user guide, provide sub-sections as need to help explain how to use this module, list what variables
must be set, discuss variables that might have default values if not specified by the user, etc.
