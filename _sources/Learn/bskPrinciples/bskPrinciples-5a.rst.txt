.. _bskPrinciples-5a:

Visualizing Message Connections
===============================

.. sidebar:: Source Code

    The Python code shown below can be downloaded :download:`here </../../docs/source/codeSamples/bsk-5a.py>`.

The previous tutorial, :ref:`bskPrinciples-5`, shows how to create stand-alone messages and subscribe
module input messages to them.  Once a simulation grows beyond a few modules it can be difficult to
verify that every input is subscribed to the expected output or stand-alone message.  The
:ref:`SimulationBaseClass` helper ``ShowMessageConnectionFigure`` can create a compact Matplotlib
figure or a Graphviz-rendered file that shows the configured module message flow.

The message flow diagram is an inspection tool.  It does not change task execution, message
connections, message logging, or the simulation results.  It can be called after the simulation
objects have been created and added to tasks, either before or after the simulation is executed.

The sample script below creates a module, connects a stand-alone input message, and records the
module output message.  The stand-alone input message is supplied through ``extraMessages`` so the
diagram can draw where the module input is connected.

.. literalinclude:: ../../codeSamples/bsk-5a.py
   :language: python
   :linenos:
   :lines: 18-

Running the script returns the following Matplotlib figure.

.. image:: /_images/Scenarios/bsk-5a.svg
   :align: center

If Graphviz is installed, the same script also writes the following Graphviz-rendered SVG file.
The sample uses ``unitTestSupport.saveScenarioGraphvizFigure`` so documentation images are saved
to the standard scenario image folder.  When ``show_plots`` is ``True``, the tutorial displays both
the Matplotlib figure and the saved Graphviz output.

.. image:: /_images/Scenarios/bsk-5a-graphviz.svg
   :align: center

Creating the Figure
-------------------

The figure is created with::

    fig = scSim.ShowMessageConnectionFigure(
        show_plots=False,
        extraMessages=None,
        includeUnlinked=True,
        includeRecorders=True,
        renderer="matplotlib",
        fileName=None,
        graphvizFormat="svg",
        graphvizLayout="vertical",
    )

With the default ``renderer="matplotlib"`` option, the return value is the Matplotlib figure object.
This lets a scenario return the figure in its ``figureList`` dictionary for automatic documentation,
or save the figure using the normal Matplotlib interface.

The options are:

``show_plots``
    If this is ``True``, the figure is displayed with ``plt.show()``.  Scenario and documentation
    tests normally set this to ``False`` and return the figure instead.

``extraMessages``
    Optional stand-alone source messages to include in the graph.  This can be a dictionary such as
    ``{"vehicleConfigMsg": vcMsg}``, or a list of dictionaries such as
    ``[{"vehicleConfigMsg": vcMsg}, {"rwConfigMsg": fswRwParamMsg}]``.  The dictionary keys become
    the labels used in the figure.  Use this option for stand-alone messages that are not owned by a
    module, such as vehicle configuration messages, reaction wheel configuration messages, commanded
    torque messages, or an epoch message.  Connections from these supplied messages are drawn with
    dashed lines.

``includeUnlinked``
    If this is ``True``, input message fields that are not subscribed to a source are shown as gray
    ports.  This is useful when checking whether a setup forgot to connect an input message.  If it
    is ``False``, these unused input message fields are hidden to reduce clutter.  Linked inputs
    whose source message cannot be found remain in the graph and are listed in the programmatic
    graph output as ``unresolvedInputs``.

``includeRecorders``
    If this is ``True``, message recorder modules created with ``.recorder()`` are included in the
    diagram.  This is useful when verifying that the desired message is being recorded.  If it is
    ``False``, recorder modules are hidden so the figure focuses on the regular Basilisk modules.
    A recorder attached to an input message is drawn from that module input endpoint to the recorder.
    A recorder attached to an output message is drawn from that module output endpoint to the recorder.

``renderer``
    Selects the drawing backend.  The default value ``"matplotlib"`` returns a Matplotlib figure.
    The value ``"graphviz"`` writes a Graphviz ``dot`` file and renders it through the Graphviz
    ``dot`` executable.  Graphviz does not create a Matplotlib figure object; it creates a saved
    output artifact such as an SVG, PNG, or PDF file and returns the file path.

``fileName``
    Optional output path used by the Graphviz renderer.  If the path has an extension such as
    ``.svg`` or ``.pdf``, that extension selects the Graphviz output format.  If no extension is
    provided, ``graphvizFormat`` is appended.  If no ``fileName`` is supplied, a temporary file is
    created.

``graphvizFormat``
    Output format used by the Graphviz renderer when ``fileName`` does not provide an extension.
    Common values are ``"svg"``, ``"png"``, ``"pdf"``, and ``"dot"``.  The ``"dot"`` value writes
    only the Graphviz source file and does not run the Graphviz layout command.

``graphvizLayout``
    Module layout direction used by the Graphviz renderer.  The default value ``"vertical"``
    stacks the modules from top to bottom to keep large scenarios narrower.  The value
    ``"horizontal"`` draws the execution order from left to right, matching the original
    Graphviz layout style.  The aliases ``"TB"`` and ``"LR"`` can also be used.

Reading the Figure
------------------

The module boxes are shown in the same order that the simulation executes them.  Each box includes
the module tag and its process and task names.  Input message fields are drawn on the left side of a
module box, while output message fields are drawn on the right side.

The default message port color is gray.  When a message port participates in a discovered connection,
the port is shown in orange on both ends of the connection.  Solid lines represent links between
endpoints discovered within the configured simulation.  Dashed lines represent links whose source was
provided through ``extraMessages``.

Creating and Showing a Graphviz Diagram
---------------------------------------

For larger simulations, Graphviz can route the connection lines around module boxes more cleanly
than the Matplotlib renderer.  The default Graphviz layout is vertical, which is usually more
compact for documentation pages than a wide left-to-right module chain.  Because Graphviz is
file-oriented, the result must be written to a file.  If ``show_plots`` is ``True``, Basilisk opens
that rendered file with the operating system's default viewer after it is written.

For example, this writes an SVG message-flow diagram, opens it with the default viewer, and returns
the SVG path::

    outputPath = scSim.ShowMessageConnectionFigure(
        show_plots=True,
        renderer="graphviz",
        fileName="messageConnections.svg",
        graphvizLayout="vertical",
        extraMessages={"inputMsg": inputMsg},
        includeUnlinked=False,
        includeRecorders=True,
    )

The Graphviz renderer also writes the intermediate ``messageConnections.dot`` file next to the SVG
file.  This DOT file can be useful when debugging the layout or when a user wants to run Graphviz
manually with custom command line options.

The Graphviz renderer requires the ``dot`` executable to be installed and available on the system
path.  The Python ``graphviz`` package is not required because Basilisk writes the DOT source
directly.

Extracting the Graph
--------------------

If the connections are needed for a unit test or custom analysis, use ``GetMessageConnectionGraph``
instead of drawing a figure::

    graph = scSim.GetMessageConnectionGraph(
        extraMessages={"inputMsg": inputMsg},
        includeUnlinked=False,
        includeRecorders=True,
    )

This method uses the same ``extraMessages``, ``includeUnlinked``, and ``includeRecorders`` options.
It returns a dictionary with the following entries:

``modules``
    The ordered module records, including process, task, priority, period, inputs, and outputs.

``standaloneMessages``
    The stand-alone messages provided through ``extraMessages``.

``inputs`` and ``outputs``
    The endpoint records found on modules and supplied stand-alone messages.

``edges``
    The discovered source-to-target message connections.

``unlinkedInputs``
    Input endpoints that are not connected, included only when ``includeUnlinked`` is ``True``.

``unresolvedInputs``
    Linked input endpoints whose source message was not found among module outputs or
    ``extraMessages``.

The corresponding Graphviz DOT source can be retrieved without rendering a file using::

    dotText = scSim.GetMessageConnectionDot(
        extraMessages={"inputMsg": inputMsg},
        includeUnlinked=False,
        includeRecorders=True,
        graphvizLayout="vertical",
    )
