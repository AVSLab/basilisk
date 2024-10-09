
.. _vizardLiveComm:

Live Communication with Vizard
==============================

.. image:: /_images/static/basiliskVizardLogo.png
       :align: right
       :scale: 50 %

BSK can link with Vizard **live** in three ways:

* ``liveStream`` : The BSK sim runs in lockstep with Vizard. This supports user input handling and
  performs standard optical navigation if cameras are configured.
* ``broadcastStream`` : The BSK sim broadcasts read-only messages in a publish/subscribe pattern,
  where many Vizards may subscribe simultaneously.
* ``noDisplay`` : The BSK sim performs optical navigation headlessly through Vizard, only sending
  messages to Vizard when an image is requested. This is a more efficient OpNav mode, but shows no
  graphical display in Vizard.

.. caution::
    Legacy ``opNavMode`` has been deprecated. For ``opNavMode = 1``, use ``liveStream`` flag. For ``opNavMode = 2``, use ``noDisplay`` flag.


``liveStream`` Mode
===================

During ``liveStream`` mode, BSK sends a message containing simulation state information to Vizard at every
timestep. BSK then waits for an 'OK' response before proceeding. In this way, both BSK and Vizard operate
in lockstep with each other. This is the best method for visualizing a scenario live on your local machine.

To enable livestreaming through ``vizSupport`` use the ``liveStream`` flag:

.. code-block:: python

    viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, scObject
                                              , liveStream=True
                                              )

Or, to bypass the ``vizSupport`` module:

.. code-block:: python

    viz = vizInterface.VizInterface()
    viz.liveStream = True

When the ``liveStream`` flag is set to **True**, BSK will also automatically obtain user inputs from Vizard
at each timestep, as detailed in the accompanying section.

.. note::
    BSK can only take advantage of 2-way communication during livestream mode, so ``liveStream`` must be
    set to **True**.

Attaching a "DirectComm" Vizard Instance
----------------------------------------
Once BSK is configured to livestream, an instance of Vizard can be attached by copying the port address
into the "*Socket Address*" field. Make sure that the "*DirectComm*" option is selected.

The communication protocol and port number can be changed using:

.. code-block:: python

    viz.reqComProtocol = "xxx";
    viz.reqComAddress  = "xxx";
    viz.reqPortNumber  = "XXXX";

BSK assembles the full ``liveStream`` address as::

    viz.reqComProtocol + "://" + viz.reqComAddress + ":" + viz.reqPortNumber

.. list-table:: Vizard 2-way Communication Parameters
    :widths: 10 10 10 70
    :header-rows: 1

    * - Variable
      - Type
      - Default
      - Description
    * - ``reqComProtocol``
      - string
      - ``tcp``
      - Transport protocol to use for the 2-way connection. See all protocol options
        `here <http://api.zeromq.org/3-2:zmq-connect>`__.
    * - ``reqComAddress``
      - string
      - ``localhost``
      - Network address to use for the connection.  This can be either a host name or an IP address.
    * - ``reqPortNumber``
      - string
      - ``5556``
      - Port number to use for the connection.

The default 2-way address is therefore ``tcp://localhost:5556``.


Creating ``VizEventDialog`` Panels
----------------------------------
``VizEventDialog`` panels are displayed in Vizard, and contain choices for a user to select. These panels
can be configured to have a prescribed duration and several options for a user to select. The following
list shows all ``VizEventDialog`` structure variables.

.. list-table:: ``VizEventDialog`` variables
    :widths: 20 10 10 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``eventHandlerID``
      - string
      -
      - Yes
      - Name of Vizard event handler to be returned with ``VizEventReply`` responses
    * - ``displayString``
      - string
      -
      - Yes
      - Contains the information or choice that should be posed to the user.
    * - ``userOptions``
      - string[]
      -
      - No
      - Array of display strings, one entry for each user choice that will be shown. If this is
        empty, the dialog is assumed to be informational only. Strings must be unique (cannot display
        the same option multiple times in a panel).
    * - ``durationOfDisplay``
      - double
      - nanoseconds
      - No
      - Determines when to close a panel. Default is 0, which leaves the panel on display until closed by user.
    * - ``useSimElapsedTimeForDuration``
      - bool
      -
      - No
      - If true and ``durationOfDisplay`` is set, use the sim elapsed time to calculate when to
        hide window. If false, use real time (system clock). Default is false.
    * - ``useConfirmationPanel``
      - int
      -
      - No
      - -1 to not show a confirmation panel, 0 to use viz default, 1 to require a user confirmation
        of their selection.
    * - ``hideOnSelection``
      - int
      -
      - No
      - -1 to continue to show panel , 0 to use viz default, 1 to hide panel after user
        makes a selection, 2 to destroy panel after user makes a selection
    * - ``dialogFormat``
      - string
      -
      - No
      - Select format for dialog box: "WARNING", "CAUTION", or none to use viz default format


Here is an example an ``VizEventDialog`` panel creation:

.. code-block:: python

    powerModePanel = vizInterface.VizEventDialog()
    powerModePanel.eventHandlerID = "Power Mode Panel"
    powerModePanel.displayString = "Set system power mode:"
    powerModePanel.userOptions.append("Nominal")
    powerModePanel.userOptions.append("Low-Power")
    powerModePanel.useConfirmationPanel = True

    viz.vizEventDialogs.append(powerModePanel)

See :ref:`scenarioBasicOrbitStream` for a Basilisk example script that uses Vizard event panels.

.. note::
    The list ``viz.vizEventDialogs`` sends current panel requests to Vizard as part of the
    VizMessage, then clears itself before the next timestep. If information in a panel needs to
    be modified, the same ``VizEventDialog`` instance (with the same ``eventHandlerID``) can be
    modified and **re-appended** to ``viz.vizEventDialogs``. This will cause the panel to re-open
    if minimized, with updated information. If the panel list needs to be manually cleared, this
    can be done using ``viz.vizEventDialogs.clear()``.


Handling User Input
^^^^^^^^^^^^^^^^^^^
Responses from panels can be used as inputs back to BSK. The key is that the responses must be
read from the :ref:`VizUserInputMsgPayload` message at the desired rate.

From Python, this can be achieved by calling ``scSim.ExecuteSimulation()`` at the desired input
reception rate so that responses can be parsed and used to influence the simulation state.

The required structure resembles the following:

.. code-block:: python

    currentTime = 0
    inputTimeStep = macros.sec2nano(5) # Read inputs every 5 seconds
    ...
    scSim.InitializeSimulation()
    for i in range(int(totalDuration/inputTimeStep)):
        currentTime += inputTimeStep
        scSim.ConfigureStopTime(currentTime)
        scSim.ExecuteSimulation()

        userInputs = viz.userInputMsg.read()
        keyInputs = userInputs.keyboardInput
        eventInputs = userInputs.vizEventReplies

        # Parse "keyInputs" and "eventInputs", modify sim state

The 2-way communication output message, ``viz.userInputMsg`` , is an instance
of :ref:`VizUserInputMsgPayload`. This message fills like a queue: Vizard collects all inputs
that were recorded over the last ``scSim.ExecuteSimulation`` call, and hands them all over together.

.. caution::
    Setting a low input frequency (here, represented by ``inputTimeStep``) could lead to build-up of
    responses, that could logically conflict with one another.

This behavior could also be built into a BSK module, in which case the above code structure would
not be needed. However, this module would have to hard-code the mappings for different Vizard response
types and their associated BSK actions.

Keyboard Parsing
^^^^^^^^^^^^^^^^
There are two types of replies that Vizard can send in return. The ``keyboardInput`` field of the
message contains a string of keyboard characters that have been pressed since the last timestep.
**Keys will only be recorded if pre-specified.** In the example below, listeners are configured
for the keys 'a', 'b', 'c', and 'd':

.. code-block:: python

    viz.settings.keyboardLiveInput = "abcd"

.. caution::
    Note that Vizard has certain keys pre-programmed as hot-keys for menus and scene actions.
    If a hot-key is selected as a duplicate listener, Vizard will display a warning, and
    dual-actions may occur.

To parse ``keyInputs``, search the string for characters of interest:

.. code-block:: python

    if 'a' in keyInputs:
        # 'a' key action
    if 'b' in keyInputs:
        # 'b' key action
    ...

Panel Response Parsing
^^^^^^^^^^^^^^^^^^^^^^
Vizard can also return ``VizEventReply`` structures, which contain information about selections
made within ``VizEventDialog`` panels. The following list shows all ``VizEventReply`` structure variables.

.. list-table:: ``VizEventReply`` variables
    :widths: 20 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Description
    * - ``eventHandlerID``
      - string
      - Name provided when setting up the VizEventDialog object
    * - ``reply``
      - string
      - Option selected by user
    * - ``eventHandlerDestroyed``
      - bool
      - Was the panel closed and destroyed?


To parse ``eventInputs`` , loop over the list:

.. code-block:: python

    for response in eventInputs:
        eventID = response.eventHandlerID
        eventOption = response.reply

        if eventID == "Power Mode Panel":
            if eventOption == "Low-Power":
                # change mode
        elif eventID == ...

See the scenario :ref:`scenarioBasicOrbitStream` for an implemented 2-way communication example.


``broadcastStream`` Mode
========================
In addition to livestreaming, BSK can broadcast its read-only messages to a secondary port
using the ``broadcastStream`` flag. This follows a publish/subscribe architecture, which means many
subscriber Vizards can hook up to a single BSK instance.

To enable broadcasting use the ``broadcastStream`` flag:

.. code-block:: python

    viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, scObject
                                              , broadcastStream=True
                                              )

.. note::
    BSK blindly broadcasts to a port, regardless of whether there are subscribers present or not. This means
    that unlike ``liveStream`` mode, BSK will not wait for a Vizard connection to begin executing.

Vizard can also run both ``liveStream`` and ``broadcastStream`` modes simultaneously:

.. code-block:: python

    viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, scObject
                                              , liveStream=True
                                              , broadcastStream=True
                                              )

This configuration behaves the same as a normal livestream (BSK waits for connection to start), after which
subscriber Vizards can connect. Subscriber Vizards cannot connect before the publisher Vizard.

Broadcast Synchronization
-------------------------
Using ``liveStream`` and ``broadcastStream`` modes simultaneously can be useful for teaching/training
environments, where an instructor is running BSK locally and interacting with panels in a publisher Vizard,
but broadcasting to separate subscriber Vizards run by students/trainees.

The trainer has the option to force certain visual settings to the broadcast:

.. list-table:: ``broadcastStream`` synced settings
    :widths: 30
    :header-rows: 1

    * - Variable
    * - ``orbitLinesOn``
    * - ``trueTrajectoryLinesOn``
    * - ``spacecraftCSon``
    * - ``planetCSon``
    * - ``showHillFrame``

See :ref:`vizardSettings` for types and descriptions of these variables.

This option is ON by default, but can be disabled within the menu of the trainer Vizard. Additionally,
panel states (instantiation, selection, deletion) will be synchronized with the subscriber Vizard. Choices
selected on panels are shown briefly in the subscriber Vizard before closure.

When disabled, the subscriber Vizard will be briefly notified that they have increased autonomy with visual
settings. However, no interactions with panels will have any effect on the BSK sim.

.. note::
    Subscriber Vizards are able to move the time slider at the bottom of the window, however panels will not replay.


Attaching a "Receive Only" Vizard Instance
------------------------------------------
Once BSK is configured to broadcast, an instance of Vizard can be attached by copying the port address
into the "*Socket Address*" box. Make sure that the "*Receive Only*" option is selected.

The communication protocol and port number can be changed from Python using:

.. code-block:: python

    viz.pubComProtocol = "xxx";
    viz.pubComAddress  = "xxx";
    viz.pubPortNumber  = "XXXX";

BSK assembles the full ``broadcastStream`` address as::

    viz.pubComProtocol + "://" + viz.pubComAddress + ":" + viz.pubPortNumber

.. list-table:: Vizard Broadcast Communication Parameters
    :widths: 10 10 10 70
    :header-rows: 1

    * - Variable
      - Type
      - Default
      - Description
    * - ``pubComProtocol``
      - string
      - ``tcp``
      - Transport protocol to use for the 2-way connection. See all protocol options
        `here <http://api.zeromq.org/3-2:zmq-connect>`__.
    * - ``pubComAddress``
      - string
      - ``localhost``
      - Network address to use for the connection.  This can be either a host name or an IP address.
    * - ``pubPortNumber``
      - string
      - ``5570``
      - Port number to use for the connection.

The default broadcast address is therefore ``tcp://localhost:5570``.

.. note::
    Although the default TCP ports seemed to be available across our development machines, specific
    firewall restrictions or other applications may restrict port use. If the port experiences trouble
    binding, a ZMQ error will be thrown to the terminal and a different port number should be chosen.
    (Error #19 if port is in use, Error #49 if address:port does not exist.)

In order to broadcast temporarily over a network, a dynamic IP address can be used. With both devices on
the same network, set the broadcast address as:

.. code-block:: python

    viz.pubComAddress = "XX.XXX.XXX.XX"

.. caution::
    Some networks may block broadcasting. If the broadcast has trouble connecting, check whether the
    two devices can ping each other across the network. The terminal command is ``ping XX.XXX.XXX.XX``
    for Mac, or ``telnet XX.XXX.XXX.XX`` for Windows and Linux.

If you're looking to broadcast for longer durations (i.e. more than a few hours), you may need to set
up a static IP address. IP addresses are typically assigned dynamically, meaning that they can change
unpredictably throughout the day. Setting up a static IP address ensures that the broadcast can be
found at the same address consistently.


``noDisplay`` Mode
==================
Vizard can operate headless with a high-performance optical navigation mode, in which it only
updates and renders the scene when an image is requested from BSK. This mode does not show a display
to the user, which means that it *cannot* be used in conjunction with ``liveStream`` or
``broadcastStream`` modes.

To use the ``noDisplay`` flag:

.. code-block:: python

	viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, scObject
                                              , noDisplay=True
                                              )
