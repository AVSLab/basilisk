.. _bskPrinciples-11:

Advanced: Effector Module Branching
===================================

Module branching in Basilisk enables the attachment of compatible dynamic effectors on state
effectors as an alternative to attaching them directly to the hub. For example, a gimballed thruster
can be modeled as a ``thrusterDynamicEffector`` attached to a ``spinningBodyTwoDOFStateEffector``,
or a robotic docking arm can be modeled as a ``constraintDynamicEffector`` attached to a 7-DOF
revolute ``spinningBodyNDOFStateEffector``. Multiple dynamic effectors can be attached to a state
effector as well, for example applying both drag and SRP to a solar panel.

Further details on how to set up new state effectors and dynamic effector modules for branching can
be found in :ref:`effectorBranching`. Integrated testing of effector branching is elaborated on in
:ref:`test_effectorBranching_integrated`.

Allowable Configurations
------------------------
The currently supported configurations of state effectors compatible as "parent" effectors and
dynamic effectors compatible as "child" effectors is summarized in the table below. Green shows
currently supported configurations, yellow shows configurations expected to be supported in the
future, and red shows configurations not planned to be supported. Additionally, blue shows the
special case of prescribed effector branching explained in :ref:`prescribedMotionStateEffector`.

.. raw:: html

    <style>
      /* Responsive wrapper */
      .table-wrapper {
        overflow-x: auto;
        -webkit-overflow-scrolling: touch;
        margin: 1em 0;
      }

      /* Table base */
      table.effector-compat {
        border-collapse: collapse;
        table-layout: fixed;              /* consistent sizing across browsers */
        width: 100%;
        text-align: center;
        font-size: 0.9em;
      }

      /* Cells */
      table.effector-compat th,
      table.effector-compat td {
        border: 1px solid #777;
        padding: 6px 8px;
      }

      /* Header cells */
      table.effector-compat thead th {
        background-color: #e5e5e5;
        color: #222;
        font-weight: bold;
      }

      /* Second column (row labels) */
      table.effector-compat td.label {
        background-color: #f5f5f5;
        color: #222;
        font-weight: bold;
        white-space: normal;          /* allow wrapping */
        overflow-wrap: anywhere;      /* wrap long words */
        text-align: left;             /* readability */
      }

      /* Rotated column headers (use inner wrapper, not the <th> itself) */
      table.effector-compat th.rotate {
        vertical-align: middle;
        background-color: #d9d9d9;
        padding: 0;                   /* inner span handles spacing */
        height: 140px;                /* stable header height */
        width: 38px;                  /* narrow columns for tall text */
      }
      @supports (writing-mode: vertical-rl) {
        table.effector-compat th.rotate > .rotate-inner {
          writing-mode: vertical-rl;
          text-orientation: mixed;
          transform: rotate(180deg);  /* bottom → top like original */
          display: inline-block;
          white-space: nowrap;
          padding: 8px 4px;
          line-height: 1.1;
        }
      }
      @supports not (writing-mode: vertical-rl) {
        table.effector-compat th.rotate > .rotate-inner {
          display: inline-block;
          transform: rotate(-90deg);
          transform-origin: center;
          white-space: nowrap;
          padding: 8px 4px;
          line-height: 1.1;
        }
      }

      /* FIRST COLUMN (vertical "Parent Effectors") — rotate-only, no writing-mode */
      table.effector-compat td.vlabel {
        position: relative;     /* anchor for absolute child */
        padding: 0;
        text-align: center;
        overflow: hidden;       /* keep content inside the cell */
      }
      table.effector-compat td.vlabel .rotate-inner {
        position: absolute;
        top: 50%;
        left: 50%;
        transform: translate(-50%, -50%) rotate(-90deg);  /* center + rotate */
        transform-origin: center;
        display: block;
        white-space: nowrap;
        line-height: 1;
        padding: 6px 4px;
        font-weight: bold;
        color: #222;
      }
      @supports (writing-mode: vertical-rl) {
        table.effector-compat td.vlabel .rotate-inner {
          writing-mode: initial !important;
          text-orientation: initial !important;
        }
      }

      /* Column widths via <colgroup> */
      table.effector-compat col.col-vert  { width: 32px; }   /* first column */
      table.effector-compat col.col-label { width: 280px; }  /* second column */
      table.effector-compat col.col-data  { width: 32px; }   /* each data column */

      /* Color palette */
      table.effector-compat td.green  { background-color: #90ee90; }
      table.effector-compat td.yellow { background-color: #fff59d; }
      table.effector-compat td.red    { background-color: #ff9999; }
      table.effector-compat td.blue   { background-color: #9ec5fe; }

      /* Caption */
      table.effector-compat caption {
        caption-side: top;
        text-align: left;
        font-weight: bold;
        margin-bottom: 0.5em;
        font-size: 1.05em;
      }
    </style>

    <div class="table-wrapper">
    <table class="effector-compat">
      <caption>Effector Branching Compatibility</caption>

      <!-- IMPORTANT: <colgroup> must be AFTER caption, BEFORE thead -->
      <colgroup>
        <col class="col-vert">            <!-- 1st column: vertical label -->
        <col class="col-label">           <!-- 2nd column: wider, wraps -->
        <col class="col-data" span="16">  <!-- 16 child-effector columns -->
      </colgroup>

      <thead>
        <tr>
          <th rowspan="2" colspan="2"></th>
          <th colspan="16">Children Effectors</th>
        </tr>
        <tr>
          <th class="rotate"><span class="rotate-inner">Thruster Dynamic Effector</span></th>
          <th class="rotate"><span class="rotate-inner">Thruster State Effector</span></th>
          <th class="rotate"><span class="rotate-inner">Constraint Effector</span></th>
          <th class="rotate"><span class="rotate-inner">Drag Effector</span></th>
          <th class="rotate"><span class="rotate-inner">External Force Torque</span></th>
          <th class="rotate"><span class="rotate-inner">SRP Effector</span></th>
          <th class="rotate"><span class="rotate-inner">Fuel Tank Effector</span></th>
          <th class="rotate"><span class="rotate-inner">Gravity Effector</span></th>
          <th class="rotate"><span class="rotate-inner">Spinning Bodies</span></th>
          <th class="rotate"><span class="rotate-inner">Translating Bodies</span></th>
          <th class="rotate"><span class="rotate-inner">Hinged Rigid Bodies</span></th>
          <th class="rotate"><span class="rotate-inner">Prescribed Bodies</span></th>
          <th class="rotate"><span class="rotate-inner">Linear Spring Mass Damper</span></th>
          <th class="rotate"><span class="rotate-inner">Spherical Pendulum</span></th>
          <th class="rotate"><span class="rotate-inner">Reaction Wheel</span></th>
          <th class="rotate"><span class="rotate-inner">VSCMG</span></th>
        </tr>
      </thead>

      <tbody>
        <tr>
          <td rowspan="9" class="label vlabel">
            <span class="rotate-inner">Parent Effectors</span>
          </td>
          <td class="label">Spinning Bodies 1DOF</td>
          <td class="green"></td><td class="yellow"></td><td class="green"></td><td class="yellow"></td>
          <td class="green"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
        </tr>
        <tr>
          <td class="label">Spinning Bodies 2DOF</td>
          <td class="green"></td><td class="yellow"></td><td class="green"></td><td class="yellow"></td>
          <td class="green"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
        </tr>
        <tr>
          <td class="label">Spinning Bodies NDOF</td>
          <td class="green"></td><td class="yellow"></td><td class="green"></td><td class="yellow"></td>
          <td class="green"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
        </tr>
        <tr>
          <td class="label">Hinged Rigid Bodies</td>
          <td class="yellow"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="yellow"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
        </tr>
        <tr>
          <td class="label">Dual Hinged Rigid Bodies</td>
          <td class="yellow"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="yellow"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
        </tr>
        <tr>
          <td class="label">N Hinged Rigid Bodies</td>
          <td class="yellow"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="yellow"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
        </tr>
        <tr>
          <td class="label">Linear Translating Bodies 1DOF</td>
          <td class="yellow"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="yellow"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
        </tr>
        <tr>
          <td class="label">Linear Translating Bodies NDOF</td>
          <td class="yellow"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="yellow"></td><td class="yellow"></td><td class="yellow"></td><td class="yellow"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
          <td class="red"></td><td class="red"></td><td class="red"></td><td class="red"></td>
        </tr>
        <tr>
          <td class="label">Prescribed Motion</td>
          <td class="blue"></td><td class="blue"></td><td class="blue"></td><td class="blue"></td>
          <td class="blue"></td><td class="blue"></td><td class="blue"></td><td class="blue"></td>
          <td class="blue"></td><td class="blue"></td><td class="blue"></td><td class="blue"></td>
          <td class="blue"></td><td class="blue"></td><td class="blue"></td><td class="blue"></td>
        </tr>
      </tbody>
    </table>
    </div>

Setup
-----
Attaching a dynamic effector on a state effector is performed the same way as attaching a dynamic
effector to the hub, but instead called with the state effector. Given a setup as::

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    scObject = spacecraft.Spacecraft()

    stateEff = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
    dynEff = extForceTorque.ExtForceTorque()

    scObject.addStateEffector(stateEff)

    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, stateEff)
    scSim.AddModelToTask(simTaskName, dynEff)

By default, effectors are attached to the hub using::

    scObject.addDynamicEffector(dynEff)
    scSim.AddModelToTask(simTaskName, dynEff)

Dynamic effector's can instead be attached to a state effector as::

    stateEff.addDynamicEffector(dynEff)

Or in the case of a multi-degree-of-freedom parent effector, additionally specify which segment to
attach the dynamic effector to as::

    stateEff = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    segment = 2
    stateEff.addDynamicEffector(dynEff, segment)

Where segment is the integer number of the segement to be attached to, with 1 being the segment
attached to the hub, increasing outward from the hub. For example, spinningBodyOneDOFStateEffector
will always attach to segment 1, attaching to the second to last segment of a 7DOF robotic arm would
be segment 6. Note that in either case the dynamic effector is still added to the sim task.

However, some effectors use alternative methods for adding to a spacecraft. For example, the
attachment of a :ref:`thrusterDynamicEffector` set using the :ref:`simIncludeThruster`::

    thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    thFactory = simIncludeThruster.thrusterFactory()
    thFactory.create('MOOG_Monarc_22_6', [0, 0, 0], [0, -1.5, 0])

    thFactory.addToSpacecraft(thrModelTag, thrusterSet, scObject)

Would instead be attached using::

    thFactory.addToSpacecraftSubcomponent(thrModelTag, thrusterSet, stateEff, segment)

Additionally, the :ref:`constraintDynamicEffector` requires attachment to two parents. It works with
either a hub attached to the state effector of another vehicle, or the attachment of two different
state effectors to one another.::

    scObject2 = spacecraft.Spacecraft()
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()

    stateEff.addDynamicEffector(constraintEffector)
    scObject2.addDynamicEffector(constraintEffector)
