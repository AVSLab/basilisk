Executive Summary
-----------------
This module, a sub-class of :ref:`dynamicEffector`, implements a first order gravity gradient torque acting on a spacecraft.  It is written
in a general manner such that one or more gravitational objects are considered.  This allows a continues simulation to
apply gravity gradients torque near the Earth, the moon and onwards to Mars.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - gravityGradientOutMsg
      - :ref:`GravityGradientMsgPayload`
      - gravity gradient output message



Detailed Module Description
---------------------------
A first order gravity gradient torque is implemented as discussed in chapter 4 of `Analytical Mechanics of Space
Systems <https://doi.org/10.2514/4.105210>`_.  Let :math:`[I_c]` be the total inertia tensor about the spacecraft
center of mass location `C`.  Note this :math:`[I_c]` can vary in time in this effector.  Thus, if the
spacecraft has a time-varying mass distribution (flexing panels, deploying structures, fuel slosh, etc.), this
effector retrieves the current :math:`[I_c]` value from the dynamics state engine.

Assume a planet center inertial position vector is given by :math:`{\bf R}_{P_i/N}`.  If there are `N` planets
contributing to the net gravity gradient torque, then this is evaluated using

.. math::
    :label: eq-gg-Lg

    {\bf L}_G = \sum_{i=1}^N \frac{3 \mu}{| {\bf R}_{C/P_i} |^5} {\bf R}_{C/P_i} \times [I_c] {\bf R}_{C/P_i}

The spacecraft location relative to the :math:`i^{\text{th}}` planet is

.. math::
    :label: eq-qq-RCPi

    {\bf R}_{C/P_i} = {\bf R}_{C/N} - {\bf R}_{P_i/N}

As a spacecraft leaves the sphere of influence of a planet the gravity gradient torque contribution become
vanishingly small.  This is equivalent to how gravity accelerations are computed relative to all gravitational
bodies, regardless of how far away they are.  At every time step the gravity gradient effectors is able to
pull from the state engine the current planet locations allowing arbitrary integration methods to be used
with this external torque.


Module Assumptions and Limitations
----------------------------------
The effector assumes that a first order gravity gradient torque solution is sufficient to solve the
dynamical system.


User Guide
----------

Basic Setup
^^^^^^^^^^^
The gravity effector setup follows the standard process of creating the effector and asigning it to a
spacecraft as well as adding it to the task list::

    ggEff = GravityGradientEffector.GravityGradientEffector()
    ggEff.ModelTag = scObject.ModelTag
    scObject.addDynamicEffector(ggEff)
    scSim.AddModelToTask(simTaskName, ggEff)

Specifying Gravity Bodies
^^^^^^^^^^^^^^^^^^^^^^^^^
To specify which planets must be considered for gravity gradient torques, use the command::

    ggEff.addPlanetName("name")

where ``name`` should be the Spice planetary output name.  For example, for earth this is `earth_planet_data`.  If
the ``gravBodyFactory`` class is used to setup planets, then the ``planetName`` message will contain this
information::

    ggEff.addPlanetName(earth.planetName)

.. warning::
    The effector requires at least one planet to be specified.

.. note::
    If you added `N` gravity bodies for the gravitational acceleration consideration, you don't have to add all of
    these objects to the gravity gradient effector as well.  It is ok to just add a subset as well.  However, any
    gravity body added to the gravity gradient effector must also have been added as a gravitational body to the
    spacecraft.

Module Output Message Name
^^^^^^^^^^^^^^^^^^^^^^^^^^
The effector write an output message with the current gravity gradient torque information at each ``update`` cycle.
The output message is ``gravityGradientOutMsg``.

