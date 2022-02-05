Executive Summary
-----------------
This module uses the Multi-Sphere-Method (MSM) to evaluate the mutual electrostatic force and torque interactions between a series of spacecraft object.  The charging is specified through a voltage where the object is assumed to have a constant voltage across the surface.  The MSM model for each space object is given through a list of body-fixed sphere locations and sphere radii.  See `Multi-Sphere Method for Modeling Electrostatic Forces and Torques <http://dx.doi.org/10.1016/j.asr.2012.08.014>`__ for more information on the MSM method.  The goal of this module is to simulate charged astrodynamics and include the influence of charging on relative motion.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  
The module msg connection is set by the user from python.  
The msg type contains a link to the message structure definition, while the description 
provides information on what this message is used for.

.. _ModuleIO_MSM_FORCE_TORQUE:
.. figure:: /../../src/simulation/dynamics/msmForceTorque/_Documentation/Images/moduleMsmForceTorque.svg
    :align: center

    Figure 1: ``msmForceTorque()`` Module I/O Illustration

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - scStateInMsgs
      - :ref:`SCStatesMsgPayload`
      - vector of spacecraft state input messages
    * - voltInMsgs
      - :ref:`VoltMsgPayload`
      - vector of voltage input messages
    * - eTorqueOutMsgs
      - :ref:`CmdTorqueBodyMsgPayload`
      - vector of E-torques in body frame components
    * - eForceOutMsgs
      - :ref:`CmdForceInertialMsgPayload`
      - vector of E-forces in inertial frame components
    * - chargeMsmOutMsgs
      - :ref:`ChargeMsmMsgPayload`
      - vector of MSM charge messages


Module Assumptions and Limitations
----------------------------------
This module assumes the electrostatic torques and forces are constant during the integration step.


User Guide
----------
The electrostatic force and torque module is created using:

.. code-block:: python
    :linenos:

    module = msmForceTorque.MsmForceTorque()
    module.ModelTag = "msmForceTorqueTag"
    unitTestSim.AddModelToTask(unitTaskName, module)

To add a spacecraft, along with the associated list of MSM sphere body-fixed locations and radii, use::

    module.addSpacecraftToModel(scStateMsg,
                                , messaging.DoubleVector(radii_List)
                                , unitTestSupport.npList2EigenXdVector(r_SB_B_List)
                                )

where ``radii_list`` is a list of sphere radii, and ``r_SB_B_List`` is a list of body-fixed MSM sphere locations.
This command creates the corresponding output messages in the message vectors ``eTorqueOutMsgs`` and
``eForceOutMsgs``.  Thus, the e-force of the first space object is accessed through ``module.eForceOutMsgs[0]``, etc.

The ``addSpacecraftToModel`` also creates a corresponding voltage input message in ``module.voltInMsgs[i]``
where ``i`` is the number in which the spacecraft object was added.

.. note::

   If MSM spheres of one spacecraft become too close to spheres of another spacecraft (i.e.
   center-to-center distance less than the sphere radius), then a warning statement is provided.  In such
   situations the MSM accuracy is beginning to break down.
