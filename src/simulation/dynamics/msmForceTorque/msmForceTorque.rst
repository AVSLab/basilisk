Executive Summary
-----------------
This module uses the Multi-Sphere-Method (MSM) to evaluate the mutual electrostatic force and torque interactions between a series of spacecraft object.  The charging is specified through a voltage where the object is assumed to have aconstant voltaged across the surface.  The MSM model for each space object is given through a list of body-fixed sphere locations and sphere radii.  See `Multi-Sphere Method for Modeling Electrostatic Forces and Torques <http://dx.doi.org/10.2514/1.52185>`__ for more information on the MSM method.  The goal of this module is to simulate charged astrodynamics and include the influence of charging on relative motion.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  
The module msg connection is set by the user from python.  
The msg type contains a link to the message structure definition, while the description 
provides information on what this message is used for.

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
      - :ref:`VoltageMsgPayload`
      - vector of voltage input messages
    * - eTorqueOutMsgs
      - :ref:`CmdTorqueBodyMsgPayload`
      - vector of E-torques in body frame components
    * - eForceOutMsgs
      - :ref:`CmdForceBodyMsgPayload`
      - vector of E-forces in body frame components

