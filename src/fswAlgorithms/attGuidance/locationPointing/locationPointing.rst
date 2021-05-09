Executive Summary
-----------------
This module generates an attitude guidance message to make a specified spacecraft pointing vector target an inertial location.
This location could be on a planet if this module is connected with :ref:`groundLocation` for example.  

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
    * - scInMsg
      - :ref:`SCStatesMsgPayload`
      - input msg with inertial spacecraft states 
    * - LocationInMsg
      - :ref:`GroundStateMsgPayload`
      - input msg containing the inertial point location of interest
    * - AttGuidOutMsg
      - :ref:`AttGuidMsgPayload`
      - output message with the attitude guidance



Detailed Module Description
---------------------------
The inertial location of interest is given by :math:`{\bf r}_{L/N}`. The vector pointing from the
satellite location :math:`{\bf r}_{S/N}` to this location is then

.. math::
    {\bf r}_{L/S} = {\bf r}_{L/N} - {\bf r}_{S/N}

Let :math:`\hat{\bf r}_{L/S}` be the normalized heading vector to this location.

The unit vector :math:`\hat{\bf p}` is a body-fixed vector and denotes the body axis which is to point towards
the desired location :math:`L`.  Thus this modules performs a 2-degree of freedom attitude guidance and
control solution.

The eigen-axis to rotate :math:`\hat{\bf p}` towards :math:`\hat{\bf r}_{L/S}` is given by

.. math::

    \hat{\bf e} = \frac{\hat{\bf p} \times \hat{\bf r}_{L/S}}{|\hat{\bf p} \times \hat{\bf r}_{L/S}|}

The principle rotation angle :math:`\phi` is

.. math::

    \phi = \arccos (\hat{\bf p} \cdot \hat{\bf r}_{L/S} )

The attitude tracking error :math:`{\pmb\sigma}_{B/R}` is then given by

.. math::

    {\pmb\sigma}_{B/R} = - \tan(\phi/4) \hat{\bf e}

The tracking error rates :math:`{\pmb\omega}_{B/R}` are obtained through numerical differentiation of the
MRP values.  During the first module ``Update`` evaluation the numerical differencing is not possible and
this value is thus set to zero.

The inertial reference frame rate is then given by

.. math::

    {\pmb\omega}_{R/N} = {\pmb\omega}_{B/N} - {\pmb\omega}_{B/R}

The inertial reference frame is found by doing an numerical derivative of :math:`{\pmb\omega}_{R/N}`.
The the first 2 ``Update`` evaluations the inertial acceleration cannot be determined and thus is
set to zero.



.. note::

    The module checks for several conditions such as heading vectors
    being collinear, the MRP switching during the numerical differentiation, etc.



User Guide
----------
The one required variable that must be set is ``pHat_B``.  This is body-fixed unit vector which is to be
pointed at the desired inertial location.

The variable ``smallAngle`` defined the minimum angular separation where two vectors are considered colinear.
It is defaulted to zero, but can be set to any desired value in radians.




