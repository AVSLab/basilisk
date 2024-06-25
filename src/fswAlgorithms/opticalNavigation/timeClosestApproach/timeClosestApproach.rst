Executive Summary
-----------------
This module computes the time of closest approach estimation and its covariance during a rectilinear flyby. It was written based on the paper "Attitude Uncertainty Quantification Of Rectilinear Asteroid Flyby For The Emirates Mission Th The Asteroid Belt" that was presented by Thibaud Teil and Riccardo Calaon in 2023.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - filterInMsg
      - :ref:`FilterMsgPayload`
      - Input message containing the relative spacecraft state (position and velocity) and it's covariance. "of the spacecraft with respect to the small body, estimated from a filter"
    * - tcaOutMsg
      - :ref:`TimeClosestApproachMsgPayload`
      - Output time of closest approach message containing time of closest approach during the flyby and the covariance.

Detailed Module Description
---------------------------
The relative position and velocity vector and the covariance of the spacecraft with respect to the small body are obtained as filter estimates. The input parameter ``filterInMsg`` that provides an initial spacecraft state and covariance. This module assumes a rectilinear flyby and receives a current state and output the time of closest approach estimate and it's covariance.

Rectilinear Motion Model
........................
In this case the flyby is modeled as rectilinear motion of the spacecraft, i.e., the spacecraft moves with a constant velocity. At every filter read, the relative position and velocity vectors :math:`\boldsymbol{r}` and :math:`\boldsymbol{v}` of the spacecraft with respect to the small body are provided. The following coefficients are defined: the flight path angle :math:`\gamma_0= \theta - \frac{\pi}{2}` of the spacecraft, and the ratio between velocity and radius magnitudes :math:`f_0 = \frac{v_0}{r_0}`. From these quantities, the angle between -:math:`\boldsymbol{r}` and :math:`\boldsymbol{v}` of the spacecraft with the respect to the asteroid which is defined as :math:`\theta` the following equation shows how it can be found:

.. math::
    \theta = \arccos\left( -  \mathbf{\hat{r}}  \cdot  \mathbf{\hat{v}} \right )
By replacing the values of :math:`\gamma_0` and  :math:`f_0` we can obtain the time of closest approach through this equation:

.. math::
    t_{CA} = - \frac{\sin(\gamma_0)}{f_0}
The following equation solves for time of closest approach uncertainty where :math:`P` is the spacecraft state covariance.

.. math::
    \sigma_{t_{CA}}^2 = \frac{1}{f_0^2}  \left[ \frac{\mathbf{\hat{v}}^T}{\ \mathbf{r} } \ \frac{{1}}{ \mathbf{v} } \  \left( \mathbf{\hat{r}}^T - \sin \gamma_0 \mathbf{\hat{v}}^ T \right)] [{P}] (t) [\left[ \frac{\mathbf{\hat{v}}^T}{\ \mathbf{r} } \ \frac{{1}}{\ \mathbf{v} } \  \left( \mathbf{\hat{r}}^T - \sin\gamma_0 \mathbf{\hat{v}}^ T \right)]^T




Module Assumptions and Limitations
----------------------------------
This module assumes the flyby to be a rectilinear motion of the spacecraft, meaning the spacecraft moves with a constant velocity. The limitations of this module are inherent to the geometry of the problem, which determines whether or not all the constraints can be satisfied. For example, one constraint for this module to work is that the target should have a small gravitational influence, implying its mass is relatively low. Another constraint is that the distance between the target and the spacecraft should be large enough to minimize the gravitational effects of the target on the spacecraft


User Guide
----------
The required module configuration is::

    tca_module = timeClosestApproach.TimeClosestApproach()
    tca_module.ModelTag = "TimeClosestApproach"
    unitTestSim.AddModelToTask(unitTaskName, tca_module)

The module is configurable with the following parameters:

.. list-table:: Module Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``FlightPathAngle``
     - π / 2
     - The flight path angle is the orientation of the velocity vector with the respect to the position vector.
   * - ``Ratio``
     - 0
     - Ratio between relative velocity and position norms at time of read
   * - ``tCA``
     - 0
     - Time of closest approach.
   * - ``sigmaTca``
     - 0
     - Standard deviation of Time closest approach
