Executive Summary
-----------------

This module filters position measurements that have been processed from planet images in order to
estimate spacecraft relative position to an observed body in the inertial frame.
The filter used is a square root unscented Kalman filter, and the images are first processed by image processing
and a measurement model in order to produce this filter's measurements.

The module
:download:`PDF Description </../../src/fswAlgorithms/opticalNavigation/relativeODuKF/_Documentation/inertialUKF_DesignBasis.pdf>`
contains further information on this module's mathematical framework.

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
    * - navTransOutMsg
      - :ref:`NavTransMsgPayload`
      - navigation translation output message
    * - opNavFilterMsg
      - :ref:`OpNavSUKFMsgPayload`
      - output filter data message containing states and covariances
    * - opNavHeadingMsg
      - :ref:`OpNavUnitVecMsgPayload`
      - opnav input message containing the unit vector towards the target

Module models
-------------------------------
The measurement model for the filter is functionally contained in the center of brightness
converter module. The message read in therefore contains the predicted measurement:

.. math::

    H[X] = \frac{X[0:3]}{\|X[0:3]\|} = \hat{r}

The dynamics modules used are point mass, single body gravity, propagated with and RK4
integrator

.. math::

    F[X] = \dot{X} = - \frac{\mu}{X[0:3]^3}X[0:3] = - \frac{\mu}{r^3}r

where :math:`r` is the position of the spacecraft center of masss relative to the central body
of gravitational parameter :math:`\mu`.


Module assumptions and limitations
-------------------------------

The module inherits all assumptions made while implementing a Kalman filter:
    • Observability considerations
    • Gaussian covariances
    • Linearity limits
    • and more

Otherwise, the limitations are governed by the measurement model and dynamics models relative
to the required performance.

User Guide
----------
This section is to outline the steps needed to setup a flybyODSRuKF converter in Python.

#. Import the module::

    from Basilisk.fswAlgorithms import flybyODuKF

#. Create an instantiation of converter class::

    flybyOD = flybyODuKF.FlybyODuKF()

#. Setup SRuKF general parameters::

    flybyOD.alpha = 0.02
    flybyOD.beta = 2.0

#. Setup SRuKF measurement parameters::

    flybyOD.muCentral = 1
    flybyOD.measNoiseScaling = 1

#. Setup initial state and covariances::

    flybyOD.stateInitial = [[1000.*1e3], [1000.*1e3], [1000.*1e3], [0.], [1.*1e3], [0.]]
    flybyOD.covarInitial =[ [10., 0., 0., 0., 0., 0.],
                             [0., 10., 0., 0., 0., 0.],
                             [0., 0., 10., 0., 0., 0.],
                             [0., 0., 0., 0.01, 0., 0.],
                             [0., 0., 0., 0., 0.01, 0.],
                             [0., 0., 0., 0., 0., 0.01]]

#. Setup process noise::

    sigmaPos = 0.01
    sigmaVel = 0.0001
    flybyOD.processNoise = [[sigmaPos, 0., 0., 0., 0., 0.],
                      [0., sigmaPos, 0., 0., 0., 0.],
                      [0., 0., sigmaPos, 0., 0., 0.],
                      [0., 0., 0., sigmaVel, 0., 0.],
                      [0., 0., 0., 0., sigmaVel, 0.],
                      [0., 0., 0., 0., 0., sigmaVel]]

#. Subscribe to the messages, primarily the measurement message::

    flybyOD.opNavHeadingMsg.subscribeTo(cobConverter.opnavUnitVecOutMsg)
    
