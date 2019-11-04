
This module filters incoming star tracker measurements and reaction wheel data in order to get the best possible inertial attitude estimate. The filter used is an unscented Kalman filter using the Modified Rodrigues Parameters (MRPs) as a non-singular attitude measure.  Measurements can be coming in from several camera heads.

More information on can be found in the
:download:`PDF Description </../../src/fswAlgorithms/attDetermination/InertialUKF/_Documentation/Basilisk-inertialUKF-20190402.pdf>`

