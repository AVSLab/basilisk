
This module filters position measurements that have been processed from planet images in order to estimate spacecraft relative position to an observed body in the inertial frame. The filter used is an unscented Kalman filter, and the images are first processed by houghCricles and pixelLineConverter in order to produce this filter's measurements.

The module
:download:`PDF Description </../../src/fswAlgorithms/opticalNavigation/relativeODuKF/_Documentation/Basilisk-relativeOD-20190620.pdf>`
contains further information on this module's function,
how to run it, as well as testing.
