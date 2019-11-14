
This module filters position measurements that have been processed from planet images in order to estimate spacecraft relative position to an observed body in the inertial frame. It is similar to the relativeOD filter except that it estimates measurement bias in pixels and therefore integrates the pixel and line transformation in the measurement model. This means this module reads in circle data directly.

The module
:download:`PDF Description </../../src/fswAlgorithms/opticalNavigation/pixelLineBiasUKF/_Documentation/Basilisk-PixelLineBiasUKF-20190620.pdf>`
contains further information on this module's function,
how to run it, as well as testing.
