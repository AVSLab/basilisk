
This module implements a opNav point attitude guidance routine.

This algorithm is intended to be incredibly simple and robust: it finds the angle error between the camera boresight (or desired control axis in the camera frame) and the planet heading in the camera frame and brings them to zero. This is analoguous to sunSafePoint.  The file
:download:`PDF Description </../../src/fswAlgorithms/attGuidance/opNavPoint/_Documentation/Basilisk-opNavPoint-20190820.pdf>`.
contains further information on this module's function, how to run it, as well as testing.

