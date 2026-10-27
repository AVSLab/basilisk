- GitHub issue 1657: :ref:`opNavPoint` wrote a principal rotation axis in
  camera frame components into the body-relative attitude error ``sigma_BR``,
  including the 180 degree fallback axis. The commanded rotation was correct
  only when the camera frame coincided with the body frame
  (``sigma_CB = [0, 0, 0]``); any other camera mounting rotated the spacecraft
  about the wrong axis.
