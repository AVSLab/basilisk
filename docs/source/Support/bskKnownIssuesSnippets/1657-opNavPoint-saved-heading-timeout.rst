- :ref:`opNavPoint` used ``lastTime == 0`` to mark that no heading was saved.
  The heading of a valid image at simulation time 0 was therefore discarded on
  the next update, and the module switched to the search rate if the following
  image was invalid. A valid image that arrived exactly ``timeOut`` seconds
  after the previous one was also ignored for one update, and with the
  default ``timeOut`` of zero, valid images were never used.
