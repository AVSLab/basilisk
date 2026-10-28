- :ref:`opNavPoint` treated a retained ``valid=1`` message as a new image on
  every update, preventing timeout and reinterpreting its camera vector with
  the current attitude. The module now accepts each valid message write time
  once and reuses the saved inertial heading until it expires.
