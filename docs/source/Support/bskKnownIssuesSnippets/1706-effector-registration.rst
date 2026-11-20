- In Basilisk ``2.12.2``, a scheduled ``ReactionWheelStateEffector`` or ``VSCMGStateEffector``
  can terminate the simulation process if its spacecraft has not registered its device states (#1706).
  Attach the effector to the spacecraft and add the spacecraft to a simulation task before initialization.
  A configured effector now raises ``BasiliskError`` if its states are missing on update.
