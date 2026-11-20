- In Basilisk ``2.12.2``, updating ``SimpleTransmitter`` or ``SpaceToGroundTransmitter``
  without an attached storage unit can terminate the simulation process (#1705).
  On affected versions, attach a storage status message with ``addStorageUnitToTransmitter()``
  before the first update. Both transmitters now publish zero baud while storage input is unavailable.
