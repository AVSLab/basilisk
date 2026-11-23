- In Basilisk 0.7.2 through 2.12.2, :ref:`pixelLineConverter` adds a half-pixel offset on each image axis with an odd pixel count (#1717).
  On affected releases, subtract 0.5 pixels from the measured circle coordinate sent to the converter on each odd axis. Remove this correction after upgrading to the fixed version.
