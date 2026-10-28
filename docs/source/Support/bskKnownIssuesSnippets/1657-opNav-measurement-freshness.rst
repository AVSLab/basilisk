- :ref:`pixelLineConverter`, :ref:`horizonOpNav`, and :ref:`faultDetection`
  republished retained valid measurements with new output timestamps, which could
  prevent downstream :ref:`opNavPoint` timeout. They now consume each input write
  once, preserve the output and its timestamp between writes, and publish invalid
  output when new inputs provide no usable measurement. This allows slower
  consumers to read valid observations without continually refreshing their age.
  Fault detection checks freshness independently for its primary and secondary
  inputs; mode 2 requires both to be fresh in the same update.
