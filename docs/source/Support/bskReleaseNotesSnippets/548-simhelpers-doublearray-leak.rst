- Fixed a memory leak in :ref:`simHelpers` ``timeStringToGregorianUTCMsg``. The SWIG-allocated scratch
  ``doubleArray`` used to receive the ``str2et_c`` result was never released (about 32 bytes leaked per
  call); it is now freed with ``delete_doubleArray`` once the value has been read (issue #548).
