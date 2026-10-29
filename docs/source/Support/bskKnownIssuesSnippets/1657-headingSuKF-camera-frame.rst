- :ref:`headingSuKF` applied the inverse camera mounting rotation when writing
  ``r_BN_C``, producing incorrect camera-frame headings for rotated cameras.
  It now uses the body-to-camera rotation defined by ``sigma_CB``.
