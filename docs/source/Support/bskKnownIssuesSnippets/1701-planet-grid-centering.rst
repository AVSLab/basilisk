- In Basilisk 2.11.x and 2.12.x, :ref:`albedo` and :ref:`earthRadiationModel` shifted grids with an odd number of rows
  half a cell north, omitting polar area and underestimating infrared flux over the poles. An odd number of columns
  shifted map values half a cell east. Earlier :ref:`albedo` releases, including 1.8.1 and 2.10.2, left the last latitude
  or longitude coordinate at zero when its count was odd. On affected releases, use even latitude and longitude counts
  and resample odd-sized albedo maps to an even number of rows and columns (GitHub issue 1701).
