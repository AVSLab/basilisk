
.. _bskPrinciples-6a:

Importing External Data Sources
===============================

Basilisk provides a range of simulation support data.  You can see the Basilisk
provided data sets in :ref:`supportDataList`.  The data is organized into topical folders
which contain the data files.

The data importing is handled with :ref:`dataFetcher`.  This process pulls the data
file from the local ``basilisk/supportData`` folder if it exists (repo was cloned),
or from the ``pooch`` cache if it was downloaded by ``pooch`` data management package.

Importing the data fetching tools and using local or valid cached files do not
contact GitHub. When a file must be downloaded from Basilisk's GitHub repository,
the downloader checks whether the installed release tag exists and falls back to
``develop`` if it is unavailable. The tag result is cached for subsequent
downloads in the same Python process. Files with explicit external URLs, such
as NASA ephemeris files, do not need this tag check.

At the top of your simulation script, import the data fetching tools using::

    from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

Next, if you want to import a file ``GGM2BData`` inside the folder ``LocalGravData``, then
use::

    ggm2b_path = get_path(DataFile.LocalGravData.GGM2BData)

Note that the file extension is not needed, only the file name.
To set the data file path as a string to a Basilisk module, you can use::

    planet.useSphericalHarmonicsGravityModel(str(ggm2b_path), 100)
