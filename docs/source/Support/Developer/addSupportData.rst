.. _addSupportData:

Adding Support Data Files
=========================

This page explains how to correctly add new files to the ``supportData``
directory so they integrate with Basilisk's Pooch-based data management system
and are versioned on GitHub.

Basilisk **does not ship** supportData files inside the wheel.
Instead, all files are registered and fetched on demand.

When you add or update support data files, three major parts may need updating:

1. ``supportData/`` directory (the files themselves)
2. ``makeRegistry.py`` (to regenerate the MD5 registry)
3. ``dataFetcher.py`` (to map the file into enums/category paths)


----------------------------------------------------------------------
Folder Layout
----------------------------------------------------------------------

Files must live inside:

::

    basilisk/supportData/<CategoryName>/<file>

Where ``<CategoryName>`` must match one of the following:

- ``AlbedoData``
- ``AtmosphereData``
- ``DentonGEO``
- ``EphemerisData``
- ``LocalGravData``
- ``MagneticField``

If you add a new sub-category, **you must also add it to**
``dataFetcher.py`` (explained in Step 4).

----------------------------------------------------------------------
Step 1 — Add the File to supportData/
----------------------------------------------------------------------

Place your file under the appropriate category, e.g.:

::

    basilisk/supportData/AtmosphereData/NewMarsAtmosphere2025.csv

Make sure the file is not covered by any ignore patterns
(see ``makeRegistry.py``).

----------------------------------------------------------------------
Step 2 — Ensure makeRegistry.py will include the file
----------------------------------------------------------------------

The registry generator ignores certain patterns:

.. code-block:: python

    IGNORE_PATTERNS = (
        "__pycache__",
        ".pyc",
        "__init__.py",
        "*.bsp",
    )

If your file accidentally matches a pattern, remove or update the entry.

----------------------------------------------------------------------
Step 3 — Regenerate the registrySnippet.py file
----------------------------------------------------------------------

From the **project root**, run:

::

    python src/utilities/supportDataTools/makeRegistry.py > src/utilities/supportDataTools/registrySnippet.py

This writes a dictionary of:

- file to MD5 hash
- symbols used by Pooch to verify downloads

Commit the changed ``registrySnippet.py``.

----------------------------------------------------------------------
Step 4 — Update dataFetcher.py (Enums and Base Paths)
----------------------------------------------------------------------

In ``dataFetcher.py``, each category has:

1. An Enum listing files
2. A base path value
3. A mapping in ``CATEGORY_BASE_PATHS``

For example, adding ``NewMarsAtmosphere2025.csv`` requires:

1. Add to the correct Enum:

.. code-block:: python

    class AtmosphereData(Enum):
        NewMarsAtmosphere2025 = "NewMarsAtmosphere2025.csv"

2. Ensure the base path exists:

.. code-block:: python

    ATMOSPHERE_DATA_BASE_PATH = "supportData/AtmosphereData/"

3. Ensure the category name → base path map includes it:

.. code-block:: python

    CATEGORY_BASE_PATHS = {
        "AtmosphereData": ATMOSPHERE_DATA_BASE_PATH,
        ...
    }


----------------------------------------------------------------------
External Data Sources
----------------------------------------------------------------------

Some kernel files are **not in the Git repo** and **should not be hashed**:

.. code-block:: python

    EXTERNAL_KERNEL_URLS = {
        "supportData/EphemerisData/de430.bsp": "https://naif.jpl.nasa.gov/...",
        ...
    }

These entries automatically override registry hashes:

.. code-block:: python

    for key in EXTERNAL_KERNEL_URLS:
        REGISTRY[key] = None

This prevents MD5 failures when NAIF updates files.

**Note:**
Even for external files, an entry **must still exist in ``dataFetcher.py``**
(Enums + category base path). This allows Pooch to download the file on demand
and return a local path via ``get_path()``.
