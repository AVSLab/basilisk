.. _usageMetrics:

Usage Metrics
=============

Basilisk collects daily PyPI and GitHub usage metrics with the
``Collect Usage Metrics`` GitHub Actions workflow. The workflow writes durable
history to the orphan ``usage-metrics`` branch so routine snapshots do not add
commits to ``develop`` or trigger the normal build workflow.

Usage Plot
----------

The metrics workflow regenerates the following plot after every successful
collection. The documentation references the image on the ``usage-metrics``
branch, so readers see the latest published data without waiting for another
documentation build. The plotted lines show seven-day trailing means; exact
daily values remain available in ``metrics.csv``.

.. image:: https://raw.githubusercontent.com/AVSLab/basilisk/usage-metrics/usage.svg
   :alt: Daily Basilisk PyPI download and GitHub clone activity
   :width: 100%
   :target: https://github.com/AVSLab/basilisk/tree/usage-metrics

Collected Metrics
-----------------

The branch contains the following generated files:

``metrics.csv``
    One row per UTC date. It contains PyPI download events, downloads made by
    ``pip``, GitHub clone events and daily unique cloners, and snapshots of the
    current fork and release-asset download counts.

``summary.json``
    Machine-readable current totals, source coverage dates, and the latest
    GitHub clone window.

``README.md``
    A human-readable summary and the interpretation caveats for each metric.

``usage.svg``
    A light- and dark-theme plot of the seven-day trailing means for PyPI
    downloads and GitHub clone activity.

PyPI history is read from the public PyPI data replicated by
`ClickPy <https://clickpy.clickhouse.com/>`__. The non-mirror count excludes
``bandersnatch``, ``z3c.pypimirror``, ``Artifactory``, and ``devpi``, matching
the `PyPI Stats known-mirror list <https://pypistats.org/faqs>`__. The separate
``pip`` count selects records whose installer is identified as ``pip``.

GitHub Configuration
--------------------

GitHub exposes clone traffic only to repository collaborators and retains only
the latest 14 days. The workflow reads the existing ``BOT_ACCESS_TOKEN``
Actions secret. That token must be either:

- a fine-grained token with read-only repository Administration permission; or
- a classic token with sufficient access to read the repository traffic API.

See the `GitHub repository traffic API
<https://docs.github.com/en/rest/metrics/traffic>`__ for the current permission
requirements. The token is used only for API reads. The workflow's scoped
``GITHUB_TOKEN`` publishes the generated files, so repository Actions settings
must permit workflows to write repository contents.

After the workflow reaches the default branch, run it once manually to seed the
``usage-metrics`` branch. The daily schedule then merges GitHub's overlapping
14-day windows, which preserves clone counts beyond GitHub's retention period.

Retrieving the Data
-------------------

The ``usage-metrics`` branch becomes available after the first successful
workflow run. Because the Basilisk repository is public, anyone can read the
generated data; no GitHub token or repository membership is required.

The branch and its rendered summary can be viewed at
`github.com/AVSLab/basilisk/tree/usage-metrics
<https://github.com/AVSLab/basilisk/tree/usage-metrics>`__. The individual data
files can be downloaded directly:

.. code-block:: bash

   curl --fail --location --remote-name \
     https://raw.githubusercontent.com/AVSLab/basilisk/usage-metrics/metrics.csv
   curl --fail --location --remote-name \
     https://raw.githubusercontent.com/AVSLab/basilisk/usage-metrics/summary.json
   curl --fail --location --remote-name \
     https://raw.githubusercontent.com/AVSLab/basilisk/usage-metrics/usage.svg

To inspect or copy the files without switching the current checkout away from
its source branch, fetch the metrics branch as a remote-tracking branch:

.. code-block:: bash

   git fetch origin usage-metrics:refs/remotes/origin/usage-metrics
   git show origin/usage-metrics:metrics.csv
   git show origin/usage-metrics:summary.json

To obtain the generated files together with the complete daily commit history,
clone only the orphan branch into a separate directory:

.. code-block:: bash

   git clone --branch usage-metrics --single-branch \
     https://github.com/AVSLab/basilisk.git basilisk-usage-metrics

Interpretation Limits
---------------------

PyPI supplies file-download events, not successful installations or unique
users. Repeated installs, continuous integration, caches, mirrors, and other
installers affect the counts.

GitHub's window-level unique-cloner count must not be added across windows or
dates because the same user can occur more than once. The fork count represents
currently existing forks, not every fork ever created. Release-asset downloads
include only files explicitly uploaded to a release; GitHub does not publish a
download count for automatically generated source ZIP and tar archives.
