- In Basilisk 2.12.1, importing the support-data loader checks GitHub for a release
  tag even when local or cached files are available, adding network-dependent
  startup delays. The lookup is now deferred until a repository-hosted file
  needs downloading. Local files, valid cached files, and explicit external
  download URLs avoid the lookup; release-tag selection and the fallback to
  ``develop`` are preserved when a download is required.
