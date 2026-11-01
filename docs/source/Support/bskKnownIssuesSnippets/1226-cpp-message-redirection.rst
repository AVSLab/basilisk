- GitHub issue 1226: Fixed C++ output redirection and message lifetimes,
  restoring shared OpNav outputs affected since 2.12.0. Message-vector access
  now preserves the correct source owner across later module imports, preventing
  premature deletion and false deleted-module errors.
- Configure redirection before connecting readers or recorders, and keep modules
  alive during simulation. Use stand-alone gateways for redirected feedback;
  native ownership cycles remain unsupported and are not automatically detected.
  Redirecting through raw pointer-vector copies or slices raises
  ``ReferenceError``; use the original module output or a Python list selection.
  Keep the module's output-vector entries intact.
- Camera and Vizard image buffers are valid only until the producer replaces
  them. Message recorders do not archive image bytes; save images or copy their
  bytes while current.
- Fixed Python iterator crashes and numeric-vector access errors that depended
  on module import order, including storage-message reads and :ref:`linkBudget`
  lookup tables.
