.. _numbaModules:

Making Numba Modules
====================

.. sidebar:: Source Code

    The Python code shown below can be downloaded :download:`here </../../docs/source/codeSamples/making-numbaModules.py>`.

``NumbaModel`` is a Basilisk module type that achieves near-C execution speed
while being written entirely in Python.  It is a good fit for numerically
intensive inner loops (attitude control laws, filter update steps, trajectory
integrators) that are too slow as plain Python modules but do not justify a
full C++ module.

Under the hood, ``NumbaModel`` uses `Numba <https://numba.readthedocs.io>`_
to JIT-compile the user-supplied ``UpdateStateImpl`` static method into a
``cfunc`` (a C-callable function pointer) that the Basilisk C++ scheduler
calls directly every tick.  The compilation happens once, at ``Reset`` time.

.. note::

   Because ``UpdateStateImpl`` is compiled by Numba in *nopython* mode, a
   subset of Python and NumPy is available inside it.  Read the
   :ref:`numbaModules_constraints` section carefully before writing complex
   logic.


Defining a Numba Module
-----------------------

Subclass ``NumbaModel`` from ``Basilisk.architecture.numbaModel`` and
implement ``UpdateStateImpl`` as a ``@staticmethod``.  All message attributes
and memory fields must be created in ``__init__``; the parameter names of
``UpdateStateImpl`` tell the framework which of those attributes to wire up.

.. literalinclude:: ../../codeSamples/making-numbaModules.py
   :language: python
   :linenos:
   :lines: 19-


Parameter Naming Convention
----------------------------

Unlike regular Python or C++ modules, ``NumbaModel`` does **not** require you
to write a ``Reset`` or ``UpdateState`` method.  Instead, the framework
introspects the parameter names of ``UpdateStateImpl`` at ``Reset`` time and
automatically wires each one to the correct buffer.

You **may** override ``Reset`` (e.g. to perform additional Python-level
setup), but you must call ``super().Reset(CurrentSimNanos)`` so that the
cfunc is compiled and registered::

    def Reset(self, CurrentSimNanos=0):
        super().Reset(CurrentSimNanos)   # must be called
        # ... additional setup ...

Do **not** override ``UpdateState``.  The C++ scheduler calls the
JIT-compiled cfunc registered by ``NumbaModel.Reset`` directly; a Python
``UpdateState`` override would intercept that call and bypass the compiled
function entirely.

The recognised ``UpdateStateImpl`` parameter names are:

.. list-table::
   :widths: 35 15 50
   :header-rows: 1

   * - Parameter name pattern
     - Type inside ``UpdateStateImpl``
     - What it provides
   * - ``<name>OutMsgPayload``
     - Numba Record (struct-like)
     - Writable payload of ``self.<name>OutMsg``
   * - ``<name>InMsgPayload``
     - Numba Record or array of Records
     - Read-only snapshot of ``self.<name>InMsg``
   * - ``<name>InMsgIsLinked``
     - ``bool`` (scalar reader), ``uint8[:]`` (list), or Record (dict)
     - ``True``/non-zero when the corresponding reader is subscribed
   * - ``CurrentSimNanos``
     - ``uint64``
     - Current simulation time in nanoseconds
   * - ``moduleID``
     - ``int64``
     - This module's unique integer ID
   * - ``memory``
     - Numba Record
     - Persistent state fields (see :ref:`numbaModules_memory`)
   * - ``bskLogger``
     - ``BskLoggerProxy``
     - Logging proxy (see :ref:`numbaModules_logging`)
   * - ``rng``
     - ``numpy.random.Generator``
     - Per-module RNG seeded from ``self.RNGSeed`` at Reset (see :ref:`numbaModules_rng`)

Any parameter name not matching one of the patterns above raises an
``AttributeError`` at ``Reset`` time with a descriptive message.

.. note::

    Parameter order inside ``UpdateStateImpl`` does **not** matter; the
    framework matches by name, not position.


Output Messages
~~~~~~~~~~~~~~~

Declare output messages as ``messaging.<Type>Msg()`` attributes in ``__init__``::

    self.dataOutMsg = messaging.CModuleTemplateMsg()

Use the suffix ``OutMsgPayload`` in ``UpdateStateImpl`` to receive a writable
view of the payload::

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, ...):
        dataOutMsgPayload.dataVector[0] = 42.0

The Basilisk header timestamp is written automatically after ``UpdateStateImpl``
returns.  You do not need to call ``.write()``.

For **multiple** output messages, assign a list or dict to the attribute::

    self.dataOutMsg = [messaging.CModuleTemplateMsg(),
                       messaging.CModuleTemplateMsg()]

Inside ``UpdateStateImpl``, access them by integer index::

    dataOutMsgPayload[0].dataVector[0] = v
    dataOutMsgPayload[1].dataVector[0] = v * 2.0

Or by string key when using a dict::

    self.dataOutMsg = {'pos': messaging.CModuleTemplateMsg(),
                       'neg': messaging.CModuleTemplateMsg()}

    # inside UpdateStateImpl:
    dataOutMsgPayload['pos'].dataVector[0] =  v
    dataOutMsgPayload['neg'].dataVector[0] = -v


Input Messages (Readers)
~~~~~~~~~~~~~~~~~~~~~~~~

Declare readers as ``messaging.<Type>MsgReader()`` attributes::

    self.dataInMsg = messaging.CModuleTemplateMsgReader()

Subscribe before ``InitializeSimulation``::

    mod.dataInMsg.subscribeTo(src.dataOutMsg)

The payload is refreshed from the source every tick by the C++ scheduler.
Access it in ``UpdateStateImpl`` via the ``InMsgPayload`` parameter.

If a reader **may** be unlinked, declare the corresponding ``IsLinked``
parameter and guard reads behind it::

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked, dataOutMsgPayload):
        if dataInMsgIsLinked:
            dataOutMsgPayload.dataVector[0] = dataInMsgPayload.dataVector[0]

If ``<name>InMsgPayload`` is declared *without* a corresponding
``<name>InMsgIsLinked``, ``Reset`` raises ``RuntimeError`` when the reader
is not subscribed. This is an intentional early failure rather than a silent
wrong-data scenario.

List and dict attributes work the same way as for output messages.  For a
**list** reader, ``IsLinked`` is a ``uint8[:]`` array indexed by position
(``dataInMsgIsLinked[0]``, ``dataInMsgIsLinked[1]``, …).  For a **dict**
reader, ``IsLinked`` is a Record keyed by the same names as the payload, so
both can be accessed identically::

    if dataInMsgIsLinked['a']:
        val = dataInMsgPayload['a'].dataVector[0]


.. _numbaModules_memory:

Persistent Memory
~~~~~~~~~~~~~~~~~

``self.memory`` is a namespace that holds persistent module state across ticks.
Fields are defined as attributes in ``__init__``::

    self.memory.step   = 0                 # scalar integer (→ int64)
    self.memory.gain   = 1.5              # scalar float   (→ float64)
    self.memory.bias   = [0.1, 0.2, 0.3]  # 1-D array (converted to numpy)
    self.memory.matrix = np.eye(3)         # 2-D array

At ``Reset`` time the fields are packed into a single structured numpy array
whose pointer is handed to the C++ scheduler.  After ``Reset``, the fields are
accessible from Python via the same ``self.memory.<name>`` syntax and any
assignment is forwarded directly to the underlying C buffer - no re-compilation
is needed.

Rules:

- All fields must be defined **before** ``Reset`` is called (i.e., in
  ``__init__`` or before ``InitializeSimulation``).
- New fields cannot be added after ``Reset``.
- The numpy dtype of each field is inferred from the value assigned in
  ``__init__``: a plain ``0`` becomes ``int64``, a plain ``0.0`` becomes
  ``float64``.  Use an explicit numpy type (e.g. ``np.int32(0)``) only when
  a specific width is required.
- Inside ``UpdateStateImpl``, ``memory`` behaves like a Numba ``Record``.
  Array fields are accessed with standard bracket indexing:
  ``memory.bias[1]``, ``memory.matrix[i, j]``.


.. _numbaModules_logging:

Logging with ``bskLogger``
~~~~~~~~~~~~~~~~~~~~~~~~~~

Add ``bskLogger`` as a parameter to receive a Numba-compatible logging proxy::

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, bskLogger, memory):
        bskLogger.bskLog(bskLogging.BSK_INFORMATION, "tick")
        bskLogger.bskLog1(bskLogging.BSK_WARNING, "step:", memory.step)

The proxy respects the log level set on ``self.bskLogger`` at ``Reset`` time:
messages below the current threshold are suppressed.  Output goes to stdout via
``print``; ANSI colour codes match the rest of Basilisk's log output.

Available methods:

.. list-table::
   :widths: 50 50
   :header-rows: 1

   * - Method
     - Prints
   * - ``bskLog(level, msg)``
     - ``[TAG] msg``
   * - ``bskLog1(level, msg, v0)``
     - ``[TAG] msg v0``
   * - ``bskLog2(level, msg, v0, v1)``
     - ``[TAG] msg v0 v1``
   * - ``bskLog3(level, msg, v0, v1, v2)``
     - ``[TAG] msg v0 v1 v2``

Level constants (``bskLogging.BSK_DEBUG``, ``BSK_INFORMATION``,
``BSK_WARNING``, ``BSK_ERROR``) work inside ``UpdateStateImpl`` because Numba
resolves module-level integer attributes at compile time.

``msg`` must be a **string literal** - variables of string type are not
supported in nopython mode.  Values ``v0``–``v2`` can be any numeric type
that Numba supports (``int32``, ``float64``, etc.).

.. tip::

   Plain ``print("label:", value)`` also works in ``UpdateStateImpl`` with no
   additional setup.


.. _numbaModules_rng:

Per-module Random Number Generator (``rng``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add ``rng`` as a parameter to receive a ``numpy.random.Generator`` that is
private to this module instance and seeded from ``self.RNGSeed`` at ``Reset``
time::

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, rng, memory):
        noise = rng.standardNormal(3)      # independent per-module stream
        dataOutMsgPayload.dataVector[:3] = memory.signal + noise

The generator is created with ``numpy.random.default_rng(self.RNGSeed)`` and
its state persists across ticks, so each call advances the sequence
deterministically.  ``self.RNGSeed`` is inherited from :ref:`SysModel
<sysModel>` and defaults to ``0x1badcad1``; set it before
``InitializeSimulation()`` for reproducible Monte Carlo runs::

    sNav.RNGSeed = 12345

Key properties:

- **Per-module isolation**: each instance owns an independent generator - two
  modules with the same seed produce the same sequence independently, and two
  modules with different seeds never interfere.
- **Re-seeded on Reset**: calling ``Reset`` again creates a fresh generator
  from the current ``self.RNGSeed``, resetting the sequence.
- **Requires scipy** for BLAS-backed numpy linalg inside Numba nopython mode.
  A clear ``ImportError`` is raised at import time if scipy is missing.

All ``numpy.random.Generator`` methods supported by Numba nopython mode are
available: ``standardNormal``, ``random``, ``integers``, ``uniform``, etc.


.. _numbaModules_constraints:

What is Allowed in ``UpdateStateImpl``
---------------------------------------

``UpdateStateImpl`` is compiled by Numba in **nopython mode**.  The following
is a summary of what is and is not supported.

Allowed
~~~~~~~

- **NumPy array operations** on pre-allocated arrays (element-wise arithmetic,
  ``[i]`` / ``[i, j]`` indexing, ``array.copy()``, ``np.zeros(n)``,
  ``np.dot``, reductions, etc.).  See the
  `Numba NumPy support list <https://numba.readthedocs.io/en/stable/reference/numpysupported.html>`_
  for the full set.
- **Explicit numpy scalars** such as ``np.int32(1)``, ``np.float64(3.14)``.
  Use these when a specific width is required (e.g. to match a memory field
  declared as ``np.int32``); otherwise plain literals are fine.
- **Control flow**: ``if``/``elif``/``else``, ``for`` over ``range(n)``,
  ``while``, ``break``, ``continue``, ``return``.
- **Math functions**: ``math.sqrt``, ``math.sin``, etc. (import ``math``
  at module level, use inside the function normally).
- **Calling other** ``@nb.njit`` **functions** - factor complex logic into
  helper functions decorated with ``@numba.njit``.
- **``print(label, value, ...)``** - variadic print to stdout works and is
  the simplest way to emit debug output.
- **``bskLogger``** methods as described in :ref:`numbaModules_logging`.
- **Reading from** ``memory`` **Record fields** and **writing to them**.
- **Reading from** ``InMsgPayload`` **Record fields** and **writing to**
  ``OutMsgPayload`` **Record fields**.

Not Allowed
~~~~~~~~~~~

- **Python objects** of any kind: no lists, dicts, sets, tuples, classes,
  generators, or iterators.  Use numpy arrays for all collections.
- **String variables** - string *literals* in ``print`` and ``bskLogger`` calls
  work, but you cannot assign a string to a variable or pass one as a
  function argument (other than via the ``bskLog*`` API).
- **f-strings and %-formatting** - these require Python object creation.
  Use ``bskLog1``/``bskLog2`` to attach numeric values to log messages.
- **Exceptions** - ``raise``, ``try``/``except``/``finally`` are not
  supported.  Use conditional logic and ``bskLogger`` to handle error
  conditions.
- **Calling Python functions** that are not themselves ``@nb.njit``-compiled.
  Standard library functions (``os``, ``sys``, etc.) are unavailable.
- **Dynamic memory allocation** with arbitrary Python types.  NumPy array
  creation with a *constant* size (``np.zeros(3)``) is fine; allocation
  whose size depends on a runtime variable may require care.
- **Global mutable state** - do not capture mutable Python objects in the
  closure of ``UpdateStateImpl``.  Use ``memory`` for persistent state.
- **Messaging API** - do not call ``.write()``, ``.read()``, or
  ``.subscribeTo()`` inside ``UpdateStateImpl``.  Payload structs are
  passed as already-refreshed Records; writing back through them is
  sufficient.


Re-use and Re-initialisation
-----------------------------

**Re-wiring messages mid-simulation** (calling ``subscribeTo`` without a
``Reset``) is fully supported.  The C++ scheduler refreshes the payload
pointer dynamically every tick via registered reader slots.

**Calling ``Reset`` a second time** re-compiles the cfunc and re-initialises
``memory`` from the current buffer state.  The buffer is always the source of
truth: Python-side writes (``self.memory.gain = 2.0``) and cfunc-internal
writes (``memory.step += 1`` inside ``UpdateStateImpl``) are treated
identically; both persist through a subsequent Reset.  The next run always
continues from whatever value was last written, by any means.


Performance Notes
-----------------

- Compilation happens once at ``Reset`` time and is cached to disk by Numba
  (keyed on the bytecode of ``UpdateStateImpl``).  Subsequent runs with the
  same code load the cached binary instantly.
- The cfunc is called from C++ with zero Python overhead per tick.  Execution
  speed is comparable to a hand-written C extension for the same algorithm.
- Keep ``UpdateStateImpl`` free of Python-level function calls; every call
  that Numba cannot inline adds compilation complexity and may defeat
  caching.
