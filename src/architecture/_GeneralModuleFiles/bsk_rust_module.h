/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

#ifndef BSK_RUST_MODULE_H
#define BSK_RUST_MODULE_H

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"

/*! @brief Declares the three BSK lifecycle entry points for a C module whose
 *  implementation is written in Rust.
 *
 *  **Background**
 *
 *  Basilisk C modules consist of two parts:
 *
 *  1. A plain-C *config struct* that holds all parameters, message ports,
 *     and optional persistent state.  It is parsed by SWIG to produce the
 *     Python-accessible wrapper class.
 *  2. Three *lifecycle functions* — ``SelfInit``, ``Reset``, ``Update`` —
 *     called by the Basilisk task scheduler at well-defined points.
 *
 *  For Rust modules the lifecycle functions are compiled into a Rust static
 *  library linked into the SWIG extension.  ``BSK_RUST_DECL`` emits the
 *  matching ``extern "C"`` *declarations* so that the C compiler and the
 *  SWIG-generated glue can find them.
 *
 *  **Shim-based workflow**
 *
 *  Writing raw ``unsafe extern "C"`` Rust is error-prone.  The recommended
 *  workflow uses ``build.rs`` (via the ``bsk-build`` crate) to auto-generate
 *  both the C header below and a *shim* from the Rust source: ``bsk-build``
 *  finds the config struct by its ``impl BskModule for <Type>`` block, reads
 *  its message-port fields, and emits the ``extern "C"`` entry points that
 *  read/write messages around the module's own ``update``. The user
 *  implements ``init``, ``reset``, and ``update`` in safe Rust with
 *  typed message arguments — no FFI boilerplate by hand. See the Basilisk
 *  documentation's "Writing a Rust Plugin" page for the full guide.
 *
 *  **Config struct field ordering**
 *
 *  For Rust modules specifically, the suggested layout below reads
 *  well and keeps the mandatory ``runtime`` field impossible to miss::
 *
 *      typedef struct {
 *          // 1. Runtime mirror — required; see "Runtime mirror" below.
 *          //    Not required to be first, but every other field reads it,
 *          //    so this SDK's examples put it first for readability.
 *          BskRustModuleRuntime runtime;         //!< [-]  moduleID/ModelTag/etc. mirror
 *
 *          // 2. Scalar / array parameters
 *          double K;                             //!< [Nm]       proportional gain
 *          double P;                             //!< [Nm/(rad/s)] rate gain
 *
 *          // 3. Heap-allocated Rust state (stateful modules only — see below)
 *          void *state;                          //!< [-]  Option<Box<T>> owned state
 *
 *          // 4. Input message ports
 *          AttGuidMsg_C attGuidInMsg;            //!< [-]  attitude guidance
 *
 *          // 5. Output message ports
 *          CmdTorqueBodyMsg_C cmdTorqueOutMsg;   //!< [Nm] control torque
 *
 *          // 6. BSK logger
 *          BSKLogger *bskLogger;                 //!< [-]  BSK logging handle
 *      } myModuleConfig;
 *
 *      BSK_RUST_DECL(myModule, myModuleConfig)
 *
 *  On the Rust side, ``attGuidInMsg``/``cmdTorqueOutMsg`` above are written
 *  as ``MsgReader<AttGuidMsg>``/``MsgWriter<CmdTorqueBodyMsg>`` — see the
 *  "Writing a Rust Plugin" documentation page for the Rust-side type.
 *
 *  **moduleID**
 *
 *  ``moduleID`` is a unique ``int64_t`` assigned by Basilisk's
 *  ``ModuleIdGenerator`` when the module is registered with a task.  It is
 *  stamped onto every outgoing message header (via ``*_C_write``) so that
 *  message recording and the logging subsystem can identify which module
 *  produced a given message.  The shim reads it from ``cfg->runtime.moduleID``
 *  and forwards it to every ``*_C_write`` call automatically.
 *
 *  **Runtime mirror — BskRustModuleRuntime**
 *
 *  A Rust module has no C++ base class, so this struct mirrors the relevant
 *  ``SysModel`` fields (module ID, name, ...) into the config's own
 *  ``runtime`` field before every ``SelfInit``/``Reset``/``Update`` call —
 *  Rust code reads it like any other config field.
 *
 *  ``modelTag`` is a borrowed pointer valid only for the duration of the
 *  call. On the Rust side this is enforced by the compiler, not just this
 *  comment: the Rust mirror of this struct isn't ``Copy``/``Clone``, and its
 *  string accessor returns a ``&str`` tied to that borrow, so retaining
 *  either past the lifecycle call that received it is a compile error.
 *
 *  **currentSimNanos**
 *
 *  ``currentSimNanos`` is the current simulation time in nanoseconds [ns],
 *  passed to both ``Reset`` and ``Update`` (not ``SelfInit``).  It is also
 *  written into each outgoing message header by ``*_C_write``.
 *  (BSK C modules call this parameter ``callTime``; Rust modules use the
 *  more explicit C++ ``SysModel`` name ``currentSimNanos``.)
 *
 *  **Logging**
 *
 *  A ``bskLogger: *mut BSKLogger`` field (see the field-ordering example
 *  above) gets the same standard logging a hand-written C module has
 *  through ``_bskLog(configData->bskLogger, BSK_WARNING, info)`` /
 *  ``_bskError(configData->bskLogger, info)`` — ``bsk-messages``'
 *  ``BskLoggerExt`` trait wraps those same two entry points as
 *  ``.debug()``/``.info()``/``.warning()``/``.bsk_error()`` methods on the
 *  field itself, e.g. ``self.bskLogger.warning("...")``. ``bsk-build``'s
 *  own generated "unconnected required input" check
 *  (see "Message port patterns" below) uses this identical path.
 *
 *  **Message port patterns**
 *
 *  Whether an input port is required or optional is determined from the
 *  ``impl BskModule`` block's ``type Inputs`` tuple.  The element type you
 *  already have to write for ``update`` to type-check is the single source
 *  of truth::
 *
 *      impl BskModule for myModuleConfig {
 *          type Inputs = (AttGuidMsg, Option<CmdTorqueBodyMsg>);
 *          //              ^^^^^^^^^  ^^^^^^^^^^^^^^^^^^^^^^^^^
 *          //              required   optional
 *          ...
 *      }
 *
 *  *Required input* — connectivity is checked in ``Reset`` and before each
 *  ``Update`` read; a missing connection raises the standard
 *  ``BasiliskError``.
 *
 *  *Optional input* — wrap the corresponding ``Inputs`` tuple element in
 *  ``Option<Msg>``; ``update`` then receives ``Option<Msg>`` (``None`` when
 *  unlinked) instead of an error.
 *
 *  *Output* — initialized automatically in ``SelfInit``.
 *
 *  **Python wiring** (same as any BSK C module)::
 *
 *      ctrl = myModule.myModule()                    # Python wrapper class
 *      ctrl.ModelTag = "myCtrl"
 *      ctrl.K = 5.0                                  # set parameters
 *      ctrl.attGuidInMsg.subscribeTo(src.attGuidOutMsg)   # connect input
 *      sim.AddModelToTask("task", ctrl)               # assigns moduleID
 *      # ctrl.cmdTorqueOutMsg is readable after InitializeSimulation()
 *
 *  **Stateful modules — owned heap state**
 *
 *  Modules that need persistent heap state between calls (e.g. integrators,
 *  filters) add an ``Option<Box<T>>`` field to the config struct — ordinary,
 *  safe Rust ownership, no manual pointer casts::
 *
 *      // In lib.rs:
 *      #[repr(C)]
 *      pub struct myModuleConfig {
 *          pub runtime: BskModuleRuntime,
 *          pub state: Option<Box<MyState>>,
 *          // ...
 *      }
 *
 *      impl BskModule for myModuleConfig {
 *          fn init(&mut self) {
 *              self.state = Some(Box::new(MyState::default()));
 *          }
 *          // update() accesses it via self.state.as_mut()...
 *      }
 *
 *  ``bsk-build`` maps ``Option<Box<T>>`` to a nullable ``void *`` in the
 *  generated header (Rust guarantees the same layout as a bare pointer) and
 *  gives the struct a C++ destructor that runs its ordinary Rust drop glue,
 *  freeing ``state`` automatically whenever the owning C++ wrapper object is
 *  destroyed (Python garbage collection, explicit ``del``, or process exit).
 *  No ``Cleanup_*`` function or custom SWIG destructor to write by hand.
 *
 *  ``Option<Box<T>>`` isn't chosen to guard against the Python API — it's
 *  what makes the automatic cleanup above possible at all (a bare pointer
 *  field has no drop glue for ``Drop_name`` to run). Python has no
 *  legitimate reason to read or write this field directly, so the CMake
 *  macro (``bsk_add_rust_module``) detects every ``Option<Box<T>>`` field
 *  and marks it ``%immutable`` in the generated SWIG interface — the
 *  setter is absent from the Python API, so a script can't null it out
 *  (silently leaking the boxed value) or alias it to an unrelated pointer
 *  (which would make ``Drop_name`` free memory Rust never allocated).
 *
 *  **Grouping parameters — nested structs**
 *
 *  A field may be another ``#[repr(C)]`` struct defined in the same crate,
 *  by value (not a pointer), to group related parameters::
 *
 *      #[repr(C)]
 *      pub struct Vec2 { pub x: f64, pub y: f64 }
 *
 *      #[repr(C)]
 *      pub struct myModuleConfig {
 *          pub runtime: BskModuleRuntime,
 *          pub target: Vec2,
 *          // ...
 *      }
 *
 *  ``bsk-build`` generates ``Vec2``'s C struct alongside ``myModuleConfig``
 *  and Python reads/writes it field-by-field like any built-in BSK module
 *  (``ctrl.target.x = 1.0``) — no special handling needed.
 *
 *  A raw pointer to one of these structs (``*mut Vec2``), or to any other
 *  type, is rejected — ``BSKLogger`` is the only pointee a field may name.
 *  SWIG's pointer-field setter would transfer ownership away from the
 *  Python object with nothing on the Rust side to ever free it; this
 *  applies just as much to a pointer to a primitive (``*mut u8``) as to a
 *  struct, so there is currently no supported field type for a persistent
 *  string or byte-buffer parameter.
 *
 *  A field also may not be a Rust ``enum``, even a fieldless ``#[repr(u8)]``
 *  (or similar) one — SWIG's setter for an enum-typed field accepts any
 *  integer, not just ones matching a declared variant, so a match on the
 *  field's value could hit undefined behavior once Python is free to write
 *  an out-of-range value. Use the underlying integer type as the field
 *  (e.g. ``pub mode: u8``) and convert it with a checked conversion inside
 *  ``update``/``reset`` instead.
 */

#ifdef __cplusplus
#   define BSK_RUST_EXTERN_C_BEGIN  extern "C" {
#   define BSK_RUST_EXTERN_C_END    }
#else
#   define BSK_RUST_EXTERN_C_BEGIN
#   define BSK_RUST_EXTERN_C_END
#endif

/*! @brief Snapshot of the ``SysModel`` runtime fields a Rust module may need.
 *
 *  Passed by pointer to every lifecycle call and copied into the config
 *  struct's own ``runtime`` field (see "Runtime mirror" above); valid for
 *  the duration of the call only — do not retain it beyond that
 *  (``modelTag`` in particular is a borrowed pointer).
 */
typedef struct BskRustModuleRuntime {
    int64_t moduleID;         /*!< [-] unique ID assigned by ModuleIdGenerator */
    const char *modelTag;     /*!< [-] SysModel::ModelTag, borrowed for this call only */
    uint64_t callCounts;      /*!< [-] SysModel::CallCounts step counter */
    uint32_t rngSeed;         /*!< [-] SysModel::RNGSeed */
} BskRustModuleRuntime;

/*! Emit ``extern "C"`` lifecycle function declarations for a Rust-backed
 *  Basilisk C module named \p name, whose config struct type is \p configType.
 *
 *  ``SelfInit_name(cfg, runtime)``
 *      Called once at task registration.  The shim copies ``*runtime`` into
 *      ``cfg->runtime`` and initialises output message ports (``*_C_init``).
 *
 *  ``Reset_name(cfg, currentSimNanos, runtime)``
 *      Called before the first step and on explicit resets.  The shim copies
 *      ``*runtime`` into ``cfg->runtime``, checks required input
 *      connectivity, then calls ``BskModule::reset``.
 *
 *  ``Update_name(cfg, currentSimNanos, runtime)``
 *      Called every simulation step.  The shim copies ``*runtime`` into
 *      ``cfg->runtime``, reads all input messages, calls
 *      ``BskModule::update``, and writes all output messages.
 *
 *  ``Drop_name(cfg)``
 *      Runs ``cfg``'s ordinary Rust drop glue (freeing any owned heap state,
 *      see "Stateful modules" above) without deallocating ``cfg`` itself.
 *      Called automatically from the generated header's inline
 *      ``configType`` destructor — never call it directly.
 *
 *  \p configType must be declared before this macro and must have a field
 *  named ``runtime`` of type ``BskRustModuleRuntime`` somewhere in it (any
 *  position — see "Config struct field ordering" above). bsk-build passes
 *  whatever struct name the crate's ``impl BskModule`` block actually uses,
 *  which need not match ``name##Config``.
 */
#define BSK_RUST_DECL(name, configType) \
    BSK_RUST_EXTERN_C_BEGIN \
    void SelfInit_##name(configType *configData, const BskRustModuleRuntime *runtime); \
    void Reset_##name(configType *configData, uint64_t currentSimNanos, const BskRustModuleRuntime *runtime); \
    void Update_##name(configType *configData, uint64_t currentSimNanos, const BskRustModuleRuntime *runtime); \
    void Drop_##name(configType *configData); \
    BSK_RUST_EXTERN_C_END

#endif /* BSK_RUST_MODULE_H */
