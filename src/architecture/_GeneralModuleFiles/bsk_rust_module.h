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

/*! @brief Declares Rust-owned configuration allocation plus the three BSK
 *  lifecycle entry points for a module implemented in Rust.
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
 *  **Macro-generated workflow**
 *
 *  Writing raw ``unsafe extern "C"`` Rust is error-prone.  The recommended
 *  workflow uses ``build.rs`` to generate the C header while the
 *  ``#[bsk_build::module]`` procedural attribute emits the ``extern "C"``
 *  lifecycle entry points that
 *  read/write messages around the module's own ``update``. The user
 *  implements ``init``, ``reset``, and ``update`` in safe Rust with
 *  named, typed message values — no FFI boilerplate by hand. Message-port
 *  fields use ``#[bsk(input)]``, ``#[bsk(input, optional)]``, or
 *  ``#[bsk(output)]`` to declare their role explicitly. See the Basilisk
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
 *          // 3. Input message ports
 *          AttGuidMsg_C attGuidInMsg;            //!< [-]  attitude guidance
 *
 *          // 4. Output message ports
 *          CmdTorqueBodyMsg_C cmdTorqueOutMsg;   //!< [Nm] control torque
 *
 *          // 5. BSK logger
 *          BSKLogger *bskLogger;                 //!< [-]  BSK logging handle
 *      } myModuleConfig;
 *
 *      BSK_RUST_DECL(myModule, myModuleConfig, myModuleConfigHandle)
 *
 *  On the Rust side, ``attGuidInMsg``/``cmdTorqueOutMsg`` above are annotated
 *  with ``#[bsk(input)]``/``#[bsk(output)]`` and use
 *  ``MsgReader<AttGuidMsg>``/``MsgWriter<CmdTorqueBodyMsg>`` — see the
 *  "Writing a Rust Plugin" documentation page for the complete Rust form.
 *
 *  **moduleID**
 *
 *  ``moduleID`` is a unique ``int64_t`` assigned by Basilisk's
 *  ``ModuleIdGenerator`` when the module is registered with a task.  It is
 *  stamped onto every outgoing message header (via ``*_C_write``) so that
 *  message recording and the logging subsystem can identify which module
 *  produced a given message. The generated lifecycle code forwards it to
 *  every ``*_C_write`` call automatically.
 *
 *  **Runtime mirror — BskRustModuleRuntime**
 *
 *  A Rust module has no C++ base class, so this struct mirrors the relevant
 *  ``SysModel`` fields (module ID, name, ...) for each lifecycle call.
 *  ``BskContext`` gives safe Rust module logic a borrowed view of that
 *  snapshot. The config's own ``runtime`` field is refreshed only as a
 *  temporary compatibility measure for the public configuration view.
 *
 *  ``modelTag`` is a borrowed pointer valid only for the duration of the
 *  call. On the Rust side this is enforced by the compiler, not just this
 *  comment: ``BskContext::model_tag()`` returns a ``&str`` tied to that
 *  context borrow, so safe module logic cannot retain it past the lifecycle
 *  call that received it.
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
 *  ``BskContext::logger()`` supplies the same standard logging a hand-written
 *  C module has through ``_bskLog`` / ``_bskError``. ``bsk-messages``'
 *  ``BskLoggerExt`` trait wraps those entry points as
 *  ``.debug()``/``.info()``/``.warning()``/``.bsk_error()`` methods.
 *  The raw ``bskLogger`` config field remains only for the transitional
 *  configuration view; the shared wrapper borrows it into the lifecycle
 *  context. ``bsk-build``'s own generated "unconnected required input" check
 *  (see "Message port patterns" below) uses this identical path.
 *
 *  **Message port patterns**
 *
 *  Annotate each Rust config field so the port role is explicit. The
 *  procedural attribute generates named input and output value structs whose
 *  fields retain those config field names::
 *
 *      #[bsk(input)]
 *      pub attGuidInMsg: MsgReader<AttGuidMsg>,
 *      #[bsk(input, optional)]
 *      pub disturbanceInMsg: MsgReader<CmdTorqueBodyMsg>,
 *      #[bsk(output)]
 *      pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>,
 *
 *  *Required input* — ``#[bsk(input)]`` checks connectivity in ``Reset`` and
 *  before each ``Update`` read; a missing connection raises the standard
 *  ``BasiliskError``.
 *
 *  *Optional input* — ``#[bsk(input, optional)]`` gives the generated input
 *  field type ``Option<Msg>`` (``None`` when unlinked) instead of raising an
 *  error.
 *
 *  *Output* — ``#[bsk(output)]`` is initialized automatically in
 *  ``SelfInit`` and written from the same named field returned by ``reset``
 *  or ``update``.
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
 *  **Stateful modules — Rust-owned state**
 *
 *  Modules that need persistent implementation state set the
 *  ``BskModule::State`` associated type. This state is stored beside the
 *  config inside the opaque Rust module instance and never crosses the FFI
 *  boundary. It may therefore contain ordinary Rust collections, strings,
 *  enums, and smart pointers::
 *
 *      #[derive(Default)]
 *      pub struct MyState {
 *          history: Vec<f64>,
 *          status: String,
 *      }
 *
 *      impl BskModule for myModuleConfig {
 *          type State = MyState;
 *          // reset()/update() receive &mut Self::State
 *      }
 *
 *  The generated ``Destroy_name`` function runs ordinary Rust drop glue for
 *  both the config and state. No ``Cleanup_*`` function, raw state pointer,
 *  or custom destructor is needed. Stateless modules use ``type State = ();``.
 *
 *  **Grouping parameters — nested structs**
 *
 *  A field may be another ``#[repr(C)]`` struct defined in the same crate,
 *  by value (not a pointer), to group related parameters::
 *
 *      #[repr(C)]
 *      pub struct Vec2 { pub x: f64, pub y: f64 }
 *
 *      #[bsk_build::module]
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

/*! @brief Borrowed framework services supplied to a Rust lifecycle call.
 *
 *  The wrapper owns every referenced value. Rust may use this structure only
 *  for the duration of the lifecycle call and must not retain either pointer.
 *  ``runtime.modelTag`` borrows the wrapper's ``SysModel::ModelTag`` storage,
 *  while ``bskLogger`` refers to the wrapper's logging object.
 *
 *  The shared C++ wrapper constructs this value immediately before each
 *  opaque-handle lifecycle call.
 */
typedef struct BskRustModuleContext {
    BskRustModuleRuntime runtime; /*!< [-] borrowed SysModel runtime snapshot */
    BSKLogger *bskLogger;         /*!< [-] borrowed Basilisk logger */
} BskRustModuleContext;

/*! Emit ``extern "C"`` allocation and lifecycle function declarations for a
 *  Rust-backed Basilisk module named \p name, whose config view is
 *  \p configType and whose opaque instance type is \p handleType.
 *
 *  ``Create_name()``
 *      Constructs the complete module instance in Rust, including arbitrary
 *      Rust-owned state, and returns its opaque owning handle.
 *
 *  ``Config_name(handle)``
 *      Returns a borrowed pointer to the instance's FFI-safe parameter and
 *      message-port view. The pointer remains valid until ``Destroy_name``.
 *
 *  ``Destroy_name(handle)``
 *      Runs the config and internal state's Rust drop glue and returns the
 *      complete allocation to Rust.
 *
 *  ``SelfInit_name(handle, context)``
 *      Called once at task registration. The generated lifecycle code
 *      initialises output message ports (``*_C_init``).
 *
 *  ``Reset_name(handle, currentSimNanos, context)``
 *      Called before the first step and on explicit resets. The generated
 *      lifecycle code checks required input connectivity, then calls
 *      ``BskModule::reset`` with a safe borrowed context.
 *
 *  ``Update_name(handle, currentSimNanos, context)``
 *      Called every simulation step. The generated lifecycle code reads all
 *      input messages, calls ``BskModule::update`` with a safe borrowed
 *      context, and writes all output messages.
 *
 *  \p configType must be declared before this macro and must have a field
 *  named ``runtime`` of type ``BskRustModuleRuntime`` somewhere in it (any
 *  position — see "Config struct field ordering" above). bsk-build passes
 *  whatever struct name the crate's ``impl BskModule`` block actually uses,
 *  which need not match ``name##Config``.
 */
#define BSK_RUST_DECL(name, configType, handleType) \
    typedef struct handleType handleType; \
    BSK_RUST_EXTERN_C_BEGIN \
    handleType *Create_##name(void); \
    configType *Config_##name(handleType *handle); \
    void Destroy_##name(handleType *handle); \
    void SelfInit_##name(handleType *handle, const BskRustModuleContext *context); \
    void Reset_##name(handleType *handle, uint64_t currentSimNanos, const BskRustModuleContext *context); \
    void Update_##name(handleType *handle, uint64_t currentSimNanos, const BskRustModuleContext *context); \
    BSK_RUST_EXTERN_C_END

#endif /* BSK_RUST_MODULE_H */
