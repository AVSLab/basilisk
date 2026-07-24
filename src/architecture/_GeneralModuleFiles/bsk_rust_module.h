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

/*! @brief Declares Rust-owned module allocation plus the three BSK lifecycle
 *  entry points for a module implemented in Rust.
 *
 *  **Background**
 *
 *  Basilisk C modules consist of two parts:
 *
 *  1. A plain-C *config struct* that holds all parameters, message ports,
 *     and optional persistent state.
 *  2. Three *lifecycle functions* — ``SelfInit``, ``Reset``, ``Update`` —
 *     called by the Basilisk task scheduler at well-defined points.
 *
 *  For Rust modules the lifecycle functions are compiled into a Rust static
 *  library linked into the SWIG-generated Python module. ``BSK_RUST_DECL`` emits the
 *  matching ``extern "C"`` *declarations* so that the C compiler and the
 *  SWIG-generated glue can find them. Unlike the C-module wrapper, the Rust
 *  wrapper hides direct parameter fields and exposes generated properties
 *  backed by guarded Rust getters and setters.
 *
 *  **Macro-generated workflow**
 *
 *  Writing raw ``unsafe extern "C"`` Rust is error-prone.  The recommended
 *  workflow uses ``build.rs`` to generate the C header while the
 *  ``#[bsk_build::module]`` procedural attribute emits the ``extern "C"``
 *  lifecycle entry points that
 *  read/write messages around the module's own ``update``. The user
 *  implements ``init``, ``reset``, and ``update`` in safe Rust with
 *  named, typed message values and ``BskResult`` return types — no FFI
 *  boilerplate by hand. Message-port fields use ``#[bsk(input)]``,
 *  ``#[bsk(input, optional)]``, or ``#[bsk(output)]`` to declare their role
 *  explicitly. See the Basilisk documentation's "Making Rust Modules"
 *  page for the full guide.
 *
 *  **Config struct field ordering**
 *
 *  A Rust module config contains only Python-visible parameters and message
 *  ports. Framework metadata, logging, and internal Rust state live outside
 *  this FFI view. The suggested layout is::
 *
 *      typedef struct {
 *          // 1. Scalar / array parameters
 *          double K;                             //!< [Nm]       proportional gain
 *          double P;                             //!< [Nm/(rad/s)] rate gain
 *
 *          // 2. Input message ports
 *          AttGuidMsg_C attGuidInMsg;            //!< [-]  attitude guidance
 *
 *          // 3. Output message ports
 *          CmdTorqueBodyMsg_C cmdTorqueOutMsg;   //!< [Nm] control torque
 *      } myModuleConfig;
 *
 *      BSK_RUST_DECL(myModule, myModuleConfig, myModuleConfigHandle)
 *
 *  On the Rust side, ``attGuidInMsg``/``cmdTorqueOutMsg`` above are annotated
 *  with ``#[bsk(input)]``/``#[bsk(output)]`` and use
 *  ``MsgReader<AttGuidMsg>``/``MsgWriter<CmdTorqueBodyMsg>`` — see the
 *  "Making Rust Modules" documentation page for the complete Rust form.
 *
 *  **moduleID**
 *
 *  ``moduleID`` is a unique ``int64_t`` assigned by Basilisk's
 *  ``ModuleIdGenerator`` when the Python-visible C++ wrapper's ``SysModel``
 *  base is constructed. Task registration does not assign the ID. It is
 *  stamped onto every outgoing message header (via ``*_C_write``) so that
 *  message recording and the logging subsystem can identify which module
 *  produced a given message. The generated lifecycle code forwards it to
 *  every ``*_C_write`` call automatically.
 *
 *  **Lifecycle context — BskRustModuleRuntime**
 *
 *  A Rust module has no C++ base class, so this struct mirrors the relevant
 *  ``SysModel`` fields (module ID, name, ...) for each lifecycle call.
 *  ``BskContext`` gives safe Rust module logic a borrowed view of that
 *  snapshot. Runtime services do not appear in the public config struct.
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
 *  C module has through the no-throw logging adapter. Its borrowed
 *  ``BskLoggerRef`` provides
 *  ``.debug()``/``.info()``/``.warning()`` methods.
 *  The shared wrapper borrows its framework-managed logger into the lifecycle
 *  context. The logger does not appear in the public config struct.
 *  A no-throw C++ adapter catches any logger exception before returning to
 *  Rust. Expected configuration, input, and runtime failures return
 *  ``Err(BskError::new(...))`` from a lifecycle method; they are not logging
 *  operations. The generated boundary carries that failure as data and
 *  raises ``BasiliskError`` only after Rust has returned normally.
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
 *  before each ``Update`` read; a missing connection returns an expected
 *  Rust error that the C++ wrapper translates into ``BasiliskError``.
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
 *      sim.AddModelToTask("task", ctrl)               # schedule the module
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
 *          // reset()/update() receive &mut Self::State and return BskResult
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
 *          pub target: Vec2,
 *          // ...
 *      }
 *
 *  ``bsk-build`` generates ``Vec2``'s C struct alongside ``myModuleConfig``
 *  and Python transfers the complete nested value through the generated
 *  getter and setter.
 *
 *  A raw pointer to one of these structs (``*mut Vec2``), or to any other
 *  type, is rejected.
 *  SWIG's pointer-field setter would transfer ownership away from the
 *  Python object with nothing on the Rust side to ever free it; this
 *  applies just as much to a pointer to a primitive (``*mut u8``) as to a
 *  struct, so there is currently no supported field type for a persistent
 *  string or byte-buffer parameter.
 *
 *  A field also may not be a Rust ``enum``, even a fieldless ``#[repr(u8)]``
 *  (or similar) one. An invalid integer discriminant copied across an FFI
 *  boundary would be undefined behavior before Rust could validate it. Use
 *  the underlying integer type as the field (e.g. ``pub mode: u8``), validate
 *  it in the generated setter, and convert it to the Rust enum only after the
 *  value is known to be valid.
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
 *  Embedded in the context passed to every lifecycle call and valid only for
 *  that call. Do not retain it afterward; ``modelTag`` in particular is a
 *  borrowed pointer.
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

/*! Classification for an error returned by a Rust module ABI function.
 *
 *  This is a fixed-width integer rather than a C enum so its representation
 *  exactly matches Rust's ``#[repr(u32)] BskRustErrorKind`` on every supported
 *  compiler.
 */
typedef uint32_t BskRustErrorKind;

enum {
    BSK_RUST_ERROR_EXPECTED = 1U,        /*!< module returned an expected error */
    BSK_RUST_ERROR_PANIC = 2U,           /*!< caught panic or call on poisoned instance */
    BSK_RUST_ERROR_INVALID_ARGUMENT = 3U /*!< lifecycle ABI contract was violated */
};

/*! Opaque Rust-owned error and diagnostic message.
 *
 *  A non-null error returned by a Rust module must be released with
 *  ``Destroy_BskRustError``. Its message remains valid until that call.
 */
typedef struct BskRustError BskRustError;

BSK_RUST_EXTERN_C_BEGIN
BskRustErrorKind BskRustError_kind(const BskRustError *error);
const char *BskRustError_message(const BskRustError *error);
void Destroy_BskRustError(BskRustError *error);
BSK_RUST_EXTERN_C_END

/*! Emit ``extern "C"`` allocation and lifecycle function declarations for a
 *  Rust-backed Basilisk module named \p name, whose config view is
 *  \p configType and whose opaque instance type is \p handleType.
 *
 *  ``Create_name(&handle)``
 *      Constructs the complete module instance in Rust, including arbitrary
 *      Rust-owned state, and writes its opaque owning handle. Returns null on
 *      success or an owning ``BskRustError`` on failure.
 *
 *  ``Config_name(handle)``
 *      Returns a borrowed pointer to the instance's FFI-safe parameter and
 *      message-port view. The pointer remains valid until ``Destroy_name``.
 *      Generated wrappers use this view only for message ports; Python-facing
 *      configuration values use the guarded accessors below.
 *
 *  ``GetConfigField_name`` / ``SetConfigField_name``
 *      Copy one generated configuration value across the C boundary. The
 *      field index and byte size are generated from the same Rust struct.
 *      Setters validate the complete typed value before changing it and
 *      return an expected error without modifying the previous value when
 *      validation fails.
 *
 *  ``ConfigFieldDeprecationDate_name`` /
 *  ``ConfigFieldDeprecationMessage_name``
 *      Return static metadata used by the generated Python getter, setter,
 *      and property. Null means that the field is not deprecated.
 *
 *  ``Destroy_name(handle)``
 *      Runs the config and internal state's Rust drop glue and returns the
 *      complete allocation to Rust. Returns null on success or an owning
 *      ``BskRustError`` if Rust catches a panic while dropping the instance.
 *
 *  ``SelfInit_name(handle, context)``
 *      Called once at task registration. The generated lifecycle code
 *      initialises output message ports (``*_C_init``). Returns null on
 *      success or an owning ``BskRustError`` on failure.
 *
 *  ``Reset_name(handle, currentSimNanos, context)``
 *      Called before the first step and on explicit resets. The generated
 *      lifecycle code checks required input connectivity, then calls
 *      ``BskModule::reset`` with a safe borrowed context. It writes outputs
 *      only after ``reset`` returns ``Ok``. Returns null on success or an
 *      owning ``BskRustError`` on failure.
 *
 *  ``Update_name(handle, currentSimNanos, context)``
 *      Called every simulation step. The generated lifecycle code reads all
 *      input messages, calls ``BskModule::update`` with a safe borrowed
 *      context, and writes output messages only after ``update`` returns
 *      ``Ok``. Returns null on success or an owning ``BskRustError`` on
 *      failure.
 *
 *  Every generated Rust definition uses the non-unwinding C ABI and catches
 *  Rust panics before returning. The caller owns every non-null error result
 *  and must release it with ``Destroy_BskRustError``.
 *  A panic caught during ``SelfInit``, ``Reset``, or ``Update`` poisons that
 *  module instance because its internal invariants may be incomplete.
 *  Subsequent lifecycle calls return an error without re-entering module
 *  code. Expected ``BskError`` results do not poison the instance, and
 *  ``Destroy_name`` remains valid for poisoned instances.
 *  The guarded boundary returns the panic diagnostic through
 *  ``BskRustError`` and suppresses duplicate default Rust panic-hook output
 *  only on the thread executing that call. Panics outside a generated
 *  boundary continue through the previously installed application hook.
 *
 *  \p configType must be declared before this macro. bsk-build passes
 *  whatever struct name the crate's ``impl BskModule`` block actually uses,
 *  which need not match ``name##Config``.
 */
#define BSK_RUST_DECL(name, configType, handleType) \
    typedef struct handleType handleType; \
    BSK_RUST_EXTERN_C_BEGIN \
    BskRustError *Create_##name(handleType **outputHandle); \
    configType *Config_##name(handleType *handle); \
    BskRustError *GetConfigField_##name(handleType *handle, size_t fieldIndex, void *outputValue, size_t valueSize); \
    BskRustError *SetConfigField_##name(handleType *handle, size_t fieldIndex, const void *inputValue, size_t valueSize); \
    const char *ConfigFieldDeprecationDate_##name(size_t fieldIndex); \
    const char *ConfigFieldDeprecationMessage_##name(size_t fieldIndex); \
    BskRustError *Destroy_##name(handleType *handle); \
    BskRustError *SelfInit_##name(handleType *handle, const BskRustModuleContext *context); \
    BskRustError *Reset_##name(handleType *handle, uint64_t currentSimNanos, const BskRustModuleContext *context); \
    BskRustError *Update_##name(handleType *handle, uint64_t currentSimNanos, const BskRustModuleContext *context); \
    BSK_RUST_EXTERN_C_END

#endif /* BSK_RUST_MODULE_H */
