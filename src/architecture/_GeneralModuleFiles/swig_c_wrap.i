/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

%module swig_c_wrap

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "architecture/_GeneralModuleFiles/sys_model.h"
    #include "architecture/_GeneralModuleFiles/bsk_rust_module.h"
    #include <architecture/utilities/bskLogging.h>
    #include <memory>
    #include <type_traits>
%}
%include <std_string.i>

%include "sys_model.i"

// Forward-declare so RustWrapper's template arguments (below) type-check
// under SWIG's parser too; the full definition is only needed by the C++
// compiler (it is pulled in above via bsk_rust_module.h).
typedef struct BskRustModuleRuntime BskRustModuleRuntime;
typedef struct BskRustModuleContext BskRustModuleContext;

%inline %{

template <typename TConfig,
          void (*updateStateFun)(TConfig*, uint64_t, int64_t),
          void (*selfInitFun)(TConfig*, int64_t),
          void (*resetFun)(TConfig*, uint64_t, int64_t)
          >
class CWrapper : public SysModel {
    static_assert(std::is_default_constructible_v<TConfig>,
                  "The wrapped config must be default constructible (all 'Config' struct used in C "
                  "should be).");

  public:
    CWrapper() : config{std::make_unique<TConfig>()} {};
    CWrapper(TConfig* config) : config{config} {}; // We take ownership of config

    void SelfInit(){
        selfInitFun(this->config.get(), this->moduleID);};

    void UpdateState(uint64_t currentSimNanos){
        updateStateFun(this->config.get(), currentSimNanos, this->moduleID);};

    void Reset(uint64_t currentSimNanos){
        resetFun(this->config.get(), currentSimNanos, this->moduleID);};

    // Allows accessing the elements of the config from the wrapper in Python
    // Similar to how the smart pointers are implemented in SWIG
    TConfig* operator->() const { return this->config.get(); }

    TConfig& getConfig() { return *this->config.get(); }

  private:
    std::unique_ptr<TConfig> config; //!< class variable
};

// Same role as CWrapper, but for Rust-backed modules (see bsk_rust_module.h).
// Rust owns the opaque instance and every value behind it. This wrapper owns
// only that handle; its config pointer is a borrowed parameter/message-port
// view used by SWIG. Runtime metadata and logging are borrowed through a
// BskRustModuleContext created immediately before each lifecycle call.
template <typename TConfig,
          typename THandle,
          THandle* (*createInstanceFun)(),
          TConfig* (*configViewFun)(THandle*),
          void (*destroyInstanceFun)(THandle*),
          void (*updateStateFun)(THandle*, uint64_t, const BskRustModuleContext*),
          void (*selfInitFun)(THandle*, const BskRustModuleContext*),
          void (*resetFun)(THandle*, uint64_t, const BskRustModuleContext*)
          >
class RustWrapper : public SysModel {
  public:
    RustWrapper()
        : instance{createInstanceFun(), destroyInstanceFun},
          config{configViewFun(this->instance.get())} {};

    BSKLogger *bskLogger = nullptr; //!< framework logger borrowed by lifecycle context

    void SelfInit(){
        BskRustModuleContext context = this->makeContext();
        selfInitFun(this->instance.get(), &context);};

    void UpdateState(uint64_t currentSimNanos){
        BskRustModuleContext context = this->makeContext();
        updateStateFun(this->instance.get(), currentSimNanos, &context);};

    void Reset(uint64_t currentSimNanos){
        BskRustModuleContext context = this->makeContext();
        resetFun(this->instance.get(), currentSimNanos, &context);};

    // Allows accessing the elements of the config from the wrapper in Python
    // Similar to how the smart pointers are implemented in SWIG
    TConfig* operator->() const { return this->config; }

  private:
    // modelTag points into this->ModelTag, which outlives the synchronous
    // lifecycle call that consumes the context. bskLogger is owned by the
    // simulation and borrowed by the wrapper for that same call.
    BskRustModuleContext makeContext() const {
        BskRustModuleContext context;
        context.runtime.moduleID = this->moduleID;
        context.runtime.modelTag = this->ModelTag.c_str();
        context.runtime.callCounts = this->CallCounts;
        context.runtime.rngSeed = this->RNGSeed;
        context.bskLogger = this->bskLogger;
        return context;
    }

    // The instance returns to Rust for destruction. Config is a non-owning
    // view into that instance and must never be deleted by C++.
    std::unique_ptr<THandle, void (*)(THandle*)> instance; //!< owning Rust handle
    TConfig *config;                                      //!< borrowed config view
};

%}

%define %c_wrap_3(moduleName, configName, functionSuffix)
    // This macro expects the header file to be "[moduleName].h"
    // the 'Config' structure to be called [configName],
    // and the implementation functions to follow the pattern:
    //    Update_[functionSuffix]
    //    SelfInit_[functionSuffix]
    //    Reset_[functionSuffix]

    %ignore Update_ ## functionSuffix;
    %ignore SelfInit_ ## functionSuffix;
    %ignore Reset_ ## functionSuffix;

    /*
    We define the Reset method for the given moduleName as empty.
    We make this method templated, so that it has lower priority
    in overload resolution than methods using the explicit type.

    This means that:

    Reset_hillPoint(hillPointConfig*, uint64_t, int64_t) { ... }

    will always be chosen before:

    template <typename T> Reset_hillPoint(T, uint64_t, int64_t) {}

    which effectively means that the empty method will only be used if
    users do not provide their own Reset method.
    */
    %inline %{
      template <typename T> inline void Reset_ ## functionSuffix(T, uint64_t, int64_t) {}
    %}

    // Config and CWrapper constructors register embedded Msg_C storage owners.
    // When a wrapper takes a config pointer, transferModuleOwner moves existing
    // keep-alives to the wrapper. Keep these pythonappend bodies free of apostrophes
    // and hash comment lines because SWIG macro expansion mis-parses them.
    %pythonappend configName::configName() %{
        from Basilisk.architecture.messaging import _msgKeepAlive
        _msgKeepAlive.registerModule(self)
    %}

    %pythonappend CWrapper::CWrapper %{
        from Basilisk.architecture.messaging import _msgKeepAlive
        if (len(args)) > 0:
            args[0].thisown = False
            _msgKeepAlive.transferModuleOwner(args[0], self)
        else:
            _msgKeepAlive.registerModule(self)
    %}

    %include "moduleName.h"

    %template(moduleName) CWrapper<configName,Update_ ## functionSuffix,SelfInit_ ## functionSuffix,Reset_ ## functionSuffix>;

    %extend configName {
      %pythoncode %{
        def createWrapper(self):
            return moduleName(self)
      %}
    }

%enddef


%define %c_wrap_2(moduleName, configName)
    // This macro expects the header file to be "[moduleName].h"
    // the 'Config' structure to be called [configName],
    // and the implementation functions to follow the pattern:
    //    Update_[moduleName]
    //    SelfInit_[moduleName]
    //    Reset_[moduleName]
    %c_wrap_3(moduleName, configName, moduleName)
%enddef

%define %c_wrap(moduleName)
    // This macro expects the header file to be "[moduleName].h"
    // the 'Config' structure to be called [moduleName]Config,
    // and the implementation functions to follow the pattern:
    //    Update_[moduleName]
    //    SelfInit_[moduleName]
    //    Reset_[moduleName]

    %c_wrap_2(moduleName, moduleName ## Config)
%enddef

%define %rust_wrap_2(moduleName, configName, handleName)
    // The Rust build layer exports these lifecycle symbols. Keep the raw
    // functions out of Python; users interact with configName and the
    // RustWrapper specialization below.
    %ignore Create_ ## moduleName;
    %ignore Config_ ## moduleName;
    %ignore Destroy_ ## moduleName;
    %ignore Update_ ## moduleName;
    %ignore SelfInit_ ## moduleName;
    %ignore Reset_ ## moduleName;

    // SWIG 4.4 requires a complete template type here and crashes when the
    // opaque handle is only forward-declared. This parser-only empty
    // definition is not emitted into the generated C++; nodefault prevents
    // SWIG from generating invalid new/delete wrappers for the real opaque
    // C handle declared by bsk_rust_module.h.
    %ignore handleName;
    %nodefaultctor handleName;
    %nodefaultdtor handleName;
    typedef struct handleName {} handleName;

    /*
    Supply the same optional Reset behavior as %c_wrap_3. A concrete
    Reset_moduleName function takes precedence over this function template.
    */
    %inline %{
      template <typename T> inline void Reset_ ## moduleName(
          T, uint64_t, const BskRustModuleContext*) {}
    %}

    %template(moduleName) RustWrapper<
        configName,
        handleName,
        Create_ ## moduleName,
        Config_ ## moduleName,
        Destroy_ ## moduleName,
        Update_ ## moduleName,
        SelfInit_ ## moduleName,
        Reset_ ## moduleName>;
%enddef
