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

    // Allows accesing the elements of the config from the wrapper in Python
    // Similar to how the smart pointers are implemented in SWIG
    TConfig* operator->() const { return this->config.get(); }

    TConfig& getConfig() { return *this->config.get(); }

  private:
    std::unique_ptr<TConfig> config; //!< class variable
};

// Same role as CWrapper, but for Rust-backed modules (see bsk_rust_module.h).
// Rust has no access to the C++ SysModel base class, so instead of exposing
// SysModel members as separate lifecycle arguments, this wrapper mirrors the
// subset a module is likely to need (moduleID, ModelTag, CallCounts, RNGSeed)
// into a BskRustModuleRuntime snapshot, refreshed immediately before every
// call. The generated Rust shim copies that snapshot into the config struct's
// own `runtime` field, so Rust code reads it the same way C module code reads
// SysModel: through the config, not a separate parameter list.
template <typename TConfig,
          void (*updateStateFun)(TConfig*, uint64_t, const BskRustModuleRuntime*),
          void (*selfInitFun)(TConfig*, const BskRustModuleRuntime*),
          void (*resetFun)(TConfig*, uint64_t, const BskRustModuleRuntime*)
          >
class RustWrapper : public SysModel {
    static_assert(std::is_default_constructible_v<TConfig>,
                  "The wrapped config must be default constructible (all 'Config' struct used in C "
                  "should be).");

  public:
    RustWrapper() : config{std::make_unique<TConfig>()} {};
    RustWrapper(TConfig* config) : config{config} {}; // We take ownership of config

    void SelfInit(){
        BskRustModuleRuntime runtime = this->makeRuntime();
        selfInitFun(this->config.get(), &runtime);};

    void UpdateState(uint64_t currentSimNanos){
        BskRustModuleRuntime runtime = this->makeRuntime();
        updateStateFun(this->config.get(), currentSimNanos, &runtime);};

    void Reset(uint64_t currentSimNanos){
        BskRustModuleRuntime runtime = this->makeRuntime();
        resetFun(this->config.get(), currentSimNanos, &runtime);};

    // Allows accesing the elements of the config from the wrapper in Python
    // Similar to how the smart pointers are implemented in SWIG
    TConfig* operator->() const { return this->config.get(); }

    TConfig& getConfig() { return *this->config.get(); }

  private:
    // modelTag points into this->ModelTag, which outlives the synchronous
    // lifecycle call that consumes the snapshot.
    BskRustModuleRuntime makeRuntime() const {
        BskRustModuleRuntime runtime;
        runtime.moduleID = this->moduleID;
        runtime.modelTag = this->ModelTag.c_str();
        runtime.callCounts = this->CallCounts;
        runtime.rngSeed = this->RNGSeed;
        return runtime;
    }

    std::unique_ptr<TConfig> config; //!< class variable
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

// There is no %rust_wrap macro family (unlike %c_wrap/%c_wrap_2/%c_wrap_3
// above): bsk-build generates the entire .i file for a Rust module (see
// `render_swig_interface` in bsk-build's src/lib.rs), so the %ignore /
// %template / %pythonappend / %extend boilerplate that would otherwise live
// in a macro here is written out directly in that generated file instead.
// There is no hand-written-.i escape hatch for Rust modules to justify the
// indirection of a macro with a single caller.
