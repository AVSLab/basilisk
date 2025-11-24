#pragma once

/**
 * Minimal helper utilities for writing Basilisk C++ plugins.
 *
 * This header wraps a small amount of pybind11 boilerplate so that
 * plugin authors can focus on module logic, not binding details.
 *
 * Design goals:
 *  - No Basilisk internals required in plugins
 *  - No manual message registration by plugin authors (just use macro)
 *  - Safe default behavior
 *  - Forward-compatible with future SDK expansion
 *
 * Usage:
 *
 *   #include <bsk/sdk.hpp>
 *
 *   struct MyPayload { ... };
 *
 *   class MyCppModule {
 *   public:
 *       void Reset(uint64_t t);
 *       void UpdateState(uint64_t t);
 *   };
 *
 *   PYBIND11_MODULE(_my_module, m) {
 *       BSK_PLUGIN_REGISTER_PAYLOAD(MyPayload);
 *       auto cls = bsk::plugin::bind_module<MyCppModule>(m, "MyCppModule");
 *       m.def("create_factory", [](){ return bsk::plugin::make_factory<MyCppModule>(); });
 *   }
 */

#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

namespace bsk::plugin {

namespace detail {

// ---------- compile-time detection helpers ----------

template<typename, typename = void>
struct has_reset : std::false_type
{};

template<typename T>
struct has_reset<T, std::void_t<decltype(std::declval<T&>().Reset(std::declval<uint64_t>()))>> : std::true_type
{};

template<typename, typename = void>
struct has_update_state : std::false_type
{};

template<typename T>
struct has_update_state<T, std::void_t<decltype(std::declval<T&>().UpdateState(std::declval<uint64_t>()))>>
  : std::true_type
{};

template<typename, typename = void>
struct has_reset_flag : std::false_type
{};

template<typename T>
struct has_reset_flag<T, std::void_t<decltype(std::declval<const T&>().reset_called())>> : std::true_type
{};

template<typename, typename = void>
struct has_update_flag : std::false_type
{};

template<typename T>
struct has_update_flag<T, std::void_t<decltype(std::declval<const T&>().update_called())>> : std::true_type
{};

} // namespace detail

// ---------- factory helpers ----------

template<typename Module>
pybind11::cpp_function
make_factory()
{
    static_assert(std::is_default_constructible_v<Module>, "Basilisk plugin modules must be default constructible");
    return pybind11::cpp_function([]() { return Module(); });
}

// ---------- class binding ----------

template<typename Module>
pybind11::class_<Module>
bind_module(pybind11::module_& m, const char* python_name)
{
    auto cls = pybind11::class_<Module>(m, python_name);

    if constexpr (std::is_default_constructible_v<Module>) {
        cls.def(pybind11::init<>());
    }

    if constexpr (detail::has_reset<Module>::value) {
        cls.def("Reset", &Module::Reset);
    }

    if constexpr (detail::has_update_state<Module>::value) {
        cls.def("UpdateState", &Module::UpdateState);
    }

    if constexpr (detail::has_reset_flag<Module>::value) {
        cls.def_property_readonly("reset_called", &Module::reset_called);
    }

    if constexpr (detail::has_update_flag<Module>::value) {
        cls.def_property_readonly("update_called", &Module::update_called);
    }

    return cls;
}

// ---------- high-level registration ----------

template<typename Module>
inline void
register_basic_plugin(pybind11::module_& m, const char* class_name)
{
    bind_module<Module>(m, class_name);

    // Required entry point consumed by Basilisk runtime
    m.def("create_factory", []() { return make_factory<Module>(); });
}

} // namespace bsk::plugin

// ---------- stable C ABI from Basilisk core (plugins link to this) ----------
//
// IMPORTANT:
// - This symbol must exist in the running Basilisk Python package / dylibs.
// - Plugins just declare it and call it.
// - It must be exported from Basilisk core on macOS.
//
extern "C" int
bsk_msg_register_payload_type_ex(const char* type_name,
                                 unsigned long payload_size_bytes,
                                 unsigned long payload_align_bytes,
                                 unsigned int version,
                                 unsigned long long schema_hash);

namespace bsk::plugin {

// Simple payload registrar used by macros below.
inline void
register_payload_or_throw(const char* name,
                          unsigned long size,
                          unsigned long align,
                          unsigned int version = 1u,
                          unsigned long long schema_hash = 0ull)
{
    const int rc = bsk_msg_register_payload_type_ex(name, size, align, version, schema_hash);
    if (rc != 0) {
        throw std::runtime_error(std::string("Failed to register payload type: ") + name);
    }
}

} // namespace bsk::plugin

// ---------- payload registration macros ----------
//
// IMPORTANT:
// Use the TYPE form in plugins:
//
//   BSK_PLUGIN_REGISTER_PAYLOAD(MyPayload);
//
// This is robust and produces the desired registered name "MyPayload".
//
// If you *really* want to pass an expression, you must provide the name explicitly:
//
//   BSK_PLUGIN_REGISTER_PAYLOAD_NAMED("MyPayload", MyPayload{});
//
#define BSK_PLUGIN_REGISTER_PAYLOAD(PayloadType)                                                                       \
    ::bsk::plugin::register_payload_or_throw(                                                                          \
      #PayloadType, (unsigned long)sizeof(PayloadType), (unsigned long)alignof(PayloadType), 1u, 0ull)

#define BSK_PLUGIN_REGISTER_PAYLOAD_NAMED(type_name_cstr, payload_expr)                                                \
    ::bsk::plugin::register_payload_or_throw((type_name_cstr),                                                         \
                                             (unsigned long)sizeof(decltype(payload_expr)),                            \
                                             (unsigned long)alignof(decltype(payload_expr)),                           \
                                             1u,                                                                       \
                                             0ull)
