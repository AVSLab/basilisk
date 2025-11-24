#pragma once

#include "msgTypeRegistry.h"

#include <type_traits>

namespace Basilisk {
namespace messaging {
namespace detail {

// Runs at static init
inline MsgTypeHandle
register_cpp_payload(const char* name, std::size_t size, std::size_t align)
{
    MsgTypeInfo info;
    info.name = name;
    info.size = size;
    info.align = align;
    info.version = 1;
    info.schema_hash = 0; // can upgrade later

    return MsgTypeRegistry::global().register_type(info);
}

} // namespace detail
} // namespace messaging
} // namespace Basilisk

// -----------------------------------------------------------------------------
// C++ payload auto-registration
// -----------------------------------------------------------------------------
#define BSK_AUTO_REGISTER_MSG(PayloadType)                                                                             \
    static_assert(std::is_standard_layout<PayloadType>::value, "Payload must be standard layout");                     \
    static_assert(std::is_trivially_copyable<PayloadType>::value, "Payload must be trivially copyable");               \
    namespace {                                                                                                        \
    const ::Basilisk::messaging::MsgTypeHandle _bsk_msg_handle_##PayloadType =                                         \
      ::Basilisk::messaging::detail::register_cpp_payload(#PayloadType, sizeof(PayloadType), alignof(PayloadType));    \
    }

#ifdef __cplusplus
extern "C"
{
#endif

#define BSK_AUTO_REGISTER_MSG_C(PayloadType)                                                                           \
    static const int _bsk_msg_reg_##PayloadType = bsk_msg_register_payload_type(#PayloadType, sizeof(PayloadType))

#ifdef __cplusplus
}
#endif
