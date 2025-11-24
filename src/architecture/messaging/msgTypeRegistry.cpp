#include "msgTypeRegistry.h"

#include <cstring>
#include <utility>

namespace Basilisk {
namespace messaging {

namespace {
inline bool
is_power_of_two(std::size_t x)
{
    return x != 0 && ((x & (x - 1)) == 0);
}
} // namespace

MsgTypeRegistry::MsgTypeRegistry() = default;

void
MsgTypeRegistry::validate_info(const MsgTypeInfo& info)
{
    if (info.name.empty()) {
        throw MsgTypeRegistryError("MsgTypeRegistry: name must be non-empty");
    }
    if (info.size == 0) {
        throw MsgTypeRegistryError("MsgTypeRegistry: size must be non-zero");
    }
    if (info.align == 0 || !is_power_of_two(info.align)) {
        throw MsgTypeRegistryError("MsgTypeRegistry: alignment must be power of two");
    }
}

MsgTypeHandle
MsgTypeRegistry::register_type(const MsgTypeInfo& info)
{
    validate_info(info);

    std::lock_guard<std::mutex> lock(mtx_);

    auto it = by_name_.find(info.name);
    if (it != by_name_.end()) {
        MsgTypeHandle handle = it->second;
        const auto& existing = by_handle_.at(handle);

        if (existing.size != info.size || existing.align != info.align || existing.version != info.version ||
            existing.schema_hash != info.schema_hash) {
            throw MsgTypeRegistryError("MsgTypeRegistry: incompatible re-registration of type '" + info.name + "'");
        }

        return handle;
    }

    MsgTypeHandle handle = next_handle_++;
    by_name_.emplace(info.name, handle);
    by_handle_.emplace(handle, info);
    return handle;
}

#ifndef SWIG
MsgTypeHandle
MsgTypeRegistry::lookup(std::string_view name) const
{
    if (name.empty()) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    auto it = by_name_.find(std::string(name));
    return (it == by_name_.end()) ? 0 : it->second;
}
#else
MsgTypeHandle
MsgTypeRegistry::lookup(const std::string& name) const
{
    if (name.empty()) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    auto it = by_name_.find(name);
    return (it == by_name_.end()) ? 0 : it->second;
}
#endif

const MsgTypeInfo&
MsgTypeRegistry::info(MsgTypeHandle handle) const
{
    if (handle == 0) {
        throw MsgTypeRegistryError("MsgTypeRegistry: invalid handle 0");
    }

    std::lock_guard<std::mutex> lock(mtx_);
    auto it = by_handle_.find(handle);
    if (it == by_handle_.end()) {
        throw MsgTypeRegistryError("MsgTypeRegistry: unknown handle");
    }
    return it->second;
}

MsgTypeRegistry&
MsgTypeRegistry::global()
{
    static MsgTypeRegistry instance;
    return instance;
}

} // namespace messaging
} // namespace Basilisk

// -----------------------------------------------------------------------------
// C ABI
// -----------------------------------------------------------------------------

extern "C"
{

    static int validate_cstr(const char* s)
    {
        return (s != nullptr && s[0] != '\0');
    }

    int bsk_msg_register_payload_type_ex(const char* type_name,
                                         unsigned long payload_size_bytes,
                                         unsigned long payload_align_bytes,
                                         unsigned int version,
                                         unsigned long long schema_hash)
    {
        try {
            if (!validate_cstr(type_name) || payload_size_bytes == 0 || payload_align_bytes == 0) {
                return 1;
            }

            Basilisk::messaging::MsgTypeInfo info;
            info.name = type_name;
            info.size = static_cast<std::size_t>(payload_size_bytes);
            info.align = static_cast<std::size_t>(payload_align_bytes);
            info.version = version;
            info.schema_hash = schema_hash;

            Basilisk::messaging::MsgTypeRegistry::global().register_type(info);
            return 0;
        } catch (...) {
            return 2;
        }
    }

    int bsk_msg_register_payload_type(const char* type_name, unsigned long payload_size_bytes)
    {
        return bsk_msg_register_payload_type_ex(
          type_name, payload_size_bytes, static_cast<unsigned long>(alignof(void*)), 1u, 0ull);
    }

} // extern "C"
