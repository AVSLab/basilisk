#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>

namespace Basilisk {
namespace messaging {

using MsgTypeHandle = uint32_t; // 0 reserved as invalid

struct MsgTypeInfo
{
    std::string name;
    std::size_t size = 0;
    std::size_t align = 0;
    uint32_t version = 1;
    uint64_t schema_hash = 0;
};

class MsgTypeRegistryError : public std::runtime_error
{
  public:
    explicit MsgTypeRegistryError(const std::string& what_arg)
      : std::runtime_error(what_arg)
    {
    }
};

class MsgTypeRegistry
{
  public:
    MsgTypeRegistry();

    MsgTypeHandle register_type(const MsgTypeInfo& info);

#ifndef SWIG
    MsgTypeHandle lookup(std::string_view name) const;
#else
    // SWIG cannot parse std::string_view
    MsgTypeHandle lookup(const std::string& name) const;
#endif

    const MsgTypeInfo& info(MsgTypeHandle handle) const;

    static MsgTypeRegistry& global();

  private:
    static void validate_info(const MsgTypeInfo& info);

    mutable std::mutex mtx_;
    std::unordered_map<std::string, MsgTypeHandle> by_name_;
    std::unordered_map<MsgTypeHandle, MsgTypeInfo> by_handle_;
    MsgTypeHandle next_handle_ = 1;
};

} // namespace messaging
} // namespace Basilisk

// -----------------------------------------------------------------------------
// C ABI
// -----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

    int bsk_msg_register_payload_type_ex(const char* type_name,
                                         unsigned long payload_size_bytes,
                                         unsigned long payload_align_bytes,
                                         unsigned int version,
                                         unsigned long long schema_hash);

    int bsk_msg_register_payload_type(const char* type_name, unsigned long payload_size_bytes);

#ifdef __cplusplus
}
#endif
