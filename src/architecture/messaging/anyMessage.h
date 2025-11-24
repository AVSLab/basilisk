#pragma once

#include "architecture/messaging/msgTypeRegistry.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace Basilisk {
namespace messaging {

class AnyMessageError : public std::runtime_error
{
  public:
    explicit AnyMessageError(const std::string& what_arg)
      : std::runtime_error(what_arg)
    {
    }
};

class AnyMessage
{
  public:
    AnyMessage() = default;

    // Construct with a registered type handle (allocates storage).
    explicit AnyMessage(MsgTypeHandle handle) { set_type(handle); }

    // ---------------------------------------------------------------------
    // Type management
    // ---------------------------------------------------------------------

    void set_type(std::string_view type_name)
    {
        auto handle = MsgTypeRegistry::global().lookup(type_name);
        if (handle == 0) {
            throw AnyMessageError("AnyMessage: unknown message type name");
        }
        set_type(handle);
    }

    void set_type(MsgTypeHandle handle)
    {
        const auto& info = MsgTypeRegistry::global().info(handle);
        handle_ = handle;
        storage_.resize(info.size);
    }

    // ---------------------------------------------------------------------
    // Accessors
    // ---------------------------------------------------------------------

    MsgTypeHandle type() const { return handle_; }
    MsgTypeHandle type_handle() const { return handle_; }

    const MsgTypeInfo& type_info() const
    {
        if (handle_ == 0) {
            throw AnyMessageError("AnyMessage: type not set");
        }
        return MsgTypeRegistry::global().info(handle_);
    }

    std::size_t size() const { return storage_.size(); }
    std::size_t size_bytes() const { return storage_.size(); }

    const void* data() const { return storage_.empty() ? nullptr : storage_.data(); }
    void* data() { return storage_.empty() ? nullptr : storage_.data(); }

    uint64_t time_written_nanos() const { return time_written_nanos_; }

    // ---------------------------------------------------------------------
    // Write / read API
    // ---------------------------------------------------------------------

    // Primary API (timestamped)
    void write_bytes(const void* src, std::size_t nbytes, uint64_t time_nanos)
    {
        if (handle_ == 0) {
            throw AnyMessageError("AnyMessage: type not set before write");
        }
        if (nbytes != storage_.size()) {
            throw AnyMessageError("AnyMessage: payload size mismatch on write");
        }
        std::memcpy(storage_.data(), src, nbytes);
        time_written_nanos_ = time_nanos;
    }

    // Compatibility overload (used by SWIG + legacy code)
    void write_bytes(const void* src, std::size_t nbytes) { write_bytes(src, nbytes, 0); }

    void read_bytes(void* dst, std::size_t nbytes) const
    {
        if (handle_ == 0) {
            throw AnyMessageError("AnyMessage: type not set before read");
        }
        if (nbytes != storage_.size()) {
            throw AnyMessageError("AnyMessage: payload size mismatch on read");
        }
        std::memcpy(dst, storage_.data(), nbytes);
    }

    // ---------------------------------------------------------------------
    // Typed helpers
    // ---------------------------------------------------------------------

    template<typename T>
    void write(const T& payload, uint64_t time_nanos = 0)
    {
        ensure_type_matches<T>();
        write_bytes(&payload, sizeof(T), time_nanos);
    }

    template<typename T>
    void read(T& payload_out) const
    {
        ensure_type_matches<T>();
        read_bytes(&payload_out, sizeof(T));
    }

  private:
    template<typename T>
    void ensure_type_matches() const
    {
        const auto& info = type_info();
        if (info.size != sizeof(T) || info.align != alignof(T)) {
            throw AnyMessageError("AnyMessage: type mismatch between registered payload and requested T");
        }
    }

    MsgTypeHandle handle_ = 0;
    std::vector<std::uint8_t> storage_;
    uint64_t time_written_nanos_ = 0;
};

} // namespace messaging
} // namespace Basilisk
