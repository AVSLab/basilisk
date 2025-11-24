#pragma once

#include <cstdint>
#include <sstream>
#include <stdexcept>

#include "architecture/messaging/anyMessage.h"
#include "architecture/messaging/msgTypeRegistry.h"

namespace Basilisk {
namespace messaging {

class AnyMsgPortError : public std::runtime_error
{
  public:
    explicit AnyMsgPortError(const std::string& what_arg)
      : std::runtime_error(what_arg)
    {
    }
};

// Minimal dynamic-typed output port.
// Owns the underlying AnyMessage storage.
class AnyMsgOut
{
  public:
    explicit AnyMsgOut(MsgTypeHandle type_handle)
      : type_(type_handle)
      , msg_(type_handle)
    {
        if (type_handle == 0) {
            throw AnyMsgPortError("AnyMsgOut: type handle 0 is invalid");
        }
    }

    MsgTypeHandle type() const { return type_; }
    AnyMessage& message() { return msg_; }
    const AnyMessage& message() const { return msg_; }

  private:
    MsgTypeHandle type_{ 0 };
    AnyMessage msg_;
};

// Minimal dynamic-typed input port.
// Points at an upstream AnyMsgOut's AnyMessage.
class AnyMsgIn
{
  public:
    explicit AnyMsgIn(MsgTypeHandle expected_type_handle)
      : expected_type_(expected_type_handle)
    {
        if (expected_type_handle == 0) {
            throw AnyMsgPortError("AnyMsgIn: expected type handle 0 is invalid");
        }
    }

    MsgTypeHandle expected_type() const { return expected_type_; }

    bool is_connected() const { return upstream_ != nullptr; }

    // Connect this input to an upstream output.
    // Enforces type handle equality.
    void connect(AnyMsgOut& upstream)
    {
        if (upstream.type() != expected_type_) {
            std::ostringstream oss;
            const MsgTypeInfo& want = MsgTypeRegistry::global().info(expected_type_);
            const MsgTypeInfo& got = MsgTypeRegistry::global().info(upstream.type());
            oss << "AnyMsgIn::connect: type mismatch. expected " << want.name << " (handle " << expected_type_
                << "), got " << got.name << " (handle " << upstream.type() << ")";
            throw AnyMsgPortError(oss.str());
        }
        upstream_ = &upstream;
    }

    // Access the upstream message bytes.
    // Throws if not connected.
    const AnyMessage& message() const
    {
        if (!upstream_) {
            throw AnyMsgPortError("AnyMsgIn::message: not connected");
        }
        return upstream_->message();
    }

  private:
    MsgTypeHandle expected_type_{ 0 };
    AnyMsgOut* upstream_{ nullptr };
};

} // namespace messaging
} // namespace Basilisk
