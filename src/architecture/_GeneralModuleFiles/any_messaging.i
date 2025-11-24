%module(directors="0",threads="1") cAnyMessaging

%{
#include "architecture/messaging/msgTypeRegistry.h"
#include "architecture/messaging/anyMessage.h"
#include "architecture/messaging/anyMsgPort.h"

#include <string>
#include <stdexcept>
#include <Python.h>

// IMPORTANT: this must exist in the real C++ wrapper TU
namespace Basilisk {
namespace messaging {
    using BskBytes = std::string;
}
}
%}

%include "stdint.i"
%include "std_string.i"

%include "architecture/messaging/msgTypeRegistry.h"
%include "architecture/messaging/anyMessage.h"
%include "architecture/messaging/anyMsgPort.h"

%ignore Basilisk::messaging::MsgTypeRegistry::validate_info;
%ignore Basilisk::messaging::MsgTypeRegistry::global;

// Also define for SWIG's parser
%inline %{
namespace Basilisk {
namespace messaging {
    typedef std::string BskBytes;
}
}
%}

// Re-expose global registry as a helper
%inline %{
namespace Basilisk {
namespace messaging {
inline MsgTypeRegistry& global_msg_type_registry() {
    return MsgTypeRegistry::global();
}
}
}
%}

// -----------------------------------------------------------------------------
// Typemaps: Basilisk::messaging::BskBytes  <->  Python bytes/bytearray
// Limited-API safe: no Py_buffer / no buffer protocol
// -----------------------------------------------------------------------------

%typemap(typecheck) Basilisk::messaging::BskBytes {
    $1 = PyBytes_Check($input) || PyByteArray_Check($input);
}

%typemap(in) Basilisk::messaging::BskBytes {
    if (PyBytes_Check($input)) {
        char *buf = nullptr;
        Py_ssize_t n = 0;
        if (PyBytes_AsStringAndSize($input, &buf, &n) != 0) {
            SWIG_exception_fail(SWIG_TypeError, "Failed to extract bytes payload");
        }
        $1 = Basilisk::messaging::BskBytes(buf, (size_t)n);
    } else if (PyByteArray_Check($input)) {
        char *buf = PyByteArray_AS_STRING($input);
        Py_ssize_t n = PyByteArray_Size($input);
        $1 = Basilisk::messaging::BskBytes(buf, (size_t)n);
    } else {
        SWIG_exception_fail(SWIG_TypeError, "Expected bytes or bytearray");
    }
}

%typemap(out) Basilisk::messaging::BskBytes {
    const auto &s = $1;
    $result = PyBytes_FromStringAndSize(s.data(), (Py_ssize_t)s.size());
}

// -----------------------------------------------------------------------------
// Python-friendly helpers
// -----------------------------------------------------------------------------

%extend Basilisk::messaging::AnyMessage {

    void write_py(Basilisk::messaging::BskBytes data, uint64_t time_nanos = 0) {
        self->write_bytes((void const*)data.data(), (std::size_t)data.size(), time_nanos);
    }

    Basilisk::messaging::BskBytes read_py() const {
        Basilisk::messaging::BskBytes out;
        auto& reg = Basilisk::messaging::MsgTypeRegistry::global();
        const auto& info = reg.info(self->type());

        out.resize(info.size);
        if (!out.empty()) {
            self->read_bytes(&out[0], out.size());
        }
        return out;
    }
}

%extend Basilisk::messaging::AnyMsgOut {

    void write_py(Basilisk::messaging::BskBytes data, uint64_t time_nanos = 0) {
        self->message().write_bytes((void const*)data.data(), (std::size_t)data.size(), time_nanos);
    }

    Basilisk::messaging::BskBytes read_py() const {
        Basilisk::messaging::BskBytes out;
        auto& reg = Basilisk::messaging::MsgTypeRegistry::global();
        const auto& info = reg.info(self->message().type());

        out.resize(info.size);
        if (!out.empty()) {
            self->message().read_bytes(&out[0], out.size());
        }
        return out;
    }
}

%extend Basilisk::messaging::AnyMsgIn {

    Basilisk::messaging::BskBytes read_py() const {
        const Basilisk::messaging::AnyMessage& msg = self->message();
        Basilisk::messaging::BskBytes out;

        auto& reg = Basilisk::messaging::MsgTypeRegistry::global();
        const auto& info = reg.info(msg.type());

        out.resize(info.size);
        if (!out.empty()) {
            msg.read_bytes(&out[0], out.size());
        }
        return out;
    }
}
