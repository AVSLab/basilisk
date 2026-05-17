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

#ifndef PAYLOAD_EQUALITY_TRAITS_H
#define PAYLOAD_EQUALITY_TRAITS_H

/*! Primary template: equality comparison not supported for this payload type by default.
 *
 *  To enable recordOnChange() for a payload type, provide a specialization either:
 *   - automatically, via generatePayloadEqualityHeader.py (for fully-supported struct fields), or
 *   - manually, as a PayloadEqualityTraits<T> specialization in the payload header file
 *     (guarded by \#ifdef __cplusplus).
 */
template<typename T>
struct PayloadEqualityTraits {
    /// @brief Indicates whether equality comparison is supported for this payload type.
    static constexpr bool supported = false;
};

/*! Returns true if a field-wise equality comparison is defined for payload type T. */
template<typename T>
constexpr bool supportsPayloadEquality() noexcept {
    return PayloadEqualityTraits<T>::supported;
}

/*! Compare two payloads field-wise.
 *
 *  Will not compile for payload types that lack a PayloadEqualityTraits<T> specialization.
 *  Guard calls with supportsPayloadEquality<T>() via if constexpr, or add a specialization.
 */
template<typename T>
bool payloadsAreEqual(const T& lhs, const T& rhs) {
    static_assert(
        PayloadEqualityTraits<T>::supported,
        "payloadsAreEqual: no equality comparison is defined for this payload type. "
        "Add a PayloadEqualityTraits<T> specialization in the payload header file "
        "(guarded by #ifdef __cplusplus), or check whether generatePayloadEqualityHeader.py "
        "can cover all fields automatically."
    );
    return PayloadEqualityTraits<T>::equal(lhs, rhs);
}

// Auto-generated specializations, produced by generatePayloadEqualityHeader.py for each
// fully-supported payload type, and aggregated into payloadEquality_generated.h at build time.
// The __has_include guard prevents a hard failure during IDE indexing before the first build.
#if __has_include("payloadEquality_generated.h")
#  include "payloadEquality_generated.h"
#endif

#endif /* PAYLOAD_EQUALITY_TRAITS_H */
