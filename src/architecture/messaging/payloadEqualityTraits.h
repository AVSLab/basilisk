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

/*! Primary template: equality comparison is not supported for this payload type by default.
 *
 *  Native C++ code that calls ``recordOnChange()`` or ``payloadsAreEqual()`` must
 *  provide a visible ``PayloadEqualityTraits<T>`` specialization when comparison
 *  semantics can be defined safely.  The generated per-payload specializations
 *  used by Python SWIG message modules are intentionally not included through
 *  ``messaging.h`` and are not part of the native C++ messaging API.
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
        "Add a visible PayloadEqualityTraits<T> specialization when the payload "
        "comparison semantics can be defined safely."
    );
    return PayloadEqualityTraits<T>::equal(lhs, rhs);
}

#endif /* PAYLOAD_EQUALITY_TRAITS_H */
